#ifndef MIDDLEWARE_ROSBRIDGE_SERVER_HPP
#define MIDDLEWARE_ROSBRIDGE_SERVER_HPP

#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include "middleware.hpp"
#include "json.hpp"

namespace middleware {

class RosbridgeServer {
private:
    struct ClientConnection {
        std::atomic<int> fd{-1};
        std::shared_ptr<std::mutex> write_mtx{std::make_shared<std::mutex>()};
        std::vector<std::pair<std::string, SubscriptionId>> subscriptions;
        // Publisher registrations are owned by one TCP connection.  A publish
        // request is valid only while its topic remains in this set.
        std::unordered_set<std::string> advertised_topics;
        std::vector<std::string> provided_services;
        std::atomic<bool> finished{false};
    };

    struct PendingServiceCall {
        std::shared_ptr<ClientConnection> caller_conn;
        std::shared_ptr<ClientConnection> provider_conn;
        JsonValue caller_id;
        std::string service;
        std::chrono::steady_clock::time_point deadline;
    };

    Middleware& mw;
    std::string host{"127.0.0.1"};
    int port{9095};
    std::atomic<bool> is_running{false};
    int server_fd{-1};
    std::thread accept_thread;
    std::thread timeout_thread;
    std::vector<std::pair<std::thread, std::shared_ptr<ClientConnection>>> client_threads;
    std::mutex clients_mtx;
    std::unordered_map<int, std::shared_ptr<ClientConnection>> clients;

    // External service providers registered via advertise_service over TCP
    std::mutex services_mtx;
    std::unordered_map<std::string, std::shared_ptr<ClientConnection>> external_service_providers;
    std::atomic<uint64_t> next_call_id{1};
    std::unordered_map<std::string, PendingServiceCall> pending_calls;

public:
    /*
    1. Initialize RosbridgeServer with reference to internal Middleware.
    2. Set default host to 127.0.0.1 and port to 9095.
    */
    explicit RosbridgeServer(Middleware& middleware, std::string h = "127.0.0.1", int p = 9095)
        : mw(middleware), host(std::move(h)), port(p), is_running(false), server_fd(-1) {
        // Associated with Step 1 & 2: Store middleware reference and server address configuration
    }

    /*
    1. Check if server is running, and invoke stop() if active.
    */
    ~RosbridgeServer() {
        // Associated with Step 1: Ensure server is cleanly stopped on destruction
        stop();
    }

    /*
    1. Check if server is already running; if so, return true.
    2. Create a TCP socket using AF_INET and SOCK_STREAM.
    3. Configure SO_REUSEADDR on the socket.
    4. Bind the socket to the configured host IP and port.
    5. Place the socket in listening mode with a connection backlog.
    6. Mark is_running as true and spawn accept_thread to handle incoming connections.
    7. Return true on success or false on failure.
    */
    bool start() {
        // Associated with Step 1: Return true if already running
        if (is_running.load()) {
            return true;
        }

        // Associated with Step 2: Create TCP socket
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) {
            return false;
        }

        // Associated with Step 3: Enable address reuse
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Associated with Step 4: Configure socket address and bind
        struct sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) <= 0) {
            close(server_fd);
            server_fd = -1;
            return false;
        }

        if (bind(server_fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
            close(server_fd);
            server_fd = -1;
            return false;
        }

        // Associated with Step 5: Start listening for connections
        if (listen(server_fd, 64) < 0) {
            close(server_fd);
            server_fd = -1;
            return false;
        }

        // Associated with Step 6: Mark running and launch accept thread
        is_running.store(true);
        accept_thread = std::thread(&RosbridgeServer::accept_loop, this);
        // *CHANGES*: Bound externally provided service calls even when clients are idle.
        timeout_thread = std::thread(&RosbridgeServer::timeout_loop, this);

        // Associated with Step 7: Return success
        return true;
    }

    /*
    1. Check if server is currently running; if not, return immediately.
    2. Set is_running flag to false.
    3. Close the server listening socket to unblock accept().
    4. Close all active client connection sockets under clients_mtx lock.
    5. Join the accept thread.
    6. Join all client worker threads.
    7. Clear active client and thread lists.
    */
    void stop() {
        // Associated with Step 1: Check if running
        if (!is_running.exchange(false)) {
            return;
        }

        // Associated with Step 3: Close listening socket
        if (server_fd >= 0) {
            shutdown(server_fd, SHUT_RDWR);
            close(server_fd);
            server_fd = -1;
        }

        // Associated with Step 4: Disconnect all active clients
        {
            std::lock_guard<std::mutex> lock(clients_mtx);
            for (auto& pair : clients) {
                if (pair.second && pair.second->fd >= 0) {
                    shutdown(pair.second->fd, SHUT_RDWR);
                }
            }
        }

        // Associated with Step 5: Join accept thread
        if (accept_thread.joinable()) {
            accept_thread.join();
        }
        if (timeout_thread.joinable()) {
            timeout_thread.join();
        }

        // Associated with Step 6: Join client worker threads
        for (auto& entry : client_threads) {
            if (entry.first.joinable()) {
                entry.first.join();
            }
        }

        // Associated with Step 7: Clear client records
        {
            std::lock_guard<std::mutex> lock(clients_mtx);
            clients.clear();
            client_threads.clear();
        }
    }

    /*
    1. Query whether the gateway server is currently running.
    */
    bool running() const noexcept {
        // Associated with Step 1: Return running status
        return is_running.load();
    }

    /*
    1. Retrieve the configured listening port.
    */
    int get_port() const noexcept {
        // Associated with Step 1: Return port number
        return port;
    }

private:
    /*
    1. Continuously accept incoming TCP client connections while is_running is true.
    2. For each accepted socket, create a ClientConnection tracking object.
    3. Spawn a worker thread to handle client messages.
    4. Store client record under clients_mtx.
    */
    void accept_loop() {
        while (is_running.load()) {
            struct sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            // Associated with Step 1: Accept incoming connection
            int client_fd = accept(server_fd, reinterpret_cast<struct sockaddr*>(&client_addr), &client_len);
            if (client_fd < 0) {
                if (!is_running.load()) break;
                continue;
            }

            // Associated with Step 2: Create connection object
            auto conn = std::make_shared<ClientConnection>();
            conn->fd = client_fd;
            // *CHANGES*: Bound writes to a stalled peer so it cannot hold topic delivery forever.
            timeval write_timeout{};
            write_timeout.tv_sec = 1;
            setsockopt(client_fd, SOL_SOCKET, SO_SNDTIMEO, &write_timeout, sizeof(write_timeout));
#ifdef SO_NOSIGPIPE
            int no_sigpipe = 1;
            setsockopt(client_fd, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe, sizeof(no_sigpipe));
#endif

            // Associated with Step 4: Register client connection
            {
                std::lock_guard<std::mutex> lock(clients_mtx);
                clients[client_fd] = conn;
            }

            // Associated with Step 3: Launch client handler thread
            // *CHANGES*: Reap disconnected client threads during repeated connection cycles.
            for (auto it = client_threads.begin(); it != client_threads.end();) {
                if (it->second->finished.load()) {
                    it->first.join();
                    it = client_threads.erase(it);
                } else {
                    ++it;
                }
            }
            client_threads.emplace_back(std::thread(&RosbridgeServer::client_loop, this, conn), conn);
        }
    }

    /*
    1. Send a string message followed by newline '\n' to a client socket.
    2. Ensure atomic writes using the client connection's write_mtx.
    3. Return true if send was successful, false on error.
    */
    static bool send_message(const std::shared_ptr<ClientConnection>& conn, const std::string& line) {
        if (!conn) return false;
        // Associated with Step 2: Acquire client write mutex
        std::lock_guard<std::mutex> lock(*conn->write_mtx);
        int fd = conn->fd.load();
        if (fd < 0) return false;

        std::string payload = line + "\n";
        const char* buf = payload.data();
        size_t total = payload.size();
        size_t sent = 0;

        // Associated with Step 1 & 3: Send all bytes over socket
        while (sent < total) {
            // *CHANGES*: A disconnected peer must not terminate the whole gateway via SIGPIPE.
#ifdef MSG_NOSIGNAL
            ssize_t n = send(fd, buf + sent, total - sent, MSG_NOSIGNAL);
#else
            ssize_t n = send(fd, buf + sent, total - sent, 0);
#endif
            if (n <= 0) {
                // *CHANGES*: Discard a connection after a partial frame fails to send.
                shutdown(fd, SHUT_RDWR);
                return false;
            }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    /*
    1. Send a diagnostic status object conforming to ROSBRIDGE_PROTOCOL.md:
       {"op":"status","level":level,"msg":msg,"id":id}
    2. Omit the id field if the triggering request did not include one.
    */
    static void send_status(const std::shared_ptr<ClientConnection>& conn,
                            const std::string& level,
                            const std::string& msg,
                            const JsonValue* id = nullptr) {
        JsonValue status = JsonValue::make_object();
        status["op"] = "status";
        status["level"] = level;
        status["msg"] = msg;
        if (id) {
            status["id"] = *id;
        }
        send_message(conn, status.dump());
    }

    /*
    1. Read incoming bytes from the client socket into a buffer.
    2. Extract individual newline-delimited ('\n') JSON lines.
    3. Parse each JSON command and invoke process_operation().
    4. When client disconnects or error occurs, call cleanup_client() to remove all subscriptions.
    */
    void client_loop(std::shared_ptr<ClientConnection> conn) {
        std::string buffer;
        char temp[4096];

        // Associated with Step 1: Read loop
        while (is_running.load() && conn->fd >= 0) {
            ssize_t bytes_read = recv(conn->fd.load(), temp, sizeof(temp), 0);
            if (bytes_read <= 0) {
                break; // Disconnection or socket error
            }
            buffer.append(temp, static_cast<size_t>(bytes_read));

            // Associated with Step 2: Extract newline-delimited messages
            size_t pos = 0;
            while ((pos = buffer.find('\n')) != std::string::npos) {
                std::string line = buffer.substr(0, pos);
                buffer.erase(0, pos + 1);

                // Trim trailing carriage return if present
                if (!line.empty() && line.back() == '\r') {
                    line.pop_back();
                }
                if (line.empty()) continue;

                // Associated with Step 3: Parse and process JSON message
                try {
                    JsonValue op_msg = JsonValue::parse(line);
                    process_operation(conn, op_msg);
                } catch (const std::exception& e) {
                    // Send error status on malformed JSON
                    send_status(conn, "error", std::string("Malformed JSON request: ") + e.what(), nullptr);
                }
            }
        }

        // Associated with Step 4: Clean up subscriptions and connection state on disconnect
        cleanup_client(conn);
        conn->finished.store(true);
    }

    /*
    1. Read the "op" field from the parsed JSON command.
    2. Handle "subscribe": register subscription on Middleware and send informational status
       conforming to ROSBRIDGE_PROTOCOL.md line 35.
    3. Handle "unsubscribe": withdraw subscription from Middleware.
    4. Handle "publish": dispatch payload to Middleware topic.
    5. Handle "advertise" & "unadvertise": register/withdraw publisher.
    6. Handle "advertise_service" & "unadvertise_service": register/withdraw service provider and send status.
    7. Handle "call_service": route call to internal Middleware or external provider and return service_response.
    8. Handle "service_response": forward response from external provider back to caller.
    9. For unknown operations, send error status conforming to ROSBRIDGE_PROTOCOL.md line 58.
    */
    void process_operation(const std::shared_ptr<ClientConnection>& conn, const JsonValue& msg) {
        if (!msg.is_object() || !msg.contains("op")) {
            send_status(conn, "error", "Request must be a JSON object containing an 'op' field", nullptr);
            return;
        }
        std::string op = msg["op"].as_string();
        const JsonValue* req_id = msg.contains("id") ? &msg["id"] : nullptr;

        // Associated with Step 2: Handle "subscribe"
        if (op == "subscribe") {
            if (!msg.contains("topic")) {
                send_status(conn, "error", "subscribe requires 'topic' field", req_id);
                return;
            }
            std::string topic = msg["topic"].as_string();

            // *CHANGES*: Repeated subscriptions on one connection are idempotent.
            for (const auto& sub : conn->subscriptions) {
                if (sub.first == topic) {
                    send_status(conn, "info", "subscribed to " + topic, req_id);
                    return;
                }
            }

            // Weak pointer to connection prevents keeping dead connection alive in lambda
            std::weak_ptr<ClientConnection> weak_conn = conn;

            SubscriptionId sub_id = mw.subscribe(topic, [weak_conn, topic](const JsonValue& payload) {
                auto active_conn = weak_conn.lock();
                if (active_conn) {
                    JsonValue response = JsonValue::make_object();
                    response["op"] = "publish";
                    response["topic"] = topic;
                    response["msg"] = payload;
                    send_message(active_conn, response.dump());
                }
            });

            // Store subscription under client's registered list
            conn->subscriptions.emplace_back(topic, sub_id);

            // *CHANGES*: Send required informational status conforming to ROSBRIDGE_PROTOCOL.md line 35:
            // {"op":"status","level":"info","msg":"subscribed to /topic","id":"sub-1"}
            send_status(conn, "info", "subscribed to " + topic, req_id);

        // Associated with Step 3: Handle "unsubscribe"
        } else if (op == "unsubscribe") {
            if (!msg.contains("topic")) {
                send_status(conn, "error", "unsubscribe requires 'topic' field", req_id);
                return;
            }
            std::string topic = msg["topic"].as_string();
            for (auto it = conn->subscriptions.begin(); it != conn->subscriptions.end(); ++it) {
                if (it->first == topic) {
                    mw.unsubscribe(topic, it->second);
                    conn->subscriptions.erase(it);
                    break;
                }
            }
            if (req_id) {
                send_status(conn, "info", "unsubscribed from " + topic, req_id);
            }

        // Associated with Step 4: Handle "publish"
        } else if (op == "publish") {
            if (!msg.contains("topic") || !msg.contains("msg")) {
                send_status(conn, "error", "publish requires 'topic' and 'msg' fields", req_id);
                return;
            }
            std::string topic = msg["topic"].as_string();
            if (conn->advertised_topics.find(topic) == conn->advertised_topics.end()) {
                send_status(conn, "error", "publish requires an active advertisement for " + topic, req_id);
                return;
            }
            mw.publish(topic, msg["msg"]);

        // Associated with Step 5: Handle "advertise" & "unadvertise"
        } else if (op == "advertise") {
            if (msg.contains("topic")) {
                conn->advertised_topics.insert(msg["topic"].as_string());
            }
            if (req_id) {
                std::string topic = msg.contains("topic") ? msg["topic"].as_string() : "";
                send_status(conn, "info", "advertised " + topic, req_id);
            }
        } else if (op == "unadvertise") {
            if (msg.contains("topic")) {
                conn->advertised_topics.erase(msg["topic"].as_string());
            }
            if (req_id) {
                std::string topic = msg.contains("topic") ? msg["topic"].as_string() : "";
                send_status(conn, "info", "unadvertised " + topic, req_id);
            }

        // Associated with Step 6: Handle "advertise_service" & "unadvertise_service"
        } else if (op == "advertise_service") {
            if (!msg.contains("service")) {
                send_status(conn, "error", "advertise_service requires 'service' field", req_id);
                return;
            }
            std::string service = msg["service"].as_string();

            {
                std::lock_guard<std::mutex> lock(services_mtx);
                external_service_providers[service] = conn;
                conn->provided_services.push_back(service);
            }
            // *CHANGES*: Send required informational status conforming to ROSBRIDGE_PROTOCOL.md line 51:
            // "Send an informational status after a service registration completes."
            send_status(conn, "info", "service registered: " + service, req_id);

        } else if (op == "unadvertise_service") {
            if (!msg.contains("service")) {
                send_status(conn, "error", "unadvertise_service requires 'service' field", req_id);
                return;
            }
            std::string service = msg["service"].as_string();

            {
                std::lock_guard<std::mutex> lock(services_mtx);
                auto it = external_service_providers.find(service);
                if (it != external_service_providers.end() && it->second == conn) {
                    external_service_providers.erase(it);
                }
            }
            if (req_id) {
                send_status(conn, "info", "service unregistered: " + service, req_id);
            }

        // Associated with Step 7: Handle "call_service"
        } else if (op == "call_service") {
            if (!msg.contains("service")) {
                send_status(conn, "error", "call_service requires 'service' field", req_id);
                return;
            }
            std::string service = msg["service"].as_string();
            JsonValue args = msg.contains("args") ? msg["args"] : JsonValue::make_object();

            // 1. Check if service is registered on internal Middleware (e.g. /heapify, /heap_sort)
            if (mw.has_service(service)) {
                std::pair<bool, JsonValue> result = mw.call_service(service, args);
                JsonValue resp = JsonValue::make_object();
                resp["op"] = "service_response";
                resp["service"] = service;
                resp["values"] = result.second;
                resp["result"] = result.first;
                resp["status"] = result.first ? "" : (result.second.contains("error") ? result.second["error"].as_string() : "error");
                if (req_id) {
                    resp["id"] = *req_id;
                }
                send_message(conn, resp.dump());
                return;
            }

            // 2. Check if service is provided by an external client connection
            std::shared_ptr<ClientConnection> provider_conn;
            {
                std::lock_guard<std::mutex> lock(services_mtx);
                auto it = external_service_providers.find(service);
                if (it != external_service_providers.end()) {
                    provider_conn = it->second;
                }
            }

            if (provider_conn && provider_conn->fd >= 0) {
                std::string provider_call_id = "provider-call-" + std::to_string(next_call_id.fetch_add(1));
                {
                    std::lock_guard<std::mutex> lock(services_mtx);
                    pending_calls[provider_call_id] = {
                        conn, provider_conn, req_id ? *req_id : JsonValue(), service,
                        std::chrono::steady_clock::now() + std::chrono::seconds(5)};
                }

                // Forward call_service to provider
                JsonValue fwd = JsonValue::make_object();
                fwd["op"] = "call_service";
                fwd["service"] = service;
                fwd["args"] = args;
                fwd["id"] = provider_call_id;
                if (!send_message(provider_conn, fwd.dump())) {
                    PendingServiceCall pending;
                    bool found = false;
                    {
                        std::lock_guard<std::mutex> lock(services_mtx);
                        auto it = pending_calls.find(provider_call_id);
                        if (it != pending_calls.end()) {
                            pending = it->second;
                            pending_calls.erase(it);
                            found = true;
                        }
                    }
                    if (found) send_failed_call(pending, "service provider disconnected");
                }
            } else {
                // Missing provider: return clean result:false with original caller ID (ROSBRIDGE_PROTOCOL.md line 55)
                JsonValue resp = JsonValue::make_object();
                resp["op"] = "service_response";
                resp["service"] = service;
                resp["values"] = JsonValue::make_object();
                resp["result"] = false;
                resp["status"] = "service not found";
                if (req_id) {
                    resp["id"] = *req_id;
                }
                send_message(conn, resp.dump());
            }

        // Associated with Step 8: Handle "service_response" from an external provider
        } else if (op == "service_response") {
            if (!msg.contains("id")) return;
            std::string prov_id = msg["id"].is_string() ? msg["id"].as_string() : msg["id"].dump();

            PendingServiceCall pending;
            bool found = false;
            {
                std::lock_guard<std::mutex> lock(services_mtx);
                auto it = pending_calls.find(prov_id);
                // *CHANGES*: Only the provider assigned this call may complete it.
                if (it != pending_calls.end() && it->second.provider_conn == conn &&
                    msg.contains("service") && msg["service"].is_string() &&
                    msg["service"].as_string() == it->second.service &&
                    msg.contains("values") && msg.contains("result") && msg["result"].is_bool()) {
                    pending = it->second;
                    pending_calls.erase(it);
                    found = true;
                }
            }

            if (found && pending.caller_conn) {
                JsonValue caller_resp = JsonValue::make_object();
                caller_resp["op"] = "service_response";
                caller_resp["service"] = pending.service;
                caller_resp["values"] = msg["values"];
                caller_resp["result"] = msg["result"];
                caller_resp["status"] = msg.contains("status") ? msg["status"] : JsonValue("");
                caller_resp["id"] = pending.caller_id;
                send_message(pending.caller_conn, caller_resp.dump());
            }

        // Associated with Step 9: Unknown operation produce an error status (ROSBRIDGE_PROTOCOL.md line 58)
        } else {
            send_status(conn, "error", "unknown operation: " + op, req_id);
        }
    }

    /*
    1. Unregister all subscriptions associated with the disconnected client from Middleware.
    2. Withdraw any external services provided by this client.
    3. Notify callers of any pending service requests with result: false.
    4. Close the client socket if open.
    5. Remove client record from active clients map under clients_mtx lock.
    */
    void cleanup_client(const std::shared_ptr<ClientConnection>& conn) {
        if (!conn) return;

        // Associated with Step 1: Remove all subscriptions belonging to this client
        for (const auto& sub : conn->subscriptions) {
            mw.unsubscribe(sub.first, sub.second);
        }
        conn->subscriptions.clear();

        // Associated with Step 2 & 3: Remove provided services and fail pending calls
        std::vector<PendingServiceCall> failed_calls;
        {
            std::lock_guard<std::mutex> lock(services_mtx);
            for (const auto& srv : conn->provided_services) {
                auto it = external_service_providers.find(srv);
                if (it != external_service_providers.end() && it->second == conn) {
                    external_service_providers.erase(it);
                }
            }
            conn->provided_services.clear();

            // Fail pending calls waiting on this client
            for (auto it = pending_calls.begin(); it != pending_calls.end(); ) {
                if (it->second.provider_conn == conn) {
                    failed_calls.push_back(it->second);
                    it = pending_calls.erase(it);
                } else if (it->second.caller_conn == conn) {
                    it = pending_calls.erase(it);
                } else {
                    ++it;
                }
            }
        }
        for (const auto& call : failed_calls) {
            send_failed_call(call, "service provider disconnected");
        }

        // Associated with Step 4 & 5: Close socket and erase from map
        int fd;
        {
            std::lock_guard<std::mutex> write_lock(*conn->write_mtx);
            fd = conn->fd.exchange(-1);
            if (fd >= 0) close(fd);
        }
        if (fd >= 0) {
            std::lock_guard<std::mutex> lock(clients_mtx);
            clients.erase(fd);
        }
    }

    // *CHANGES*: Return bounded failures using the original caller's opaque ID.
    static void send_failed_call(const PendingServiceCall& call, const std::string& reason) {
        JsonValue response = JsonValue::make_object();
        response["op"] = "service_response";
        response["service"] = call.service;
        response["id"] = call.caller_id;
        response["values"] = JsonValue::make_object();
        response["result"] = false;
        response["status"] = reason;
        send_message(call.caller_conn, response.dump());
    }

    void timeout_loop() {
        while (is_running.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::vector<PendingServiceCall> expired;
            {
                std::lock_guard<std::mutex> lock(services_mtx);
                auto now = std::chrono::steady_clock::now();
                for (auto it = pending_calls.begin(); it != pending_calls.end();) {
                    if (it->second.deadline <= now) {
                        expired.push_back(it->second);
                        it = pending_calls.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            for (const auto& call : expired) send_failed_call(call, "service timeout");
        }
    }
};

} // namespace middleware

#endif // MIDDLEWARE_ROSBRIDGE_SERVER_HPP
