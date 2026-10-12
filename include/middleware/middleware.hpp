#ifndef MIDDLEWARE_MIDDLEWARE_HPP
#define MIDDLEWARE_MIDDLEWARE_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include <functional>
#include <mutex>
#include <atomic>
#include <utility>
#include "json.hpp"

namespace middleware {

using SubscriptionId = uint64_t;
using TopicCallback = std::function<void(const JsonValue& message)>;
using ServiceHandler = std::function<std::pair<bool, JsonValue>(const JsonValue& request_args)>;

class Middleware {
private:
    struct SubscriptionEntry {
        SubscriptionId id;
        TopicCallback callback;
    };

    mutable std::mutex mtx;
    std::atomic<SubscriptionId> next_sub_id{1};
    std::unordered_map<std::string, std::vector<SubscriptionEntry>> topics;
    std::unordered_map<std::string, ServiceHandler> services;

public:
    /*
    1. Initialize the Middleware instance with empty topic and service registries.
    2. Set next subscription ID generator to 1.
    */
    Middleware() : next_sub_id(1) {
        // Associated with Step 1 & 2: Initialize default middleware state
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Generate a new unique SubscriptionId atomically.
    3. Register the subscriber callback under the specified topic name.
    4. Release lock and return the SubscriptionId.
    */
    SubscriptionId subscribe(const std::string& topic, TopicCallback callback) {
        // Associated with Step 2: Generate unique SubscriptionId
        SubscriptionId sub_id = next_sub_id.fetch_add(1);

        // Associated with Step 1: Acquire lock
        std::lock_guard<std::mutex> lock(mtx);

        // Associated with Step 3: Register callback entry in topics map
        topics[topic].push_back({sub_id, std::move(callback)});

        // Associated with Step 4: Return subscription ID
        return sub_id;
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Locate the vector of subscribers for the specified topic.
    3. Find and remove the subscription entry matching sub_id.
    4. If no subscribers remain on the topic, remove the topic entry from map.
    5. Return true if subscription was found and removed, false otherwise.
    */
    bool unsubscribe(const std::string& topic, SubscriptionId sub_id) {
        // Associated with Step 1: Acquire lock
        std::lock_guard<std::mutex> lock(mtx);

        // Associated with Step 2: Locate topic
        auto it = topics.find(topic);
        if (it == topics.end()) {
            return false;
        }

        // Associated with Step 3: Find matching subscriber ID
        auto& list = it->second;
        for (auto sub_it = list.begin(); sub_it != list.end(); ++sub_it) {
            if (sub_it->id == sub_id) {
                list.erase(sub_it);
                // Associated with Step 4: Clean up empty topic entry
                if (list.empty()) {
                    topics.erase(it);
                }
                // Associated with Step 5: Return success
                return true;
            }
        }
        return false;
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Copy all active subscriber callbacks for the target topic into a local list.
    3. Release the lock to prevent deadlock during callback execution.
    4. Iterate through the local callback list and invoke each callback with the message.
    */
    void publish(const std::string& topic, const JsonValue& message) {
        std::vector<TopicCallback> callbacks_to_invoke;
        {
            // Associated with Step 1: Acquire lock
            std::lock_guard<std::mutex> lock(mtx);

            // Associated with Step 2: Copy callbacks for the topic
            auto it = topics.find(topic);
            if (it != topics.end()) {
                callbacks_to_invoke.reserve(it->second.size());
                for (const auto& entry : it->second) {
                    callbacks_to_invoke.push_back(entry.callback);
                }
            }
        }
        // Associated with Step 3 & 4: Release lock and invoke callbacks outside lock
        for (const auto& cb : callbacks_to_invoke) {
            if (cb) {
                cb(message);
            }
        }
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Check if a service provider is already registered under service_name.
    3. If already registered, return false.
    4. Otherwise, register the handler in the services map and return true.
    */
    bool advertise_service(const std::string& service_name, ServiceHandler handler) {
        // Associated with Step 1: Acquire lock
        std::lock_guard<std::mutex> lock(mtx);

        // Associated with Step 2 & 3: Check if already registered
        if (services.find(service_name) != services.end()) {
            return false;
        }

        // Associated with Step 4: Register service handler
        services[service_name] = std::move(handler);
        return true;
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Check if service_name exists in the services map.
    3. If found, erase the entry and return true; otherwise return false.
    */
    bool unadvertise_service(const std::string& service_name) {
        // Associated with Step 1: Acquire lock
        std::lock_guard<std::mutex> lock(mtx);

        // Associated with Step 2 & 3: Locate and erase service
        auto it = services.find(service_name);
        if (it != services.end()) {
            services.erase(it);
            return true;
        }
        return false;
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Look up the service handler by service_name.
    3. If service is not registered, release lock and return pair(false, error_message).
    4. Copy the service handler and release the lock.
    5. Execute the service handler with request_args and return its result pair.
    */
    std::pair<bool, JsonValue> call_service(const std::string& service_name, const JsonValue& request_args) {
        ServiceHandler handler;
        {
            // Associated with Step 1: Acquire lock
            std::lock_guard<std::mutex> lock(mtx);

            // Associated with Step 2 & 3: Look up service handler
            auto it = services.find(service_name);
            if (it == services.end()) {
                JsonValue err = JsonValue::make_object();
                err["error"] = "Service '" + service_name + "' not found";
                return {false, err};
            }
            // Associated with Step 4: Copy handler
            handler = it->second;
        }

        // Associated with Step 5: Invoke handler outside lock and return result
        try {
            return handler(request_args);
        } catch (const std::exception& e) {
            JsonValue err = JsonValue::make_object();
            err["error"] = std::string("Service exception: ") + e.what();
            return {false, err};
        }
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Check if the specified topic exists and has at least one active subscriber.
    3. Return true if topic has active subscribers, false otherwise.
    */
    bool has_subscribers(const std::string& topic) const {
        // Associated with Step 1: Acquire lock
        std::lock_guard<std::mutex> lock(mtx);

        // Associated with Step 2 & 3: Check topic presence and count
        auto it = topics.find(topic);
        return it != topics.end() && !it->second.empty();
    }

    /*
    1. Acquire lock on the internal mutex.
    2. Check if service_name is registered in the services map.
    3. Return true if registered, false otherwise.
    */
    bool has_service(const std::string& service_name) const {
        // Associated with Step 1: Acquire lock
        std::lock_guard<std::mutex> lock(mtx);

        // Associated with Step 2 & 3: Check service presence
        return services.find(service_name) != services.end();
    }
};

} // namespace middleware

#endif // MIDDLEWARE_MIDDLEWARE_HPP
