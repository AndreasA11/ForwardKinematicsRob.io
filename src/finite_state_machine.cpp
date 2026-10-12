#include "finite_state_machine.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <set>

namespace robot {
namespace {
using middleware::JsonValue;
using Reply = std::pair<bool, JsonValue>;

Reply failure(const std::string& reason) {
    JsonValue values = JsonValue::make_object();
    values["error"] = reason;
    return {false, values};
}

bool identifier_valid(const JsonValue& value) {
    if (!value.is_string()) return false;
    const std::string& name = value.as_string();
    if (name.empty() || name.size() > 64) return false;
    return std::all_of(name.begin(), name.end(), [](unsigned char ch) {
        return (ch >= 'A' && ch <= 'Z') || (ch >= 'a' && ch <= 'z') ||
               (ch >= '0' && ch <= '9') || ch == '_' || ch == '-';
    });
}

JsonValue document_only(const JsonValue& request) {
    JsonValue document = JsonValue::make_object();
    document["initial_state"] = request["initial_state"];
    document["states"] = request["states"];
    return document;
}

std::filesystem::path safe_file(const std::string& identifier) {
    const std::filesystem::path directory("fsm");
    if (std::filesystem::is_symlink(directory)) throw std::runtime_error("fsm directory is a symlink");
    std::filesystem::create_directories(directory);
    const auto path = directory / (identifier + ".json");
    if (std::filesystem::is_symlink(path)) throw std::runtime_error("saved file is a symlink");
    return path;
}
} // namespace

FiniteStateMachine::FiniteStateMachine(middleware::Middleware& middleware)
    : middleware_(middleware) {}

FiniteStateMachine::~FiniteStateMachine() { stop(); }

Reply FiniteStateMachine::validate(const JsonValue& document,
                                    std::unordered_map<std::string, State>& states) {
    if (!document.is_object() || !document.contains("initial_state") ||
        !document["initial_state"].is_string() || !document.contains("states") ||
        !document["states"].is_array() || document["states"].as_array().empty())
        return failure("initial_state and nonempty states are required");
    for (const auto& item : document["states"].as_array()) {
        if (!item.is_object() || !item.contains("name") || !item["name"].is_string() ||
            !item.contains("target") || !item["target"].is_object() ||
            !item.contains("epsilon") || !item["epsilon"].is_number() ||
            !std::isfinite(item["epsilon"].as_number()) || item["epsilon"].as_number() < 0 ||
            !item.contains("transitions") || !item["transitions"].is_array())
            return failure("invalid state fields");
        State state;
        state.name = item["name"].as_string();
        state.epsilon = item["epsilon"].as_number();
        if (state.name.empty() || states.contains(state.name)) return failure("duplicate or empty state name");
        const auto& target = item["target"];
        if (!target.contains("joint_names") || !target["joint_names"].is_array() ||
            !target.contains("positions") || !target["positions"].is_array() ||
            target["joint_names"].as_array().size() != target["positions"].as_array().size())
            return failure("target joint_names and positions must have matching arrays");
        const auto& target_joint_names = target["joint_names"].as_array();
        const auto& target_positions = target["positions"].as_array();
        for (std::size_t i = 0; i < target_joint_names.size(); ++i) {
            const auto& name = target_joint_names[i];
            const auto& position = target_positions[i];
            if (!name.is_string() || !position.is_number() || !std::isfinite(position.as_number()))
                return failure("target joints require names and finite positions");
            state.joint_names.push_back(name.as_string());
            state.positions.push_back(position.as_number());
        }
        for (const auto& transition : item["transitions"].as_array()) {
            if (!transition.is_object() || !transition.contains("name") ||
                !transition["name"].is_string() || !transition.contains("next_state") ||
                !transition["next_state"].is_string()) return failure("invalid transition");
            const std::string name = transition["name"].as_string();
            if (name.empty() || !state.transitions.emplace(name, transition["next_state"].as_string()).second)
                return failure("duplicate or empty transition name");
        }
        if (item.contains("on_arrival")) {
            if (item["on_arrival"].is_string()) {
                state.on_arrival = item["on_arrival"].as_string();
                if (!state.transitions.contains(state.on_arrival)) return failure("on_arrival is unknown");
            } else if (!item["on_arrival"].is_null()) {
                return failure("on_arrival must name a transition");
            }
        }
        states.emplace(state.name, std::move(state));
    }
    if (!states.contains(document["initial_state"].as_string())) return failure("initial_state is unknown");
    for (const auto& [name, state] : states)
        for (const auto& [transition, next] : state.transitions)
            if (!states.contains(next)) return failure("transition targets an unknown state");
    return {true, JsonValue::make_object()};
}

JsonValue FiniteStateMachine::enter(Shared& shared,
                                    const std::string& next, const JsonValue& transition) {
    const State& state = shared.states.at(next);
    shared.current_state = next;
    shared.last_transition = transition;
    shared.entered_at = std::chrono::steady_clock::now();
    JsonValue trajectory = JsonValue::make_object();
    JsonValue names = JsonValue::make_array();
    JsonValue positions = JsonValue::make_array();
    for (std::size_t i = 0; i < state.joint_names.size(); ++i) {
        names.push_back(state.joint_names[i]);
        positions.push_back(state.positions[i]);
    }
    JsonValue point = JsonValue::make_object();
    point["positions"] = std::move(positions);
    point["velocities"] = JsonValue::make_array();
    point["accelerations"] = JsonValue::make_array();
    JsonValue duration = JsonValue::make_object();
    duration["sec"] = 0;
    duration["nanosec"] = 0;
    point["time_from_start"] = std::move(duration);
    JsonValue points = JsonValue::make_array();
    points.push_back(std::move(point));
    trajectory["joint_names"] = std::move(names);
    trajectory["points"] = std::move(points);
    return trajectory;
}

void FiniteStateMachine::observe(Shared& shared, middleware::Middleware& middleware,
                                  const JsonValue& message) {
    if (!message.is_object() || !message.contains("name") || !message["name"].is_array() ||
        !message.contains("position") || !message["position"].is_array()) return;
    std::unordered_map<std::string, double> observed;
    const auto& names = message["name"].as_array();
    const auto& positions = message["position"].as_array();
    if (names.size() != positions.size()) return;
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i].is_string() && positions[i].is_number() && std::isfinite(positions[i].as_number()))
            observed[names[i].as_string()] = positions[i].as_number();

    JsonValue trajectory;
    bool should_publish = false;
    {
        std::lock_guard<std::mutex> lock(shared.mutex);
        if (!shared.running || shared.current_state.empty()) return;
        const State& state = shared.states.at(shared.current_state);
        if (state.on_arrival.empty()) return;
        for (std::size_t i = 0; i < state.joint_names.size(); ++i) {
            auto it = observed.find(state.joint_names[i]);
            if (it == observed.end() || std::abs(it->second - state.positions[i]) > state.epsilon) return;
        }
        const std::string transition = state.on_arrival;
        trajectory = enter(shared, state.transitions.at(transition), JsonValue(transition));
        should_publish = true;
    }
    if (should_publish) {
        middleware.publish("/joint_trajectory", trajectory);
    }
}

void FiniteStateMachine::start() {
    if (started_.exchange(true)) return;
    joint_subscription_ = middleware_.subscribe("/joint_states", [shared = shared_, &mw = middleware_](const JsonValue& msg) {
        observe(*shared, mw, msg);
    });
    middleware_.advertise_service("/fsm/update_logic", [shared = shared_, &mw = middleware_](const JsonValue& args) -> Reply {
        std::unordered_map<std::string, State> states;
        Reply valid = validate(args, states);
        if (!valid.first) return valid;
        JsonValue trajectory;
        std::string current;
        {
            std::lock_guard<std::mutex> lock(shared->mutex);
            shared->document = document_only(args);
            shared->states = std::move(states);
            shared->running = true;
            trajectory = enter(*shared, args["initial_state"].as_string(), JsonValue());
            current = shared->current_state;
        }
        mw.publish("/joint_trajectory", trajectory);
        JsonValue values = JsonValue::make_object();
        values["current_state"] = current;
        return {true, values};
    });
    middleware_.advertise_service("/fsm/get_logic", [shared = shared_](const JsonValue&) -> Reply {
        std::lock_guard<std::mutex> lock(shared->mutex);
        if (shared->current_state.empty()) return failure("no logic loaded");
        return {true, shared->document};
    });
    middleware_.advertise_service("/fsm/set_running", [shared = shared_, &mw = middleware_](const JsonValue& args) -> Reply {
        JsonValue trajectory;
        bool should_publish = false;
        bool running_val = false;
        {
            std::lock_guard<std::mutex> lock(shared->mutex);
            if (args.is_object() && args.contains("data")) {
                if (!args["data"].is_bool()) return failure("data must be boolean");
                if (args["data"].as_bool() && shared->current_state.empty()) return failure("no logic loaded");
                const bool was_running = shared->running;
                shared->running = args["data"].as_bool();
                if (shared->running && !was_running) {
                    trajectory = enter(*shared, shared->current_state, shared->last_transition);
                    should_publish = true;
                }
            }
            running_val = shared->running;
        }
        if (should_publish) {
            mw.publish("/joint_trajectory", trajectory);
        }
        JsonValue values = JsonValue::make_object();
        values["running"] = running_val;
        return {true, values};
    });
    middleware_.advertise_service("/fsm/fire_transition", [shared = shared_, &mw = middleware_](const JsonValue& args) -> Reply {
        if (!args.is_object() || !args.contains("transition") || !args["transition"].is_string())
            return failure("transition name is required");
        JsonValue trajectory;
        std::string next;
        {
            std::lock_guard<std::mutex> lock(shared->mutex);
            if (!shared->running) return failure("FSM is stopped");
            const auto& transitions = shared->states.at(shared->current_state).transitions;
            auto it = transitions.find(args["transition"].as_string());
            if (it == transitions.end()) return failure("transition is unknown in current state");
            next = it->second;
            trajectory = enter(*shared, next, args["transition"]);
        }
        mw.publish("/joint_trajectory", trajectory);
        JsonValue values = JsonValue::make_object();
        values["current_state"] = next;
        return {true, values};
    });
    middleware_.advertise_service("/fsm/save_to_file", [](const JsonValue& args) -> Reply {
        if (!args.is_object() || !args.contains("identifier") || !identifier_valid(args["identifier"]))
            return failure("identifier is required and must be valid");
        try {
            const auto path = safe_file(args["identifier"].as_string());
            std::ofstream output(path, std::ios::binary | std::ios::trunc);
            if (!output) return failure("could not open saved file");
            JsonValue document = JsonValue::make_object();
            if (args.contains("document") && args["document"].is_object()) {
                document = args["document"];
            } else {
                for (const auto& [k, v] : args.as_object()) {
                    if (k != "identifier") {
                        document[k] = v;
                    }
                }
            }
            output << document.dump() << '\n';
            if (!output) return failure("could not write saved file");
            JsonValue values = JsonValue::make_object();
            values["path"] = path.generic_string();
            return {true, values};
        } catch (const std::exception& error) { return failure(error.what()); }
    });
    middleware_.advertise_service("/fsm/load_from_file", [](const JsonValue& args) -> Reply {
        if (!args.is_object() || !args.contains("identifier") || !identifier_valid(args["identifier"]))
            return failure("invalid identifier");
        try {
            const auto path = safe_file(args["identifier"].as_string());
            std::ifstream input(path, std::ios::binary);
            if (!input) return failure("saved document not found");
            const std::string contents(std::istreambuf_iterator<char>{input}, {});
            const JsonValue document = JsonValue::parse(contents);
            if (!document.is_object())
                return failure("saved document is invalid");
            return {true, document};
        } catch (const std::exception& error) { return failure(error.what()); }
    });
    middleware_.advertise_service("/fsm/list_saved", [](const JsonValue&) -> Reply {
        JsonValue values = JsonValue::make_object();
        JsonValue identifiers = JsonValue::make_array();
        try {
            const auto directory = std::filesystem::path("fsm");
            if (std::filesystem::is_symlink(directory)) return failure("fsm directory is a symlink");
            if (std::filesystem::exists(directory)) {
                std::set<std::string> names;
                for (const auto& entry : std::filesystem::directory_iterator(directory)) {
                    if (!entry.is_regular_file() || entry.is_symlink() || entry.path().extension() != ".json") continue;
                    const std::string stem = entry.path().stem().string();
                    if (identifier_valid(JsonValue(stem))) names.insert(stem);
                }
                for (const auto& name : names) identifiers.push_back(name);
            }
            values["identifiers"] = std::move(identifiers);
            return {true, values};
        } catch (const std::exception& error) { return failure(error.what()); }
    });
    worker_ = std::thread(&FiniteStateMachine::worker_loop, this);
}

void FiniteStateMachine::stop() {
    if (!started_.exchange(false)) return;
    if (worker_.joinable()) worker_.join();
    if (joint_subscription_ != 0) {
        middleware_.unsubscribe("/joint_states", joint_subscription_);
        joint_subscription_ = 0;
    }
    for (const char* service : {"/fsm/update_logic", "/fsm/get_logic", "/fsm/set_running",
                                "/fsm/fire_transition", "/fsm/save_to_file", "/fsm/load_from_file",
                                "/fsm/list_saved"}) middleware_.unadvertise_service(service);
    std::lock_guard<std::mutex> lock(shared_->mutex);
    shared_->running = false;
}

void FiniteStateMachine::publish_status() {
    JsonValue status = JsonValue::make_object();
    {
        std::lock_guard<std::mutex> lock(shared_->mutex);
        status["current_state"] = shared_->current_state.empty() ? JsonValue() : JsonValue(shared_->current_state);
        status["time_in_state"] = shared_->current_state.empty() ? 0.0 :
            std::chrono::duration<double>(std::chrono::steady_clock::now() - shared_->entered_at).count();
        status["last_transition"] = shared_->last_transition;
        status["running"] = shared_->running;
    }
    middleware_.publish("/fsm/status", status);
}

void FiniteStateMachine::worker_loop() {
    auto next = std::chrono::steady_clock::now();
    while (started_.load()) {
        try { publish_status(); } catch (const std::exception&) {}
        next += std::chrono::milliseconds(100);
        if (next < std::chrono::steady_clock::now()) next = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next);
    }
}
} // namespace robot
