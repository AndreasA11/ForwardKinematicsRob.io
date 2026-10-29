#include "finite_state_machine.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
using middleware::JsonValue;

struct Inbox {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<JsonValue> messages;

    void add(const JsonValue& message) {
        std::lock_guard<std::mutex> lock(mutex);
        messages.push_back(message);
        changed.notify_all();
    }

    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return messages.size();
    }

    template<class Predicate>
    JsonValue wait_for(std::size_t start, Predicate predicate) {
        std::unique_lock<std::mutex> lock(mutex);
        auto found = [&] {
            for (std::size_t i = start; i < messages.size(); ++i)
                if (predicate(messages[i])) return true;
            return false;
        };
        assert(changed.wait_for(lock, std::chrono::seconds(3), found));
        for (std::size_t i = start; i < messages.size(); ++i)
            if (predicate(messages[i])) return messages[i];
        assert(false);
        return {};
    }
};

JsonValue make_sample_doc() {
    return JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[\"shoulder\",\"elbow\"],\"positions\":[0.0,0.0]},"
        "\"epsilon\":0.02,\"on_arrival\":\"to_wave\",\"transitions\":[{\"name\":\"to_wave\",\"next_state\":\"wave\"}]},"
        "{\"name\":\"wave\",\"target\":{\"joint_names\":[\"shoulder\",\"elbow\"],\"positions\":[0.8,-0.5]},"
        "\"epsilon\":0.02,\"on_arrival\":\"to_home\",\"transitions\":[{\"name\":\"to_home\",\"next_state\":\"home\"}]}"
        "]}"
    );
}

JsonValue make_joint_states_msg(const std::vector<std::string>& names, const std::vector<double>& positions) {
    JsonValue msg = JsonValue::make_object();
    JsonValue name_arr = JsonValue::make_array();
    JsonValue pos_arr = JsonValue::make_array();
    for (std::size_t i = 0; i < names.size(); ++i) {
        name_arr.push_back(names[i]);
        pos_arr.push_back(positions[i]);
    }
    msg["name"] = std::move(name_arr);
    msg["position"] = std::move(pos_arr);
    return msg;
}

void test_validation_and_update() {
    middleware::Middleware hub;
    robot::FiniteStateMachine fsm(hub);
    fsm.start();

    // 1. Initial get_logic before any logic loaded fails cleanly
    auto get_init = hub.call_service("/fsm/get_logic", JsonValue::make_object());
    assert(!get_init.first);

    // 2. Invalid tables are rejected
    // Missing initial_state
    auto bad1 = hub.call_service("/fsm/update_logic", JsonValue::parse("{\"states\":[]}"));
    assert(!bad1.first);

    // Empty states
    auto bad2 = hub.call_service("/fsm/update_logic", JsonValue::parse("{\"initial_state\":\"home\",\"states\":[]}"));
    assert(!bad2.first);

    // Initial state not found
    auto bad3 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"missing\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":0.01,\"transitions\":[]}"
        "]}"
    ));
    assert(!bad3.first);

    // Duplicate state names
    auto bad4 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":0.01,\"transitions\":[]},"
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":0.01,\"transitions\":[]}"
        "]}"
    ));
    assert(!bad4.first);

    // Mismatched target joint_names and positions
    auto bad5 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[\"a\"],\"positions\":[]},\"epsilon\":0.01,\"transitions\":[]}"
        "]}"
    ));
    assert(!bad5.first);

    // Negative epsilon
    auto bad6 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":-0.01,\"transitions\":[]}"
        "]}"
    ));
    assert(!bad6.first);

    // Duplicate transition names within a state
    auto bad7 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":0.01,\"transitions\":["
        "{\"name\":\"step\",\"next_state\":\"home\"},{\"name\":\"step\",\"next_state\":\"home\"}"
        "]}"
        "]}"
    ));
    assert(!bad7.first);

    // Transition next_state targets non-existent state
    auto bad8 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":0.01,\"transitions\":["
        "{\"name\":\"step\",\"next_state\":\"nowhere\"}"
        "]}"
        "]}"
    ));
    assert(!bad8.first);

    // on_arrival names non-existent transition
    auto bad9 = hub.call_service("/fsm/update_logic", JsonValue::parse(
        "{\"initial_state\":\"home\",\"states\":["
        "{\"name\":\"home\",\"target\":{\"joint_names\":[],\"positions\":[]},\"epsilon\":0.01,\"on_arrival\":\"unknown\",\"transitions\":[]}"
        "]}"
    ));
    assert(!bad9.first);

    // 3. Load valid document
    auto valid_reply = hub.call_service("/fsm/update_logic", make_sample_doc());
    assert(valid_reply.first);
    assert(valid_reply.second["current_state"].as_string() == "home");

    // 4. get_logic returns the valid document
    auto get_ok = hub.call_service("/fsm/get_logic", JsonValue::make_object());
    assert(get_ok.first);
    assert(get_ok.second["initial_state"].as_string() == "home");
    assert(get_ok.second["states"].as_array().size() == 2);

    // 5. Invalid replacement leaves active table running
    auto bad_replace = hub.call_service("/fsm/update_logic", JsonValue::parse("{\"states\":[123]}"));
    assert(!bad_replace.first);

    auto get_still = hub.call_service("/fsm/get_logic", JsonValue::make_object());
    assert(get_still.first);
    assert(get_still.second["initial_state"].as_string() == "home");

    fsm.stop();
}

void test_runtime_transitions_and_trajectory() {
    middleware::Middleware hub;
    Inbox traj_box;
    Inbox status_box;
    hub.subscribe("/joint_trajectory", [&](const JsonValue& msg) { traj_box.add(msg); });
    hub.subscribe("/fsm/status", [&](const JsonValue& msg) { status_box.add(msg); });

    robot::FiniteStateMachine fsm(hub);
    fsm.start();

    // Update logic -> should immediately publish target of initial_state "home"
    std::size_t traj_start = traj_box.size();
    auto update = hub.call_service("/fsm/update_logic", make_sample_doc());
    assert(update.first);
    assert(update.second["current_state"].as_string() == "home");

    JsonValue t1 = traj_box.wait_for(traj_start, [](const JsonValue& msg) {
        return msg.contains("joint_names") && msg["joint_names"].as_array().size() == 2;
    });
    assert(t1["joint_names"].as_array()[0].as_string() == "shoulder");
    assert(t1["joint_names"].as_array()[1].as_string() == "elbow");
    assert(t1["points"].as_array()[0]["positions"].as_array()[0].as_number() == 0.0);
    assert(t1["points"].as_array()[0]["positions"].as_array()[1].as_number() == 0.0);

    // Verify /fsm/status is published
    JsonValue s1 = status_box.wait_for(0, [](const JsonValue& msg) {
        return msg.contains("current_state") && msg["current_state"].is_string() &&
               msg["current_state"].as_string() == "home" && msg["running"].as_bool();
    });
    assert(s1["time_in_state"].as_number() >= 0.0);

    // Feed /joint_states far from target -> arrival condition not met, should not transition
    hub.publish("/joint_states", make_joint_states_msg({"shoulder", "elbow"}, {0.5, 0.5}));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(traj_box.size() == traj_start + 1); // Still only 1 trajectory published

    // Feed /joint_states with one joint in tolerance, one outside -> no transition
    hub.publish("/joint_states", make_joint_states_msg({"shoulder", "elbow"}, {0.01, 0.05}));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(traj_box.size() == traj_start + 1);

    // Feed /joint_states within epsilon (0.02) -> should transition via "to_wave" to "wave"
    std::size_t traj_wave_start = traj_box.size();
    hub.publish("/joint_states", make_joint_states_msg({"shoulder", "elbow"}, {0.01, -0.01}));

    JsonValue t2 = traj_box.wait_for(traj_wave_start, [](const JsonValue& msg) {
        return msg.contains("points") && msg["points"].as_array().size() > 0 &&
               std::abs(msg["points"].as_array()[0]["positions"].as_array()[0].as_number() - 0.8) < 1e-4;
    });
    assert(t2["points"].as_array()[0]["positions"].as_array()[1].as_number() == -0.5);

    // Check status after transition
    JsonValue s2 = status_box.wait_for(0, [](const JsonValue& msg) {
        return msg.contains("current_state") && msg["current_state"].is_string() &&
               msg["current_state"].as_string() == "wave" &&
               msg.contains("last_transition") && msg["last_transition"].is_string() &&
               msg["last_transition"].as_string() == "to_wave";
    });
    assert(s2["running"].as_bool());

    // Manual transition via /fsm/fire_transition
    std::size_t traj_home_start = traj_box.size();
    auto fire = hub.call_service("/fsm/fire_transition", JsonValue::parse("{\"transition\":\"to_home\"}"));
    assert(fire.first);
    assert(fire.second["current_state"].as_string() == "home");

    JsonValue t3 = traj_box.wait_for(traj_home_start, [](const JsonValue& msg) {
        return msg.contains("points") && msg["points"].as_array().size() > 0 &&
               std::abs(msg["points"].as_array()[0]["positions"].as_array()[0].as_number() - 0.0) < 1e-4;
    });
    assert(t3["points"].as_array()[0]["positions"].as_array()[1].as_number() == 0.0);

    // Set running to false
    auto stop_call = hub.call_service("/fsm/set_running", JsonValue::parse("{\"data\":false}"));
    assert(stop_call.first);
    assert(!stop_call.second["running"].as_bool());

    // While stopped, manual transition should fail
    auto fire_stopped = hub.call_service("/fsm/fire_transition", JsonValue::parse("{\"transition\":\"to_wave\"}"));
    assert(!fire_stopped.first);

    // While stopped, arrival condition should not trigger transition
    hub.publish("/joint_states", make_joint_states_msg({"shoulder", "elbow"}, {0.0, 0.0}));
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    assert(traj_box.size() == traj_home_start + 1);

    // Resume running
    auto resume_call = hub.call_service("/fsm/set_running", JsonValue::parse("{\"data\":true}"));
    assert(resume_call.first);
    assert(resume_call.second["running"].as_bool());

    // Query running without changing
    auto query_running = hub.call_service("/fsm/set_running", JsonValue::make_object());
    assert(query_running.first);
    assert(query_running.second["running"].as_bool());

    fsm.stop();
}

void test_file_operations_and_security() {
    middleware::Middleware hub;
    robot::FiniteStateMachine fsm(hub);
    fsm.start();

    // Directory traversal rejected
    auto bad_id1 = hub.call_service("/fsm/save_to_file", JsonValue::parse(
        "{\"identifier\":\"../escape\",\"initial_state\":\"home\",\"states\":[]}"
    ));
    assert(!bad_id1.first);

    auto bad_id2 = hub.call_service("/fsm/save_to_file", JsonValue::parse(
        "{\"identifier\":\"subdir/nested\",\"initial_state\":\"home\",\"states\":[]}"
    ));
    assert(!bad_id2.first);

    auto bad_id3 = hub.call_service("/fsm/save_to_file", JsonValue::parse(
        "{\"identifier\":\"invalid*chars\",\"initial_state\":\"home\",\"states\":[]}"
    ));
    assert(!bad_id3.first);

    // Save valid identifier
    const std::string id1 = "choreography_test_b";
    const std::string id2 = "choreography_test_a";

    auto save1 = hub.call_service("/fsm/save_to_file", JsonValue::parse(
        "{\"identifier\":\"" + id1 + "\",\"initial_state\":\"home\",\"states\":[{\"name\":\"home\"}]}"
    ));
    assert(save1.first);
    assert(save1.second["path"].as_string() == "fsm/" + id1 + ".json");

    auto save2 = hub.call_service("/fsm/save_to_file", JsonValue::parse(
        "{\"identifier\":\"" + id2 + "\",\"initial_state\":\"wave\",\"states\":[{\"name\":\"wave\"}]}"
    ));
    assert(save2.first);

    // Load from file
    auto load1 = hub.call_service("/fsm/load_from_file", JsonValue::parse("{\"identifier\":\"" + id1 + "\"}"));
    assert(load1.first);
    assert(load1.second["initial_state"].as_string() == "home");

    // Load non-existent file fails
    auto load_missing = hub.call_service("/fsm/load_from_file", JsonValue::parse("{\"identifier\":\"non_existent_fsm\"}"));
    assert(!load_missing.first);

    // List saved files - must contain both, sorted
    auto list_reply = hub.call_service("/fsm/list_saved", JsonValue::make_object());
    assert(list_reply.first);
    const auto& ids = list_reply.second["identifiers"].as_array();
    bool found1 = false, found2 = false;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (ids[i].as_string() == id1) found1 = true;
        if (ids[i].as_string() == id2) found2 = true;
        if (i > 0) {
            assert(ids[i - 1].as_string() <= ids[i].as_string()); // Sorted!
        }
    }
    assert(found1 && found2);

    // Clean up created files
    std::filesystem::remove("fsm/" + id1 + ".json");
    std::filesystem::remove("fsm/" + id2 + ".json");

    fsm.stop();
}

} // namespace

int main() {
    test_validation_and_update();
    test_runtime_transitions_and_trajectory();
    test_file_operations_and_security();
    return 0;
}
