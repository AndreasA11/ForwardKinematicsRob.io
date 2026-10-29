#include "parameter_server.hpp"
#include "robot_description_loader.hpp"
#include "robot_world_state_publisher.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

using middleware::JsonValue;
using Clock = std::chrono::steady_clock;

struct Stream {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<JsonValue> messages;

    void add(const JsonValue& value) {
        std::lock_guard<std::mutex> lock(mutex);
        messages.push_back(value);
        changed.notify_all();
    }

    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return messages.size();
    }

    template <typename Predicate>
    JsonValue wait(Predicate predicate, std::size_t from = 0,
                   std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
        std::unique_lock<std::mutex> lock(mutex);
        auto found = [&] {
            for (std::size_t i = from; i < messages.size(); ++i) {
                if (predicate(messages[i])) return true;
            }
            return false;
        };
        assert(changed.wait_for(lock, timeout, found));
        for (std::size_t i = from; i < messages.size(); ++i) {
            if (predicate(messages[i])) return messages[i];
        }
        assert(false);
        return {};
    }

    std::vector<JsonValue> after(std::size_t from) {
        std::lock_guard<std::mutex> lock(mutex);
        return {messages.begin() + from, messages.end()};
    }
};

JsonValue named(const std::string& name) {
    JsonValue value = JsonValue::make_object();
    value["name"] = name;
    return value;
}

std::size_t load(middleware::Middleware& hub, const std::string& xml) {
    JsonValue request = named("robot_description");
    request["value"] = xml;
    auto reply = hub.call_service("/param_server/set_param", request);
    assert(reply.first);
    return static_cast<std::size_t>(reply.second["version"].as_number());
}

JsonValue status_for(middleware::Middleware& hub, std::size_t version) {
    auto deadline = Clock::now() + std::chrono::seconds(2);
    while (Clock::now() < deadline) {
        auto reply = hub.call_service("/param_server/get_param",
            named("/robot_state_publisher/description_status"));
        if (reply.first && reply.second["value"]["version"].as_number() >= version) {
            return reply.second["value"];
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    assert(false && "description status deadline missed");
    return {};
}

bool has_link(const JsonValue& message, const std::string& link) {
    for (const JsonValue& entry : message["transforms"].as_array()) {
        if (entry["child_frame_id"].as_string() == link) return true;
    }
    return false;
}

const JsonValue& link_entry(const JsonValue& message, const std::string& link) {
    for (const JsonValue& entry : message["transforms"].as_array()) {
        if (entry["child_frame_id"].as_string() == link) return entry;
    }
    assert(false);
    return message;
}

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-9);
}

void check_matrix(const JsonValue& entry, double x, double y, double z) {
    const auto& matrix = entry["matrix"].as_array();
    assert(matrix.size() == 16);
    for (const auto& value : matrix) assert(value.is_number());
    near(matrix[12].as_number(), x);
    near(matrix[13].as_number(), y);
    near(matrix[14].as_number(), z);
    near(matrix[3].as_number(), 0);
    near(matrix[7].as_number(), 0);
    near(matrix[11].as_number(), 0);
    near(matrix[15].as_number(), 1);
    assert(entry["header"]["frame_id"].as_string() == "global_frame");
}

} // namespace

int main() {
    middleware::Middleware hub;
    ParameterServer params(hub);
    robot::WorldStatePublisher world(hub);
    robot::DescriptionLoader robot_state(hub);
    Stream tf, world_stream;
    auto tf_sub = hub.subscribe("/tf", [&](const JsonValue& value) { tf.add(value); });
    auto world_sub = hub.subscribe("/xform_world",
        [&](const JsonValue& value) { world_stream.add(value); });
    world.start();
    robot_state.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    assert(tf.size() == 0 && world_stream.size() == 0);

    const std::string first =
        "<robot><link name='base'/><link name='arm'/><link name='tip'/>"
        "<link name='branch'/>"
        "<joint name='shoulder' type='revolute'><parent link='base'/><child link='arm'/>"
        "<origin xyz='1 0 0'/><axis xyz='0 0 1'/><limit/></joint>"
        "<joint name='tool' type='fixed'><parent link='arm'/><child link='tip'/>"
        "<origin xyz='2 0 0'/></joint>"
        "<joint name='side' type='fixed'><parent link='base'/><child link='branch'/>"
        "<origin xyz='0 3 0'/></joint></robot>";
    std::size_t first_version = load(hub, first);
    assert(status_for(hub, first_version)["accepted"].as_bool());
    auto zero_tf = tf.wait([](const JsonValue& message) {
        return message["transforms"].as_array().size() == 3;
    });
    auto zero_world = world_stream.wait([](const JsonValue& message) {
        return message["transforms"].as_array().size() == 4;
    });
    assert(zero_tf["transforms"].as_array().size() == 3);
    assert(zero_world["transforms"].as_array().size() == 4);
    check_matrix(link_entry(zero_world, "base"), 0, 0, 0);
    check_matrix(link_entry(zero_world, "tip"), 3, 0, 0);
    check_matrix(link_entry(zero_world, "branch"), 0, 3, 0);

    std::size_t tf_before = tf.size(), world_before = world_stream.size();
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    assert(tf.size() >= tf_before + 3);
    assert(world_stream.size() >= world_before + 3);

    Stream late_tf, late_world;
    auto joined_at = Clock::now();
    auto late_tf_sub = hub.subscribe("/tf", [&](const JsonValue& value) { late_tf.add(value); });
    auto late_world_sub = hub.subscribe("/xform_world",
        [&](const JsonValue& value) { late_world.add(value); });
    late_tf.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 3;
    });
    late_world.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 4;
    });
    assert(Clock::now() - joined_at < std::chrono::milliseconds(300));

    hub.publish("/joint_states", JsonValue::parse(
        "{\"header\":{\"stamp\":{\"sec\":9,\"nanosec\":7}},"
        "\"name\":[\"shoulder\"],\"position\":[1.5707963267948966]}"));
    auto moved = world_stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 4 &&
            link_entry(value, "tip")["header"]["stamp"]["sec"].as_int() == 9;
    });
    check_matrix(link_entry(moved, "tip"), 1, 2, 0);
    check_matrix(link_entry(moved, "branch"), 0, 3, 0);
    near(link_entry(moved, "tip")["matrix"].as_array()[0].as_number(), 0);
    near(link_entry(moved, "tip")["matrix"].as_array()[1].as_number(), 1);

    std::size_t rejected = load(hub, "<robot><link/></robot>");
    assert(!status_for(hub, rejected)["accepted"].as_bool());
    std::size_t after_rejection = world_stream.size();
    auto still_old = world_stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 4;
    }, after_rejection);
    check_matrix(link_entry(still_old, "tip"), 1, 2, 0);

    std::size_t new_version = load(hub,
        "<robot><link name='new_root'/><link name='new_child'/>"
        "<joint name='new_joint' type='fixed'><parent link='new_root'/>"
        "<child link='new_child'/><origin xyz='5 0 0'/></joint></robot>");
    assert(status_for(hub, new_version)["accepted"].as_bool());
    std::size_t after_status = world_stream.size();
    auto new_world = world_stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 2;
    }, after_status);
    assert(has_link(new_world, "new_root") && has_link(new_world, "new_child"));
    assert(!has_link(new_world, "base") && !has_link(new_world, "tip"));
    check_matrix(link_entry(new_world, "new_child"), 5, 0, 0);
    for (const auto& publication : world_stream.after(after_status)) {
        assert(publication["transforms"].as_array().size() == 2);
    }

    robot_state.stop();
    world.stop();
    hub.unsubscribe("/tf", tf_sub);
    hub.unsubscribe("/xform_world", world_sub);
    hub.unsubscribe("/tf", late_tf_sub);
    hub.unsubscribe("/xform_world", late_world_sub);
}
