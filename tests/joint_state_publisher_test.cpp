#include "joint_state_publisher.hpp"
#include "parameter_server.hpp"

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

double position(const JsonValue& message, const std::string& name) {
    const auto& names = message["name"].as_array();
    const auto& positions = message["position"].as_array();
    assert(names.size() == positions.size());
    for (std::size_t i = 0; i < names.size(); ++i)
        if (names[i].as_string() == name) return positions[i].as_number();
    assert(false && "joint not published");
    return 0;
}

bool has(const JsonValue& message, const std::string& name) {
    for (const auto& item : message["name"].as_array())
        if (item.as_string() == name) return true;
    return false;
}

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-8);
}

void load(middleware::Middleware& hub, const std::string& xml) {
    JsonValue args = JsonValue::make_object();
    args["name"] = "robot_description";
    args["value"] = xml;
    assert(hub.call_service("/param_server/set_param", args).first);
}

} // namespace

int main() {
    middleware::Middleware hub;
    ParameterServer parameters(hub);
    robot::JointStatePublisher servo(hub);
    Inbox inbox;
    auto subscription = hub.subscribe("/joint_states", [&](const JsonValue& message) {
        inbox.add(message);
    });
    servo.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(220));
    assert(inbox.size() == 0);
    assert(hub.has_service("/joint_state_publisher/reset"));

    // An unknown target and the first point both arrive before a robot is loaded.
    hub.publish("/joint_trajectory", JsonValue::parse(
        R"({"joint_names":["arm","later","slide"],"points":[{"positions":[-0.8,0.3,0.1]},{"positions":[0.8,0.7,0.4]}]})"));
    load(hub,
        "<robot><link name='base'/><link name='a'/><link name='s'/><link name='f'/>"
        "<joint name='arm' type='revolute'><parent link='base'/><child link='a'/>"
        "<limit lower='-1' upper='1'/></joint>"
        "<joint name='slide' type='prismatic'><parent link='base'/><child link='s'/>"
        "<limit lower='0' upper='0.5'/></joint>"
        "<joint name='fixed' type='fixed'><parent link='base'/><child link='f'/></joint>"
        "</robot>");
    auto initial = inbox.wait_for(0, [](const JsonValue& msg) {
        return has(msg, "arm");
    });
    assert(initial["name"].as_array().size() == 2);
    assert(!has(initial, "fixed"));
    near(position(initial, "arm"), 0.0);
    near(position(initial, "slide"), 0.0);
    assert(initial["header"]["stamp"]["sec"].is_number());
    auto moved = inbox.wait_for(1, [](const JsonValue& msg) {
        return has(msg, "arm") && std::abs(position(msg, "arm") - 0.8) < 1e-8;
    });
    near(position(moved, "slide"), 0.4);

    std::size_t before_reset = inbox.size();
    auto reset = hub.call_service("/joint_state_publisher/reset", JsonValue::make_object());
    assert(reset.first);
    auto centered = inbox.wait_for(before_reset, [](const JsonValue& msg) {
        return has(msg, "arm") && std::abs(position(msg, "arm")) < 1e-8 &&
            std::abs(position(msg, "slide") - 0.25) < 1e-8;
    });
    near(position(centered, "slide"), 0.25);
    std::size_t after_reset = inbox.size();
    auto stable = inbox.wait_for(after_reset, [](const JsonValue& msg) {
        return has(msg, "slide");
    });
    near(position(stable, "slide"), 0.25);

    // The new robot preserves arm at its new bound, and activates retained later.
    std::size_t before_reload = inbox.size();
    load(hub,
        "<robot><link name='base'/><link name='a'/><link name='l'/>"
        "<joint name='arm' type='revolute'><parent link='base'/><child link='a'/>"
        "<limit lower='0.2' upper='0.6'/></joint>"
        "<joint name='later' type='continuous'><parent link='a'/><child link='l'/>"
        "</joint></robot>");
    auto preserved = inbox.wait_for(before_reload, [](const JsonValue& msg) {
        return has(msg, "later");
    });
    assert(preserved["name"].as_array().size() == 2);
    near(position(preserved, "arm"), 0.2);
    near(position(preserved, "later"), 0.0);
    auto activated = inbox.wait_for(before_reload + 1, [](const JsonValue& msg) {
        return has(msg, "later") && std::abs(position(msg, "later") - 0.7) < 1e-8;
    });
    near(position(activated, "arm"), 0.2);

    // A trajectory changes only the named target and respects new bounds.
    hub.publish("/joint_trajectory", JsonValue::parse(
        R"({"joint_names":["arm"],"points":[{"positions":[10]}]})"));
    auto clamped = inbox.wait_for(inbox.size(), [](const JsonValue& msg) {
        return has(msg, "arm") && std::abs(position(msg, "arm") - 0.6) < 1e-8;
    });
    near(position(clamped, "later"), 0.7);

    servo.stop();
    hub.unsubscribe("/joint_states", subscription);
    assert(!hub.has_service("/joint_state_publisher/reset"));
}
