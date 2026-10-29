#include "robot_description_loader.hpp"
#include "parameter_server.hpp"

#include <cassert>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <mutex>
#include <numbers>
#include <string>
#include <thread>
#include <vector>

namespace {

using middleware::JsonValue;
constexpr double pi = std::numbers::pi_v<double>;

struct Messages {
    std::mutex mutex;
    std::condition_variable changed;
    std::vector<JsonValue> values;

    void add(const JsonValue& message) {
        std::lock_guard<std::mutex> lock(mutex);
        values.push_back(message);
        changed.notify_all();
    }

    template <typename Predicate>
    JsonValue wait_for(Predicate matches, std::size_t start = 0) {
        std::unique_lock<std::mutex> lock(mutex);
        auto has_match = [&] {
            for (std::size_t i = start; i < values.size(); ++i) {
                if (matches(values[i])) return true;
            }
            return false;
        };
        assert(changed.wait_for(lock, std::chrono::seconds(2), has_match));
        for (std::size_t i = start; i < values.size(); ++i) {
            if (matches(values[i])) return values[i];
        }
        assert(false);
        return {};
    }

    std::size_t size() {
        std::lock_guard<std::mutex> lock(mutex);
        return values.size();
    }
};

JsonValue named(const std::string& name) {
    JsonValue args = JsonValue::make_object();
    args["name"] = name;
    return args;
}

std::size_t set_description(middleware::Middleware& hub, const JsonValue& value) {
    JsonValue args = named("robot_description");
    args["value"] = value;
    auto reply = hub.call_service("/param_server/set_param", args);
    assert(reply.first);
    return static_cast<std::size_t>(reply.second["version"].as_number());
}

JsonValue wait_status(middleware::Middleware& hub, std::size_t version) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        auto reply = hub.call_service("/param_server/get_param",
                                      named("/robot_state_publisher/description_status"));
        if (reply.first && reply.second["value"]["version"].as_number() >= version) {
            return reply.second["value"];
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    assert(false && "description status did not arrive");
    return {};
}

const JsonValue& entry(const JsonValue& tf, const std::string& child) {
    for (const JsonValue& transform : tf["transforms"].as_array()) {
        if (transform["child_frame_id"].as_string() == child) return transform;
    }
    assert(false && "joint transform absent");
    return tf;
}

double translation(const JsonValue& transform, const std::string& coordinate) {
    return transform["transform"]["translation"][coordinate].as_number();
}

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-9);
}

JsonValue joint_state(const std::string& stamp, const std::string& names,
                      const std::string& positions) {
    return JsonValue::parse("{\"header\":{\"stamp\":" + stamp +
        ",\"frame_id\":\"\"},\"name\":" + names +
        ",\"position\":" + positions + "}");
}

} // namespace

int main() {
    middleware::Middleware hub;
    ParameterServer parameters(hub);
    robot::DescriptionLoader publisher(hub);
    Messages received;
    auto sub = hub.subscribe("/tf", [&](const JsonValue& value) { received.add(value); });
    publisher.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(260));
    assert(received.size() == 0);

    const std::string xml =
        "<robot><link name='base'/><link name='arm'/><link name='tool'/>"
        "<link name='slider'/><link name='branch'/>"
        "<joint name='shoulder' type='revolute'><parent link='base'/><child link='arm'/>"
        "<origin xyz='1 0 0'/><axis xyz='0 0 2'/><limit lower='-.1' upper='.1'/></joint>"
        "<joint name='wrist' type='continuous'><parent link='arm'/><child link='tool'/>"
        "<origin xyz='2 0 0'/><axis xyz='2 0 0'/></joint>"
        "<joint name='slide' type='prismatic'><parent link='base'/><child link='slider'/>"
        "<origin rpy='0 0 1.5707963267948966'/><axis xyz='2 0 0'/>"
        "<limit lower='-.1' upper='.1'/></joint>"
        "<joint name='fixed' type='fixed'><parent link='base'/><child link='branch'/>"
        "<origin xyz='0 3 0'/><axis xyz='0 0 0'/></joint></robot>";
    std::size_t version = set_description(hub, xml);
    assert(wait_status(hub, version)["accepted"].as_bool());
    auto zero = received.wait_for([](const JsonValue& tf) {
        return tf["transforms"].as_array().size() == 4;
    });
    assert(zero["transforms"].as_array().size() == 4);
    for (const auto& transform : zero["transforms"].as_array()) {
        assert(transform["header"]["stamp"].dump() ==
               JsonValue::parse("{\"sec\":0,\"nanosec\":0}").dump());
    }
    near(translation(entry(zero, "arm"), "x"), 1);
    near(translation(entry(zero, "branch"), "y"), 3);
    assert(entry(zero, "arm")["header"]["frame_id"].as_string() == "base");

    hub.publish("/joint_states", joint_state(
        "{\"sec\":12,\"nanosec\":345,\"extra\":\"keep\"}",
        "[\"shoulder\",\"wrist\",\"slide\",\"unknown\",\"fixed\"]",
        "[1.5707963267948966,4.0,0.7,9,99]"));
    auto moved = received.wait_for([](const JsonValue& tf) {
        return tf["transforms"].as_array().size() == 4 &&
               tf["transforms"].as_array()[0]["header"]["stamp"]["sec"].as_int() == 12;
    });
    for (const auto& transform : moved["transforms"].as_array()) {
        const JsonValue& stamp = transform["header"]["stamp"];
        assert(stamp["sec"].as_int() == 12 && stamp["nanosec"].as_int() == 345);
        assert(stamp["extra"].as_string() == "keep");
        const JsonValue& rotation = transform["transform"]["rotation"];
        double qnorm = std::sqrt(std::pow(rotation["x"].as_number(), 2) +
            std::pow(rotation["y"].as_number(), 2) +
            std::pow(rotation["z"].as_number(), 2) +
            std::pow(rotation["w"].as_number(), 2));
        near(qnorm, 1.0);
    }
    near(translation(entry(moved, "arm"), "x"), 1);
    near(entry(moved, "arm")["transform"]["rotation"]["z"].as_number(),
         std::sin(pi / 4));
    near(entry(moved, "tool")["transform"]["rotation"]["x"].as_number(),
         std::sin(2.0));
    near(translation(entry(moved, "slider"), "y"), 1.4);
    near(translation(entry(moved, "branch"), "y"), 3);

    hub.publish("/joint_states", joint_state(
        "{\"sec\":13,\"nanosec\":0}", "[\"wrist\"]", "[0.3]"));
    auto partial = received.wait_for([](const JsonValue& tf) {
        return tf["transforms"].as_array().size() == 4 &&
               tf["transforms"].as_array()[0]["header"]["stamp"]["sec"].as_int() == 13;
    });
    near(entry(partial, "arm")["transform"]["rotation"]["z"].as_number(), 0);
    near(translation(entry(partial, "slider"), "y"), 0);
    near(entry(partial, "tool")["transform"]["rotation"]["x"].as_number(),
         std::sin(0.15));

    std::size_t before = received.size();
    std::this_thread::sleep_for(std::chrono::milliseconds(450));
    assert(received.size() >= before + 3);
    Messages late;
    auto late_sub = hub.subscribe("/tf", [&](const JsonValue& value) { late.add(value); });
    auto late_message = late.wait_for([](const JsonValue& tf) {
        return tf["transforms"].as_array().size() == 4;
    });
    assert(late_message["transforms"].as_array().size() == 4);

    std::size_t rejected = set_description(hub, "<robot><link/></robot>");
    assert(!wait_status(hub, rejected)["accepted"].as_bool());
    auto after_reject = received.wait_for([](const JsonValue& tf) {
        return tf["transforms"].as_array().size() == 4 &&
               tf["transforms"].as_array()[0]["header"]["stamp"]["sec"].as_int() == 13;
    }, received.size());
    near(entry(after_reject, "tool")["transform"]["rotation"]["x"].as_number(),
         std::sin(0.15));

    std::size_t new_version = set_description(hub,
        "<robot><link name='new_root'/><link name='new_child'/>"
        "<joint name='new_joint' type='fixed'><parent link='new_root'/>"
        "<child link='new_child'/><origin xyz='5 0 0'/></joint></robot>");
    assert(wait_status(hub, new_version)["accepted"].as_bool());
    auto reloaded = received.wait_for([](const JsonValue& tf) {
        return tf["transforms"].as_array().size() == 1;
    }, received.size());
    assert(reloaded["transforms"].as_array()[0]["child_frame_id"].as_string() == "new_child");
    near(translation(entry(reloaded, "new_child"), "x"), 5);
    assert(entry(reloaded, "new_child")["header"]["stamp"]["sec"].as_int() == 0);

    publisher.stop();
    hub.unsubscribe("/tf", sub);
    hub.unsubscribe("/tf", late_sub);
}
