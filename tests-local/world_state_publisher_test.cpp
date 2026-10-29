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
    JsonValue wait(Predicate predicate, std::size_t from = 0) {
        std::unique_lock<std::mutex> lock(mutex);
        auto ready = [&] {
            for (std::size_t i = from; i < messages.size(); ++i) {
                if (predicate(messages[i])) return true;
            }
            return false;
        };
        assert(changed.wait_for(lock, std::chrono::seconds(2), ready));
        for (std::size_t i = from; i < messages.size(); ++i) {
            if (predicate(messages[i])) return messages[i];
        }
        assert(false);
        return {};
    }
};

const JsonValue& link(const JsonValue& message, const std::string& name) {
    for (const auto& entry : message["transforms"].as_array()) {
        if (entry["child_frame_id"].as_string() == name) return entry;
    }
    assert(false && "missing link");
    return message;
}

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-8);
}

void check(const JsonValue& entry, double x, double y, double z, int sec) {
    assert(entry["header"]["frame_id"].as_string() == "global_frame");
    assert(entry["header"]["stamp"]["sec"].as_int() == sec);
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

    for (int a = 0; a < 3; ++a) {
        for (int b = 0; b < 3; ++b) {
            double dot = 0;
            for (int row = 0; row < 3; ++row) {
                dot += matrix[4 * a + row].as_number() *
                       matrix[4 * b + row].as_number();
            }
            near(dot, a == b ? 1 : 0);
        }
    }
    double m00 = matrix[0].as_number(), m01 = matrix[4].as_number();
    double m02 = matrix[8].as_number(), m10 = matrix[1].as_number();
    double m11 = matrix[5].as_number(), m12 = matrix[9].as_number();
    double m20 = matrix[2].as_number(), m21 = matrix[6].as_number();
    double m22 = matrix[10].as_number();
    near(m00 * (m11 * m22 - m12 * m21) -
         m01 * (m10 * m22 - m12 * m20) +
         m02 * (m10 * m21 - m11 * m20), 1);
}

} // namespace

int main() {
    middleware::Middleware hub;
    robot::WorldStatePublisher publisher(hub);
    Stream stream;
    auto subscription = hub.subscribe("/xform_world",
        [&](const JsonValue& value) { stream.add(value); });
    publisher.start();

    // A pose sent before /tf is retained, but does not itself start the stream.
    hub.publish("/global_pose", JsonValue::parse(
        R"({"position":{"x":10,"y":20,"z":30},)"
        R"("orientation":{"x":0,"y":0,"z":0.7071067811865476,"w":0.7071067811865476}})"));
    std::this_thread::sleep_for(std::chrono::milliseconds(220));
    assert(stream.size() == 0);

    // Edges are deliberately out of root-first order. The tree has a chain and a branch.
    hub.publish("/tf", JsonValue::parse(R"({"transforms":[
      {"header":{"stamp":{"sec":7,"nanosec":99},"frame_id":"arm"},
       "child_frame_id":"tip","transform":{"translation":{"x":2,"y":0,"z":0},
       "rotation":{"x":0,"y":0,"z":0,"w":1}}},
      {"header":{"stamp":{"sec":7,"nanosec":99},"frame_id":"base"},
       "child_frame_id":"arm","transform":{"translation":{"x":1,"y":0,"z":0},
       "rotation":{"x":0,"y":0,"z":0.7071067811865476,"w":0.7071067811865476}}},
      {"header":{"stamp":{"sec":7,"nanosec":99},"frame_id":"base"},
       "child_frame_id":"branch","transform":{"translation":{"x":0,"y":3,"z":0},
       "rotation":{"x":0,"y":0,"z":0,"w":1}}}]})"));
    auto composed = stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 4;
    });
    check(link(composed, "base"), 10, 20, 30, 7);
    check(link(composed, "arm"), 10, 21, 30, 7);
    check(link(composed, "tip"), 8, 21, 30, 7);
    check(link(composed, "branch"), 7, 20, 30, 7);
    near(link(composed, "tip")["matrix"].as_array()[0].as_number(), -1);
    near(link(composed, "tip")["matrix"].as_array()[5].as_number(), -1);
    assert(link(composed, "tip")["header"]["stamp"]["nanosec"].as_int() == 99);

    std::size_t before_pose = stream.size();
    hub.publish("/global_pose", JsonValue::parse(
        R"({"position":{"x":-1,"y":2,"z":3},)"
        R"("orientation":{"x":0,"y":0,"z":0,"w":1}})"));
    auto replaced_pose = stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 4 &&
               link(value, "base")["matrix"].as_array()[12].as_number() == -1;
    }, before_pose);
    check(link(replaced_pose, "base"), -1, 2, 3, 7);
    check(link(replaced_pose, "arm"), 0, 2, 3, 7);
    check(link(replaced_pose, "tip"), 0, 4, 3, 7);
    check(link(replaced_pose, "branch"), -1, 5, 3, 7);

    // A malformed pose leaves the last valid one in effect.
    std::size_t before_bad = stream.size();
    hub.publish("/global_pose", JsonValue::parse(
        R"({"position":{"x":99,"y":99,"z":99},"orientation":{"x":0}})"));
    auto after_bad = stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 4;
    }, before_bad);
    check(link(after_bad, "base"), -1, 2, 3, 7);

    std::size_t before_reload = stream.size();
    hub.publish("/tf", JsonValue::parse(R"({"transforms":[
      {"header":{"stamp":{"sec":8,"nanosec":1},"frame_id":"next_root"},
       "child_frame_id":"next_child","transform":{"translation":{"x":5,"y":0,"z":0},
       "rotation":{"x":0,"y":0,"z":0,"w":1}}}]})"));
    auto replaced_tree = stream.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 2;
    }, before_reload);
    check(link(replaced_tree, "next_root"), -1, 2, 3, 8);
    check(link(replaced_tree, "next_child"), 4, 2, 3, 8);

    Stream late;
    auto joined = Clock::now();
    auto late_sub = hub.subscribe("/xform_world",
        [&](const JsonValue& value) { late.add(value); });
    auto latest = late.wait([](const JsonValue& value) {
        return value["transforms"].as_array().size() == 2;
    });
    assert(Clock::now() - joined < std::chrono::milliseconds(300));
    check(link(latest, "next_child"), 4, 2, 3, 8);

    publisher.stop();
    hub.unsubscribe("/xform_world", subscription);
    hub.unsubscribe("/xform_world", late_sub);
}
