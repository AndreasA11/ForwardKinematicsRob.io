#include "robot_description_loader.hpp"

#include <chrono>
#include <cmath>
#include <exception>
#include <string>
#include <thread>
#include <utility>
#include <algorithm>

namespace robot {
namespace {

middleware::JsonValue named_argument(const std::string& name) {
    middleware::JsonValue args = middleware::JsonValue::make_object();
    args["name"] = name;
    return args;
}

middleware::JsonValue zero_stamp() {
    middleware::JsonValue stamp = middleware::JsonValue::make_object();
    stamp["sec"] = 0;
    stamp["nanosec"] = 0;
    return stamp;
}

middleware::JsonValue vector_json(const math3d::Vector3& vector) {
    middleware::JsonValue value = middleware::JsonValue::make_object();
    value["x"] = vector[0];
    value["y"] = vector[1];
    value["z"] = vector[2];
    return value;
}

middleware::JsonValue quaternion_json(const math3d::Quaternion& quaternion) {
    middleware::JsonValue value = middleware::JsonValue::make_object();
    value["x"] = quaternion.x;
    value["y"] = quaternion.y;
    value["z"] = quaternion.z;
    value["w"] = quaternion.w;
    return value;
}

} // namespace

DescriptionLoader::~DescriptionLoader() {
    stop();
}

void DescriptionLoader::start() {
    if (running_.exchange(true)) return;
    // *CHANGES*: Capture shared state so an in-flight middleware callback cannot outlive us.
    joint_subscription_ = middleware_.subscribe("/joint_states",
        [state = state_](const middleware::JsonValue& message) {
            receive_joint_state(*state, message);
        });
    worker_ = std::thread(&DescriptionLoader::worker_loop, this);
}

void DescriptionLoader::stop() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    if (joint_subscription_ != 0) {
        middleware_.unsubscribe("/joint_states", joint_subscription_);
        joint_subscription_ = 0;
    }
}

std::shared_ptr<const LoadedRobot> DescriptionLoader::current_robot() const {
    std::lock_guard<std::mutex> lock(state_->mutex);
    return state_->current;
}

void DescriptionLoader::receive_joint_state(
    SharedState& state, const middleware::JsonValue& message) {
    // *CHANGES*: Treat each usable message as a full replacement; malformed fields are ignored.
    if (!message.is_object() || !message.contains("header") ||
        !message["header"].is_object() ||
        !message["header"].contains("stamp") ||
        !message["header"]["stamp"].is_object() ||
        !message.contains("name") || !message["name"].is_array() ||
        !message.contains("position") || !message["position"].is_array()) return;

    const auto& names = message["name"].as_array();
    const auto& positions = message["position"].as_array();
    std::unordered_map<std::string, double> next;
    for (std::size_t i = 0; i < std::min(names.size(), positions.size()); ++i) {
        if (names[i].is_string() && positions[i].is_number() &&
            std::isfinite(positions[i].as_number())) {
            next[names[i].as_string()] = positions[i].as_number();
        }
    }
    std::lock_guard<std::mutex> lock(state.mutex);
    state.positions = std::move(next);
    state.stamp = message["header"]["stamp"];
}

void DescriptionLoader::worker_loop() {
    auto next_tick = std::chrono::steady_clock::now();
    while (running_.load()) {
        try {
            poll_once();
        } catch (const std::exception&) {
            // Keep polling if a service fails unexpectedly; a later call may succeed.
        }
        try {
            publish_tf();
        } catch (const std::exception&) {
            // Bad input must not end periodic publishing for future valid messages.
        }
        next_tick += std::chrono::milliseconds(100);
        auto now = std::chrono::steady_clock::now();
        if (next_tick < now) next_tick = now;
        std::this_thread::sleep_until(next_tick);
    }
}

void DescriptionLoader::poll_once() {
    auto reply = middleware_.call_service(
        "/param_server/get_param", named_argument("robot_description"));
    if (!reply.first || !reply.second.is_object() ||
        !reply.second.contains("version") || !reply.second["version"].is_number() ||
        !reply.second.contains("value")) {
        return;
    }
    double raw_version = reply.second["version"].as_number();
    if (!std::isfinite(raw_version) || raw_version < 1.0 ||
        std::floor(raw_version) != raw_version) return;
    std::size_t version = static_cast<std::size_t>(raw_version);
    if (version <= last_processed_version_) return;

    bool accepted = false;
    std::string error;
    try {
        const middleware::JsonValue& value = reply.second["value"];
        if (!value.is_string()) {
            error = "robot_description must be a JSON string";
        } else {
            ParseResult parsed = parse_urdf(value.as_string());
            if (!parsed) {
                error = parsed.error;
            } else {
                auto candidate = std::make_shared<LoadedRobot>();
                candidate->model = std::move(*parsed.model);
                candidate->zero_fk = compute_fk(candidate->model);
                candidate->loaded_version = version;
                {
                    // *CHANGES*: Reset joint state with an accepted reload before status.
                    std::lock_guard<std::mutex> lock(state_->mutex);
                    state_->current = std::move(candidate);
                    state_->positions.clear();
                    state_->stamp = zero_stamp();
                }
                accepted = true;
            }
        }
    } catch (const std::exception& exception) {
        error = std::string("robot description load failed: ") + exception.what();
    }
    if (!accepted && error.empty()) error = "robot description was rejected";

    // *CHANGES*: Install the new /tf tree before success status can be observed.
    // The world publisher consumes this synchronously and gates any old publication.
    if (accepted) publish_tf();

    std::shared_ptr<const LoadedRobot> loaded = current_robot();
    middleware::JsonValue status = middleware::JsonValue::make_object();
    status["version"] = middleware::JsonValue(version);
    status["accepted"] = accepted;
    status["error"] = error;
    status["loaded_version"] = middleware::JsonValue(
        loaded ? loaded->loaded_version : std::size_t{0});
    status["root_link"] = loaded ? loaded->model.root_link : "";

    middleware::JsonValue args = named_argument(
        "/robot_state_publisher/description_status");
    args["value"] = std::move(status);
    auto written = middleware_.call_service("/param_server/set_param", args);
    if (written.first) last_processed_version_ = version;
}

void DescriptionLoader::publish_tf() {
    std::shared_ptr<const LoadedRobot> loaded;
    std::unordered_map<std::string, double> positions;
    middleware::JsonValue stamp;
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        loaded = state_->current;
        if (!loaded) return;
        positions = state_->positions;
        stamp = state_->stamp;
    }

    FKResult fk = compute_fk(loaded->model, positions);
    middleware::JsonValue transforms = middleware::JsonValue::make_array();
    for (std::size_t i = 0; i < loaded->model.joints.size(); ++i) {
        const Joint& joint = loaded->model.joints[i];
        const math3d::Matrix4& matrix = fk.joint_transforms[i];
        middleware::JsonValue header = middleware::JsonValue::make_object();
        header["stamp"] = stamp;
        header["frame_id"] = joint.parent_link;

        middleware::JsonValue transform = middleware::JsonValue::make_object();
        transform["translation"] = vector_json(math3d::translation_of(matrix));
        transform["rotation"] = quaternion_json(math3d::rotation_of(matrix));

        middleware::JsonValue entry = middleware::JsonValue::make_object();
        entry["header"] = std::move(header);
        entry["child_frame_id"] = joint.child_link;
        entry["transform"] = std::move(transform);
        transforms.push_back(entry);
    }
    middleware::JsonValue message = middleware::JsonValue::make_object();
    message["transforms"] = std::move(transforms);
    middleware_.publish("/tf", message);
}

} // namespace robot
