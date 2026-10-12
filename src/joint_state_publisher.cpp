#include "joint_state_publisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <utility>

namespace robot {
namespace {

middleware::JsonValue description_request() {
    middleware::JsonValue request = middleware::JsonValue::make_object();
    request["name"] = "robot_description";
    return request;
}

double clamp_position(const Joint& joint, double value) {
    if (joint.type == JointType::Continuous) return value;
    const double lower = std::min(joint.lower_limit, joint.upper_limit);
    const double upper = std::max(joint.lower_limit, joint.upper_limit);
    return std::clamp(value, lower, upper);
}

double initial_position(const Joint& joint) {
    return clamp_position(joint, 0.0);
}

middleware::JsonValue stamp_now() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
    middleware::JsonValue stamp = middleware::JsonValue::make_object();
    stamp["sec"] = middleware::JsonValue(static_cast<double>(ns / 1000000000LL));
    stamp["nanosec"] = middleware::JsonValue(static_cast<double>(ns % 1000000000LL));
    return stamp;
}

} // namespace

JointStatePublisher::JointStatePublisher(middleware::Middleware& middleware)
    : middleware_(middleware) {}

JointStatePublisher::~JointStatePublisher() { stop(); }

void JointStatePublisher::start() {
    if (running_.exchange(true)) return;
    // Capture shared state because middleware may have copied a callback before unsubscribe.
    trajectory_subscription_ = middleware_.subscribe("/joint_trajectory",
        [state = state_](const middleware::JsonValue& message) {
            receive_trajectory(*state, message);
        });
    middleware_.advertise_service("/joint_state_publisher/reset",
        [state = state_](const middleware::JsonValue&) { return reset(*state); });
    worker_ = std::thread(&JointStatePublisher::worker_loop, this);
}

void JointStatePublisher::stop() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    if (trajectory_subscription_ != 0) {
        middleware_.unsubscribe("/joint_trajectory", trajectory_subscription_);
        trajectory_subscription_ = 0;
    }
    middleware_.unadvertise_service("/joint_state_publisher/reset");
}

void JointStatePublisher::receive_trajectory(State& state, const middleware::JsonValue& message) {
    if (!message.is_object() || !message.contains("joint_names") ||
        !message["joint_names"].is_array() || !message.contains("points") ||
        !message["points"].is_array() || message["points"].as_array().empty()) return;
    const auto& names = message["joint_names"].as_array();
    const auto& last = message["points"].as_array().back();
    if (!last.is_object() || !last.contains("positions") ||
        !last["positions"].is_array()) return;
    const auto& positions = last["positions"].as_array();
    std::lock_guard<std::mutex> lock(state.mutex);
    for (std::size_t i = 0; i < std::min(names.size(), positions.size()); ++i) {
        if (names[i].is_string() && positions[i].is_number() &&
            std::isfinite(positions[i].as_number())) {
            state.targets[names[i].as_string()] = positions[i].as_number();
        }
    }
}

std::pair<bool, middleware::JsonValue> JointStatePublisher::reset(State& state) {
    std::lock_guard<std::mutex> lock(state.mutex);
    if (state.model) {
        for (const Joint& joint : state.model->joints) {
            if (joint.type == JointType::Fixed) continue;
            const double center = joint.type == JointType::Continuous ? 0.0 :
                joint.lower_limit + (joint.upper_limit - joint.lower_limit) / 2.0;
            state.positions[joint.name] = center;
            state.targets[joint.name] = center;
        }
    }
    middleware::JsonValue values = middleware::JsonValue::make_object();
    return {true, values};
}

void JointStatePublisher::worker_loop() {
    auto next_tick = std::chrono::steady_clock::now();
    while (running_.load()) {
        try {
            poll_description();
            publish_state();
        } catch (const std::exception&) {
            // A bad description or subscriber must not stop later demo updates.
        }
        next_tick += std::chrono::milliseconds(100);
        const auto now = std::chrono::steady_clock::now();
        if (next_tick < now) next_tick = now;
        std::this_thread::sleep_until(next_tick);
    }
}

void JointStatePublisher::poll_description() {
    auto reply = middleware_.call_service("/param_server/get_param", description_request());
    if (!reply.first || !reply.second.is_object() ||
        !reply.second.contains("version") || !reply.second["version"].is_number() ||
        !reply.second.contains("value")) return;
    const double raw_version = reply.second["version"].as_number();
    if (!std::isfinite(raw_version) || raw_version < 1 ||
        std::floor(raw_version) != raw_version) return;
    const auto version = static_cast<std::size_t>(raw_version);
    if (version <= last_description_version_) return;
    last_description_version_ = version;
    const auto& value = reply.second["value"];
    if (!value.is_string()) return;
    ParseResult parsed = parse_urdf(value.as_string());
    if (!parsed) return;

    auto new_model = std::make_shared<RobotModel>(std::move(*parsed.model));
    std::lock_guard<std::mutex> lock(state_->mutex);
    std::unordered_map<std::string, double> next_positions;
    for (const Joint& joint : new_model->joints) {
        if (joint.type == JointType::Fixed) continue;
        auto previous = state_->positions.find(joint.name);
        const double value = previous == state_->positions.end() ?
            initial_position(joint) : clamp_position(joint, previous->second);
        next_positions[joint.name] = value;
    }
    state_->positions = std::move(next_positions);
    state_->model = std::move(new_model);
    state_->publish_preserved_once = true;
}

void JointStatePublisher::publish_state() {
    middleware::JsonValue names = middleware::JsonValue::make_array();
    middleware::JsonValue positions = middleware::JsonValue::make_array();
    {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->model) return;
        // *CHANGES*: Publish saved positions once after a reload before moving toward targets.
        const bool advance = !state_->publish_preserved_once;
        state_->publish_preserved_once = false;
        for (const Joint& joint : state_->model->joints) {
            if (joint.type == JointType::Fixed) continue;
            double& position = state_->positions[joint.name];
            if (advance) {
                auto target = state_->targets.find(joint.name);
                if (target != state_->targets.end()) {
                    // Snapping to the setpoint is allowed for this demo servo.
                    position = clamp_position(joint, target->second);
                }
            }
            names.push_back(joint.name);
            positions.push_back(position);
        }
    }
    middleware::JsonValue header = middleware::JsonValue::make_object();
    header["stamp"] = stamp_now();
    header["frame_id"] = "";
    middleware::JsonValue message = middleware::JsonValue::make_object();
    message["header"] = std::move(header);
    message["name"] = std::move(names);
    message["position"] = std::move(positions);
    message["velocity"] = middleware::JsonValue::make_array();
    message["effort"] = middleware::JsonValue::make_array();
    middleware_.publish("/joint_states", message);
}

} // namespace robot
