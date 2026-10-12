#include "robot_world_state_publisher.hpp"

#include <chrono>
#include <cmath>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace robot {
namespace {

double number(const middleware::JsonValue& object, const char* key) {
    if (!object.is_object() || !object.contains(key) || !object[key].is_number() ||
        !std::isfinite(object[key].as_number())) {
        throw std::invalid_argument(std::string("invalid transform number: ") + key);
    }
    return object[key].as_number();
}

struct Edge {
    std::string parent;
    std::string child;
    math3d::Matrix4 transform;
};

middleware::JsonValue empty_stamp() {
    middleware::JsonValue stamp = middleware::JsonValue::make_object();
    stamp["sec"] = 0;
    stamp["nanosec"] = 0;
    return stamp;
}

} // namespace

WorldStatePublisher::~WorldStatePublisher() {
    stop();
}

void WorldStatePublisher::start() {
    if (running_.exchange(true)) return;
    tf_subscription_ = middleware_.subscribe("/tf",
        [state = state_](const middleware::JsonValue& message) {
            receive_tf(*state, message);
        });
    // *CHANGES*: Track the most recent global Pose separately from the latest /tf tree.
    pose_subscription_ = middleware_.subscribe("/global_pose",
        [state = state_](const middleware::JsonValue& message) {
            receive_global_pose(*state, message);
        });
    worker_ = std::thread(&WorldStatePublisher::worker_loop, this);
}

void WorldStatePublisher::stop() {
    running_.store(false);
    if (worker_.joinable()) worker_.join();
    if (tf_subscription_ != 0) {
        middleware_.unsubscribe("/tf", tf_subscription_);
        tf_subscription_ = 0;
    }
    if (pose_subscription_ != 0) {
        middleware_.unsubscribe("/global_pose", pose_subscription_);
        pose_subscription_ = 0;
    }
}

void WorldStatePublisher::receive_global_pose(
    SharedState& state, const middleware::JsonValue& message) {
    try {
        const auto& position = message["position"];
        const auto& orientation = message["orientation"];
        math3d::Vector3 xyz{
            number(position, "x"), number(position, "y"), number(position, "z")};
        math3d::Quaternion q{
            number(orientation, "x"), number(orientation, "y"),
            number(orientation, "z"), number(orientation, "w")};
        math3d::Matrix4 next = math3d::from_translation_rotation(xyz, q);
        std::lock_guard<std::mutex> lock(state.mutex);
        state.global_pose = next;
    } catch (const std::exception&) {
        // *CHANGES*: A malformed pose leaves the last valid global pose in effect.
    }
}

void WorldStatePublisher::receive_tf(SharedState& state,
                                     const middleware::JsonValue& message) {
    // A new /tf message replaces the previous tree. Invalid trees are unspecified.
    try {
        if (!message.is_object() || !message.contains("transforms") ||
            !message["transforms"].is_array()) return;
        const auto& transforms = message["transforms"].as_array();
        auto next = std::make_shared<Tree>();
        next->stamp = empty_stamp();
        if (transforms.empty()) {
            std::lock_guard<std::mutex> lock(state.mutex);
            state.latest = std::move(next);
            return;
        }

        std::vector<Edge> edges;
        edges.reserve(transforms.size());
        std::unordered_set<std::string> parents;
        std::unordered_set<std::string> children;
        std::unordered_map<std::string, std::vector<std::size_t>> outgoing;
        for (const auto& entry : transforms) {
            const auto& header = entry["header"];
            std::string parent = header["frame_id"].as_string();
            std::string child = entry["child_frame_id"].as_string();
            if (!children.insert(child).second) return;
            parents.insert(parent);
            if (edges.empty()) next->stamp = header["stamp"];
            const auto& transform = entry["transform"];
            const auto& translation = transform["translation"];
            const auto& rotation = transform["rotation"];
            math3d::Vector3 xyz{
                number(translation, "x"), number(translation, "y"), number(translation, "z")};
            math3d::Quaternion quaternion{
                number(rotation, "x"), number(rotation, "y"),
                number(rotation, "z"), number(rotation, "w")};
            outgoing[parent].push_back(edges.size());
            edges.push_back({std::move(parent), std::move(child),
                math3d::from_translation_rotation(xyz, quaternion)});
        }

        std::string root;
        std::size_t roots = 0;
        for (const std::string& parent : parents) {
            if (!children.contains(parent)) {
                root = parent;
                ++roots;
            }
        }
        if (roots != 1) return;

        std::unordered_set<std::string> reached;
        std::vector<LinkPose> pending{{root, {}}};
        reached.insert(root);
        while (!pending.empty()) {
            LinkPose current = std::move(pending.back());
            pending.pop_back();
            auto found = outgoing.find(current.name);
            if (found != outgoing.end()) {
                for (std::size_t index : found->second) {
                    const Edge& edge = edges[index];
                    if (!reached.insert(edge.child).second) return;
                    pending.push_back({edge.child,
                        math3d::multiply(current.root_pose, edge.transform)});
                }
            }
            next->links.push_back(std::move(current));
        }
        if (next->links.size() != edges.size() + 1) return;
        std::lock_guard<std::mutex> lock(state.mutex);
        state.latest = std::move(next);
    } catch (const std::exception&) {
        // Malformed /tf input must not end the middleware publisher.
    }
}

void WorldStatePublisher::worker_loop() {
    auto next_tick = std::chrono::steady_clock::now();
    while (running_.load()) {
        try {
            publish_world();
        } catch (const std::exception&) {
            // Keep future complete snapshots publishable after a malformed one.
        }
        next_tick += std::chrono::milliseconds(100);
        auto now = std::chrono::steady_clock::now();
        if (next_tick < now) next_tick = now;
        std::this_thread::sleep_until(next_tick);
    }
}

void WorldStatePublisher::publish_world() {
    // Holding the gate through delivery makes an old send finish before a new tree is installed.
    std::lock_guard<std::mutex> lock(state_->mutex);
    if (!state_->latest) return;
    middleware::JsonValue transforms = middleware::JsonValue::make_array();
    for (const LinkPose& link : state_->latest->links) {
        middleware::JsonValue header = middleware::JsonValue::make_object();
        header["stamp"] = state_->latest->stamp;
        header["frame_id"] = "global_frame";
        middleware::JsonValue matrix = middleware::JsonValue::make_array();
        // *CHANGES*: Compose the latest global root pose parent first for every link.
        math3d::Matrix4 global_link = math3d::multiply(
            state_->global_pose, link.root_pose);
        for (double value : global_link.data) matrix.push_back(value);
        middleware::JsonValue entry = middleware::JsonValue::make_object();
        entry["header"] = std::move(header);
        entry["child_frame_id"] = link.name;
        entry["matrix"] = std::move(matrix);
        transforms.push_back(entry);
    }
    middleware::JsonValue message = middleware::JsonValue::make_object();
    message["transforms"] = std::move(transforms);
    middleware_.publish("/xform_world", message);
}

} // namespace robot
