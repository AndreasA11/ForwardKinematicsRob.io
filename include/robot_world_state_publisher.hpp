#ifndef PROJECT3_ROBOT_WORLD_STATE_PUBLISHER_HPP
#define PROJECT3_ROBOT_WORLD_STATE_PUBLISHER_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "math3d.hpp"
#include "middleware/middleware.hpp"

namespace robot {

class WorldStatePublisher {
public:
    explicit WorldStatePublisher(middleware::Middleware& middleware) : middleware_(middleware) {}
    ~WorldStatePublisher();

    WorldStatePublisher(const WorldStatePublisher&) = delete;
    WorldStatePublisher& operator=(const WorldStatePublisher&) = delete;

    void start();
    void stop();

private:
    struct LinkPose {
        std::string name;
        math3d::Matrix4 root_pose;
    };

    struct Tree {
        std::vector<LinkPose> links;
        middleware::JsonValue stamp;
    };

    struct SharedState {
        // Also gates publication, so a replaced tree cannot publish after status.
        std::mutex mutex;
        std::shared_ptr<const Tree> latest;
        // *CHANGES*: Retain the latest global root pose independently of /tf reloads.
        math3d::Matrix4 global_pose;
    };

    middleware::Middleware& middleware_;
    std::shared_ptr<SharedState> state_{std::make_shared<SharedState>()};
    middleware::SubscriptionId tf_subscription_{0};
    middleware::SubscriptionId pose_subscription_{0};
    std::atomic<bool> running_{false};
    std::thread worker_;

    static void receive_tf(SharedState& state, const middleware::JsonValue& message);
    static void receive_global_pose(SharedState& state, const middleware::JsonValue& message);
    void worker_loop();
    void publish_world();
};

} // namespace robot

#endif
