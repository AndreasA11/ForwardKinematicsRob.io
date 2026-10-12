#ifndef PROJECT3_JOINT_STATE_PUBLISHER_HPP
#define PROJECT3_JOINT_STATE_PUBLISHER_HPP

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include "middleware/middleware.hpp"
#include "robot_model.hpp"

namespace robot {

class JointStatePublisher {
public:
    explicit JointStatePublisher(middleware::Middleware& middleware);
    ~JointStatePublisher();
    JointStatePublisher(const JointStatePublisher&) = delete;
    JointStatePublisher& operator=(const JointStatePublisher&) = delete;

    void start();
    void stop();

private:
    struct State {
        std::mutex mutex;
        std::shared_ptr<const RobotModel> model;
        std::unordered_map<std::string, double> positions;
        std::unordered_map<std::string, double> targets;
        bool publish_preserved_once{false};
    };

    middleware::Middleware& middleware_;
    std::shared_ptr<State> state_{std::make_shared<State>()};
    std::atomic<bool> running_{false};
    std::thread worker_;
    middleware::SubscriptionId trajectory_subscription_{0};
    std::size_t last_description_version_{0};

    static void receive_trajectory(State& state, const middleware::JsonValue& message);
    static std::pair<bool, middleware::JsonValue> reset(State& state);
    void worker_loop();
    void poll_description();
    void publish_state();
};

} // namespace robot
#endif
