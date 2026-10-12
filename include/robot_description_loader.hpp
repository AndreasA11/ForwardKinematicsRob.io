#ifndef PROJECT3_ROBOT_DESCRIPTION_LOADER_HPP
#define PROJECT3_ROBOT_DESCRIPTION_LOADER_HPP

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <unordered_map>
#include "forward_kinematics.hpp"
#include "middleware/middleware.hpp"
#include "robot_model.hpp"

namespace robot {

struct LoadedRobot {
    RobotModel model;
    FKResult zero_fk;
    std::size_t loaded_version{0};
};

class DescriptionLoader {
public:
    explicit DescriptionLoader(middleware::Middleware& middleware) : middleware_(middleware) {}
    ~DescriptionLoader();

    DescriptionLoader(const DescriptionLoader&) = delete;
    DescriptionLoader& operator=(const DescriptionLoader&) = delete;

    void start();
    void stop();
    std::shared_ptr<const LoadedRobot> current_robot() const;

private:
    // *CHANGES*: Keep the loaded model, joint positions, and stamp in one snapshot lock.
    struct SharedState {
        std::mutex mutex;
        std::shared_ptr<const LoadedRobot> current;
        std::unordered_map<std::string, double> positions;
        middleware::JsonValue stamp;

        SharedState() {
            stamp = middleware::JsonValue::make_object();
            stamp["sec"] = 0;
            stamp["nanosec"] = 0;
        }
    };

    middleware::Middleware& middleware_;
    std::shared_ptr<SharedState> state_{std::make_shared<SharedState>()};
    std::atomic<bool> running_{false};
    std::thread worker_;
    middleware::SubscriptionId joint_subscription_{0};
    std::size_t last_processed_version_{0}; // worker thread only

    static void receive_joint_state(SharedState& state, const middleware::JsonValue& message);
    void worker_loop();
    void poll_once();
    void publish_tf();
};

} // namespace robot

#endif
