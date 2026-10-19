#ifndef PROJECT3_FINITE_STATE_MACHINE_HPP
#define PROJECT3_FINITE_STATE_MACHINE_HPP

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include "middleware/middleware.hpp"

namespace robot {

class FiniteStateMachine {
public:
    explicit FiniteStateMachine(middleware::Middleware& middleware);
    ~FiniteStateMachine();
    FiniteStateMachine(const FiniteStateMachine&) = delete;
    FiniteStateMachine& operator=(const FiniteStateMachine&) = delete;
    void start();
    void stop();

private:
    struct State {
        std::string name;
        std::vector<std::string> joint_names;
        std::vector<double> positions;
        double epsilon{0};
        double dwell_time{0.0};
        std::string on_arrival;
        std::unordered_map<std::string, std::string> transitions;
    };
    struct Shared {
        std::mutex mutex;
        middleware::JsonValue document;
        std::unordered_map<std::string, State> states;
        std::string current_state;
        middleware::JsonValue last_transition;
        std::chrono::steady_clock::time_point entered_at{std::chrono::steady_clock::now()};
        bool running{false};
    };

    middleware::Middleware& middleware_;
    std::shared_ptr<Shared> shared_{std::make_shared<Shared>()};
    std::atomic<bool> started_{false};
    std::thread worker_;
    middleware::SubscriptionId joint_subscription_{0};

    static std::pair<bool, middleware::JsonValue> validate(
        const middleware::JsonValue& document, std::unordered_map<std::string, State>& states);
    static middleware::JsonValue enter(Shared& shared,
                                       const std::string& next, const middleware::JsonValue& transition);
    static void observe(Shared& shared, middleware::Middleware& middleware,
                        const middleware::JsonValue& message);
    void publish_status();
    void worker_loop();
};

} // namespace robot
#endif
