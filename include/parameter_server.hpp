#ifndef PROJECT3_PARAMETER_SERVER_HPP
#define PROJECT3_PARAMETER_SERVER_HPP

#include <cstddef>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include "middleware/middleware.hpp"

class ParameterServer {
public:
    explicit ParameterServer(middleware::Middleware& middleware) : middleware_(middleware) {
        middleware_.advertise_service("/param_server/set_param", [this](const middleware::JsonValue& args) {
            return set_param(args);
        });
        middleware_.advertise_service("/param_server/get_param", [this](const middleware::JsonValue& args) {
            return get_param(args);
        });
    }

    ParameterServer(const ParameterServer&) = delete;
    ParameterServer& operator=(const ParameterServer&) = delete;

    ~ParameterServer() {
        middleware_.unadvertise_service("/param_server/set_param");
        middleware_.unadvertise_service("/param_server/get_param");
    }

private:
    struct Entry {
        middleware::JsonValue value;
        std::size_t version{0};
    };

    middleware::Middleware& middleware_;
    std::mutex mutex_;
    std::unordered_map<std::string, Entry> parameters_;

    static bool valid_name(const middleware::JsonValue& args) {
        return args.is_object() && args.contains("name") && args["name"].is_string();
    }

    std::pair<bool, middleware::JsonValue> set_param(const middleware::JsonValue& args) {
        if (!valid_name(args)) {
            middleware::JsonValue values = middleware::JsonValue::make_object();
            values["error"] = "parameter name must be a string";
            return {false, values};
        }

        const std::string& name = args["name"].as_string();
        middleware::JsonValue value = args.contains("value") ? args["value"] : middleware::JsonValue();
        std::size_t version;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            Entry& entry = parameters_[name];
            entry.value = std::move(value);
            version = ++entry.version;
        }

        middleware::JsonValue values = middleware::JsonValue::make_object();
        values["version"] = middleware::JsonValue(version);
        return {true, values};
    }

    std::pair<bool, middleware::JsonValue> get_param(const middleware::JsonValue& args) {
        middleware::JsonValue values = middleware::JsonValue::make_object();
        values["value"] = middleware::JsonValue();
        values["version"] = 0;
        if (!valid_name(args)) {
            return {false, values};
        }

        std::lock_guard<std::mutex> lock(mutex_);
        auto it = parameters_.find(args["name"].as_string());
        if (it == parameters_.end()) {
            return {false, values};
        }
        values["value"] = it->second.value;
        values["version"] = middleware::JsonValue(it->second.version);
        return {true, values};
    }
};

#endif
