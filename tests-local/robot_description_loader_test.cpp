#include "robot_description_loader.hpp"
#include "parameter_server.hpp"

#include <cassert>
#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

namespace {

using middleware::JsonValue;

JsonValue named(const std::string& name) {
    JsonValue args = JsonValue::make_object();
    args["name"] = name;
    return args;
}

std::size_t set_description(middleware::Middleware& hub, const JsonValue& value) {
    JsonValue args = named("robot_description");
    args["value"] = value;
    auto result = hub.call_service("/param_server/set_param", args);
    assert(result.first);
    return static_cast<std::size_t>(result.second["version"].as_number());
}

JsonValue wait_for_status(middleware::Middleware& hub, std::size_t version) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (std::chrono::steady_clock::now() < deadline) {
        auto reply = hub.call_service("/param_server/get_param",
                                      named("/robot_state_publisher/description_status"));
        if (reply.first) {
            JsonValue status = reply.second["value"];
            if (status["version"].as_number() >= static_cast<double>(version)) {
                return status;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    assert(false && "description status did not arrive within two seconds");
    return {};
}

} // namespace

int main() {
    middleware::Middleware hub;
    ParameterServer parameters(hub);
    robot::DescriptionLoader loader(hub);
    assert(!loader.current_robot());
    loader.start();

    std::size_t rejected_first = set_description(hub, 42);
    JsonValue status = wait_for_status(hub, rejected_first);
    assert(!status["accepted"].as_bool() && !status["error"].as_string().empty());
    assert(status["loaded_version"].as_int() == 0 && status["root_link"].as_string().empty());
    assert(!loader.current_robot());

    const std::string first_robot =
        "<robot><link name='old_child'/><joint name='old_joint' type='fixed'>"
        "<parent link='old_root'/><child link='old_child'/><origin xyz='1 2 3'/>"
        "</joint><link name='old_root'/></robot>";
    std::size_t first_version = set_description(hub, first_robot);
    status = wait_for_status(hub, first_version);
    assert(status["accepted"].as_bool() && status["error"].as_string().empty());
    assert(status["version"].as_number() == first_version);
    assert(status["loaded_version"].as_number() == first_version);
    assert(status["root_link"].as_string() == "old_root");
    auto first = loader.current_robot();
    assert(first && first->loaded_version == first_version);
    assert(first->model.links.size() == 2 && first->zero_fk.joint_transforms.size() == 1);
    assert(first->zero_fk.joint_transforms[0].data[12] == 1);

    std::size_t bad_xml = set_description(hub, "<robot><link name='broken'></robot>");
    status = wait_for_status(hub, bad_xml);
    assert(!status["accepted"].as_bool() && !status["error"].as_string().empty());
    assert(status["loaded_version"].as_number() == first_version);
    assert(status["root_link"].as_string() == "old_root");
    assert(loader.current_robot() == first);

    std::size_t bad_type = set_description(hub, JsonValue::make_array());
    status = wait_for_status(hub, bad_type);
    assert(!status["accepted"].as_bool() && !status["error"].as_string().empty());
    assert(status["loaded_version"].as_number() == first_version);
    assert(loader.current_robot() == first);

    std::size_t newer = set_description(hub, "<robot><link name='skipped'/></robot>");
    std::size_t latest = set_description(hub, "<robot><link name='new_root'/></robot>");
    assert(latest > newer);
    status = wait_for_status(hub, latest);
    assert(status["accepted"].as_bool() && status["loaded_version"].as_number() == latest);
    auto replaced = loader.current_robot();
    assert(replaced && replaced != first && replaced->model.root_link == "new_root");
    assert(replaced->model.links.size() == 1 && replaced->model.joints.empty());
    assert(!replaced->model.link_index.contains("old_root"));

    std::ifstream fetch("fetch/fetch.urdf", std::ios::binary);
    assert(fetch.good());
    std::stringstream contents;
    contents << fetch.rdbuf();
    std::size_t fetch_version = set_description(hub, contents.str());
    status = wait_for_status(hub, fetch_version);
    assert(status["accepted"].as_bool() && status["root_link"].as_string() == "base_link");
    auto real_robot = loader.current_robot();
    assert(real_robot && real_robot->model.links.size() == 26);
    assert(real_robot->zero_fk.joint_transforms.size() == 25);

    std::string large_description = "<robot><metadata>" + std::string(900000, 'x') +
                                    "</metadata><link name='large_root'/></robot>";
    std::size_t large_version = set_description(hub, large_description);
    status = wait_for_status(hub, large_version);
    assert(status["accepted"].as_bool() &&
           status["root_link"].as_string() == "large_root");

    loader.stop();

    // The loader must use observed versions even when it starts after earlier sets.
    middleware::Middleware second_hub;
    ParameterServer second_parameters(second_hub);
    set_description(second_hub, "<robot><link name='earlier'/></robot>");
    std::size_t preexisting_version = set_description(
        second_hub, "<robot><link name='preexisting'/></robot>");
    robot::DescriptionLoader second_loader(second_hub);
    assert(!second_loader.current_robot());
    second_loader.start();
    JsonValue preexisting_status = wait_for_status(second_hub, preexisting_version);
    assert(preexisting_status["accepted"].as_bool() &&
           preexisting_status["root_link"].as_string() == "preexisting");
    second_loader.stop();
}
