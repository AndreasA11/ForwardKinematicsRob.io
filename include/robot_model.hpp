#ifndef PROJECT3_ROBOT_MODEL_HPP
#define PROJECT3_ROBOT_MODEL_HPP

#include <array>
#include <cstddef>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot {

using Vector3 = std::array<double, 3>;

struct Origin {
    Vector3 xyz{0.0, 0.0, 0.0};
    Vector3 rpy{0.0, 0.0, 0.0};
};

enum class GeometryType { Box, Cylinder, Sphere, Mesh };

struct Visual {
    Origin origin;
    GeometryType type{GeometryType::Box};
    Vector3 box_size{0.0, 0.0, 0.0};
    Vector3 mesh_scale{1.0, 1.0, 1.0};
    double radius{0.0};
    double length{0.0};
    std::string mesh_filename;
};

struct Link {
    std::string name;
    std::optional<Visual> visual;
};

enum class JointType { Revolute, Continuous, Prismatic, Fixed };

struct Joint {
    std::string name;
    JointType type{JointType::Fixed};
    std::string parent_link;
    std::string child_link;
    std::size_t parent_index{0};
    std::size_t child_index{0};
    Origin origin;
    Vector3 axis{1.0, 0.0, 0.0};
    double lower_limit{0.0};
    double upper_limit{0.0};
    bool has_limit{false};
};

struct RobotModel {
    std::string name{"robot"};
    std::vector<Link> links;
    std::vector<Joint> joints;
    std::unordered_map<std::string, std::size_t> link_index;
    std::vector<std::vector<std::size_t>> child_joints;
    std::size_t root_index{0};
    std::string root_link;
};

struct ParseResult {
    std::optional<RobotModel> model;
    std::string error;

    explicit operator bool() const { return model.has_value(); }
};

ParseResult parse_urdf(const std::string& description);

} // namespace robot

#endif
