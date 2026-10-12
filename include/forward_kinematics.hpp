#ifndef PROJECT3_FORWARD_KINEMATICS_HPP
#define PROJECT3_FORWARD_KINEMATICS_HPP

#include <string>
#include <unordered_map>
#include <vector>
#include "math3d.hpp"
#include "robot_model.hpp"

namespace robot {

struct FKResult {
    // Entries align with RobotModel::joints: each is parent-link to child-link.
    std::vector<math3d::Matrix4> joint_transforms;
    // Entries align with RobotModel::links: each is root-link to this link.
    std::vector<math3d::Matrix4> link_poses;
};

math3d::Matrix4 joint_transform(const Joint& joint, double position);

FKResult compute_fk(const RobotModel& model,
                    const std::unordered_map<std::string, double>& positions = {});

} // namespace robot

#endif
