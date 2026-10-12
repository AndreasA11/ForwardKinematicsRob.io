#include "forward_kinematics.hpp"

#include <cstddef>
#include <stdexcept>
#include <vector>

namespace robot {

math3d::Matrix4 joint_transform(const Joint& joint, double position) {
    math3d::Matrix4 origin = math3d::from_origin(joint.origin.xyz, joint.origin.rpy);
    if (joint.type == JointType::Fixed || position == 0.0) {
        return origin;
    }

    math3d::Matrix4 motion;
    if (joint.type == JointType::Revolute || joint.type == JointType::Continuous) {
        // A zero movable axis has unspecified URDF semantics; identity keeps FK safe.
        if (math3d::norm(joint.axis) == 0.0) return origin;
        motion = math3d::from_translation_rotation(
            {0.0, 0.0, 0.0}, math3d::from_axis_angle(joint.axis, position));
    } else if (joint.type == JointType::Prismatic) {
        // The URDF axis is used as written here, with no normalization.
        motion = math3d::from_translation_rotation(
            math3d::scale(joint.axis, position), {});
    }
    return math3d::multiply(origin, motion);
}

FKResult compute_fk(const RobotModel& model,
                    const std::unordered_map<std::string, double>& positions) {
    if (model.links.empty() || model.root_index >= model.links.size() ||
        model.child_joints.size() != model.links.size()) {
        throw std::invalid_argument("invalid robot tree shape");
    }

    FKResult result;
    result.joint_transforms.resize(model.joints.size());
    result.link_poses.resize(model.links.size());
    std::vector<bool> reached(model.links.size(), false);
    std::vector<std::size_t> pending{model.root_index};
    reached[model.root_index] = true;
    std::size_t visited_links = 0;
    std::size_t visited_joints = 0;

    while (!pending.empty()) {
        std::size_t parent_index = pending.back();
        pending.pop_back();
        ++visited_links;
        for (std::size_t joint_index : model.child_joints[parent_index]) {
            if (joint_index >= model.joints.size()) {
                throw std::invalid_argument("robot tree has an invalid joint index");
            }
            const Joint& joint = model.joints[joint_index];
            if (joint.parent_index != parent_index || joint.child_index >= model.links.size() ||
                reached[joint.child_index]) {
                throw std::invalid_argument("robot tree has an invalid parent-child edge");
            }
            auto found = positions.find(joint.name);
            double position = found == positions.end() ? 0.0 : found->second;
            result.joint_transforms[joint_index] = joint_transform(joint, position);
            result.link_poses[joint.child_index] = math3d::multiply(
                result.link_poses[parent_index], result.joint_transforms[joint_index]);
            reached[joint.child_index] = true;
            pending.push_back(joint.child_index);
            ++visited_joints;
        }
    }

    if (visited_links != model.links.size() || visited_joints != model.joints.size()) {
        throw std::invalid_argument("robot tree is disconnected");
    }
    return result;
}

} // namespace robot
