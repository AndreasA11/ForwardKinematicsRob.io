#include "forward_kinematics.hpp"

#include <cassert>
#include <cmath>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>

namespace {

constexpr double pi = std::numbers::pi_v<double>;

void near(double actual, double expected) {
    assert(std::abs(actual - expected) < 1e-10);
}

void near_vector(const math3d::Vector3& actual, const math3d::Vector3& expected) {
    for (int i = 0; i < 3; ++i) near(actual[i], expected[i]);
}

void near_matrix(const math3d::Matrix4& actual, const math3d::Matrix4& expected) {
    for (std::size_t i = 0; i < actual.data.size(); ++i) {
        near(actual.data[i], expected.data[i]);
    }
}

const math3d::Matrix4& pose(const robot::RobotModel& model,
                            const robot::FKResult& fk, const std::string& link) {
    return fk.link_poses[model.link_index.at(link)];
}

} // namespace

int main() {
    const std::string tree =
        "<robot>"
        "<link name='tip'/>"
        "<joint name='tool' type='fixed'><parent link='arm'/><child link='tip'/>"
        "<origin xyz='2 0 0'/><axis xyz='0 0 0'/></joint>"
        "<link name='branch'/>"
        "<joint name='branch_joint' type='fixed'><parent link='base'/><child link='branch'/>"
        "<origin xyz='0 3 0'/></joint>"
        "<joint name='shoulder' type='revolute'><parent link='base'/><child link='arm'/>"
        "<origin xyz='1 0 0'/><axis xyz='0 0 2'/>"
        "<limit lower='-0.1' upper='0.1'/></joint>"
        "<link name='arm'/><link name='base'/></robot>";
    auto parsed = robot::parse_urdf(tree);
    assert(parsed && parsed.model->root_link == "base" && parsed.model->root_index == 3);
    const robot::RobotModel& model = *parsed.model;

    robot::FKResult zero = robot::compute_fk(model);
    assert(zero.joint_transforms.size() == 3 && zero.link_poses.size() == 4);
    near_matrix(pose(model, zero, "base"), {});
    near_vector(math3d::translation_of(pose(model, zero, "arm")), {1, 0, 0});
    near_vector(math3d::translation_of(pose(model, zero, "tip")), {3, 0, 0});
    near_vector(math3d::translation_of(pose(model, zero, "branch")), {0, 3, 0});
    for (std::size_t i = 0; i < model.joints.size(); ++i) {
        near_matrix(zero.joint_transforms[i], math3d::from_origin(
            model.joints[i].origin.xyz, model.joints[i].origin.rpy));
    }

    robot::FKResult moved = robot::compute_fk(model,
        {{"shoulder", pi / 2}, {"tool", 100}, {"unknown", 5}});
    near_vector(math3d::translation_of(moved.joint_transforms[2]), {1, 0, 0});
    near_vector(math3d::translation_of(pose(model, moved, "tip")), {1, 2, 0});
    near_vector(math3d::transform_point(pose(model, moved, "tip"), {1, 0, 0}), {1, 3, 0});
    near_vector(math3d::translation_of(pose(model, moved, "branch")), {0, 3, 0});
    near_matrix(moved.joint_transforms[0], zero.joint_transforms[0]);

    const std::string slide_xml =
        "<robot><link name='base'/><link name='slider'/>"
        "<joint name='slide' type='prismatic'><parent link='base'/>"
        "<child link='slider'/><origin xyz='1 2 0' rpy='0 0 1.5707963267948966'/>"
        "<axis xyz='2 0 0'/><limit lower='-0.1' upper='0.1'/></joint></robot>";
    auto slide = robot::parse_urdf(slide_xml);
    assert(slide);
    auto slid = robot::compute_fk(*slide.model, {{"slide", 0.7}});
    near_vector(math3d::translation_of(pose(*slide.model, slid, "slider")), {1, 3.4, 0});

    const std::string rotated_origin =
        "<robot><link name='base'/><link name='child'/>"
        "<joint name='spin' type='continuous'><parent link='base'/><child link='child'/>"
        "<origin xyz='1 2 0' rpy='0 0 1.5707963267948966'/>"
        "<axis xyz='2 0 0'/></joint></robot>";
    auto spin = robot::parse_urdf(rotated_origin);
    assert(spin);
    auto spun = robot::compute_fk(*spin.model, {{"spin", pi / 2}});
    near_vector(math3d::translation_of(pose(*spin.model, spun, "child")), {1, 2, 0});
    near_vector(math3d::transform_point(pose(*spin.model, spun, "child"), {0, 1, 0}),
                {1, 2, 1});
    auto beyond_pi = robot::compute_fk(*spin.model, {{"spin", 4.0}});
    near_vector(math3d::transform_point(pose(*spin.model, beyond_pi, "child"), {0, 1, 0}),
                {1 - std::cos(4.0), 2, std::sin(4.0)});

    // A zero movable axis is unspecified by the course, but must not crash or hang.
    auto zero_axis = robot::parse_urdf(
        "<robot><link name='a'/><link name='b'/><joint name='j' type='continuous'>"
        "<parent link='a'/><child link='b'/><axis xyz='0 0 0'/></joint></robot>");
    assert(zero_axis);
    near_matrix(robot::compute_fk(*zero_axis.model, {{"j", 3.0}}).joint_transforms[0], {});

    auto lone = robot::parse_urdf("<robot><link name='alone'/></robot>");
    assert(lone);
    auto lone_fk = robot::compute_fk(*lone.model);
    assert(lone_fk.joint_transforms.empty() && lone_fk.link_poses.size() == 1);
    near_matrix(lone_fk.link_poses[0], {});

    std::ifstream fetch("fetch/fetch.urdf", std::ios::binary);
    assert(fetch.good());
    std::stringstream contents;
    contents << fetch.rdbuf();
    auto real_robot = robot::parse_urdf(contents.str());
    assert(real_robot);
    auto real_fk = robot::compute_fk(*real_robot.model);
    assert(real_fk.joint_transforms.size() == 25 && real_fk.link_poses.size() == 26);
    for (std::size_t i = 0; i < real_fk.joint_transforms.size(); ++i) {
        near_matrix(real_fk.joint_transforms[i], math3d::from_origin(
            real_robot.model->joints[i].origin.xyz, real_robot.model->joints[i].origin.rpy));
    }
}
