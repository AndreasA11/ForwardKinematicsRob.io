#include "robot_model.hpp"

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>

namespace {

void accepts(const std::string& xml) {
    auto result = robot::parse_urdf(xml);
    assert(result && result.error.empty());
}

void rejects(const std::string& xml) {
    auto result = robot::parse_urdf(xml);
    assert(!result && !result.error.empty());
}

std::string robot_with_joint(const std::string& joint) {
    return "<robot><link name='a'/><link name='b'/>" + joint + "</robot>";
}

} // namespace

int main() {
    // Defaults, joint-before-link order, direct children, XML entities, and real XML forms.
    auto parsed = robot::parse_urdf(
        " \r\n<?xml version='1.0'?>\r\n"
        "<robot xmlns:xacro='http://example.org'>"
        "<!-- <joint name='fake'/> -->"
        "<joint name='hinge' type='continuous'><parent link='base&amp;one'/>"
        "<child link='tip'/><origin xyz=' +1\t.25\r\n1.5e-1' rpy='0 0 3.'/></joint>"
        "<gazebo><![CDATA[<link name='fake'/>]]><joint name='fake'/></gazebo>"
        "<link name='tip'><visual><geometry><mesh filename='package://part.dae'/></geometry></visual>"
        "<visual><geometry><capsule/></geometry></visual></link>"
        "<link name='base&amp;one'/></robot>");
    assert(parsed && parsed.model->name == "robot" && parsed.model->root_link == "base&one");
    assert(parsed.model->links.size() == 2 && parsed.model->joints.size() == 1);
    assert(parsed.model->joints[0].axis == (robot::Vector3{1, 0, 0}));
    assert(parsed.model->joints[0].origin.xyz == (robot::Vector3{1, .25, .15}));
    assert(parsed.model->links[0].visual->mesh_scale == (robot::Vector3{1, 1, 1}));

    auto referenced = robot::parse_urdf(
        "<robot xmlns='http://example.org'><joint name='j' type='fixed'>"
        "<parent link='a&#x41;'/><child link='b&#66;'/></joint>"
        "<link name='bB'/><link name='aA'/></robot>");
    assert(referenced && referenced.model->root_link == "aA");

    accepts("<robot><link name='a'/></robot>");
    accepts(robot_with_joint("<joint name='j' type='fixed'><parent link='a'/>"
                             "<child link='b'/><axis xyz='0 0 0'/><limit lower='bad'/></joint>"));
    accepts(robot_with_joint("<joint name='j' type='revolute'><parent link='a'/>"
                             "<child link='b'/><limit/></joint>"));
    accepts(robot_with_joint("<joint name='j' type='prismatic'><parent link='a'/>"
                             "<child link='b'/><limit lower='1e-400' upper='2E+0'/></joint>"));
    accepts("<robot><link name='a'><visual><origin xyz='1 2 3'/>"
            "<geometry><box size='.25 3. +1e-1'/></geometry></visual></link></robot>");
    accepts("<robot><link name='a'><visual><geometry>"
            "<cylinder radius='0.5' length='1'/></geometry></visual></link></robot>");
    accepts("<robot><link name='a'><visual><geometry>"
            "<sphere radius='0.5'/></geometry></visual></link></robot>");

    // Each numbered validity category has a failing example.
    rejects("");
    rejects("  \r\n  ");
    rejects("<robot><link name='a'></robot>");
    rejects("<world><link name='a'/></world>");
    rejects("junk<robot><link name='a'/></robot>");
    rejects("<robot><link name='a'/></robot><robot/>");
    rejects("<robot><gazebo><link name='nested'/></gazebo></robot>");
    rejects("<robot><link/></robot>");
    rejects(robot_with_joint("<joint type='fixed'><parent link='a'/><child link='b'/></joint>"));
    rejects(robot_with_joint("<joint name='j'><parent link='a'/><child link='b'/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='fixed'><child link='b'/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='fixed'><parent link='a'/>"
                             "<child/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='floating'><parent link='a'/>"
                             "<child link='b'/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='fixed'><parent link='missing'/>"
                             "<child link='b'/></joint>"));
    rejects("<robot><link name='a'/><link name='b'/></robot>");
    rejects(robot_with_joint("<joint name='j' type='revolute'><parent link='a'/>"
                             "<child link='b'/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='prismatic'><parent link='a'/>"
                             "<child link='b'/><limit lower='oops'/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='fixed'><parent link='a'/>"
                             "<child link='b'/><origin xyz='1 2'/></joint>"));
    rejects(robot_with_joint("<joint name='j' type='fixed'><parent link='a'/>"
                             "<child link='b'/><axis xyz='0,0,1'/></joint>"));
    rejects("<robot><link name='a'><visual/></link></robot>");
    rejects("<robot><link name='a'><visual><geometry><capsule/>"
            "</geometry></visual></link></robot>");
    rejects("<robot><link name='a'><visual><geometry><box size='1 2'/>"
            "</geometry></visual></link></robot>");
    rejects("<robot><link name='a'><visual><geometry><mesh/>"
            "</geometry></visual></link></robot>");
    rejects("<robot><link name='a'><visual><origin rpy='0 0 x'/>"
            "<geometry><sphere radius='1'/></geometry></visual></link></robot>");
    rejects("<robot><link name='a'><visual><geometry><mesh filename='a' scale='1 2 3 4'/>"
            "</geometry></visual></link></robot>");
    rejects("<robot><link name='a'><visual><geometry><cylinder radius='1.' length='bad'/>"
            "</geometry></visual></link></robot>");

    std::string large_unknown(1024 * 1024, 'x');
    accepts("<robot><metadata>" + large_unknown + "</metadata><link name='a'/></robot>");

    std::ifstream fetch("fetch/fetch.urdf", std::ios::binary);
    assert(fetch.good());
    std::stringstream contents;
    contents << fetch.rdbuf();
    auto real_robot = robot::parse_urdf(contents.str());
    assert(real_robot && real_robot.model->root_link == "base_link");
    assert(real_robot.model->links.size() == 26 && real_robot.model->joints.size() == 25);
}
