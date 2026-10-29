"""Tests for Forward Kinematics computations (FK_API.md Task 5 & ROBOT_DESCRIPTION.md).
Verifies RPY matrix conventions, axis normalization, prismatic scaling, and branching trees.
"""

import math
from helpers import (
    assert_near,
    assert_vector_near,
    load_robot,
    publish_joint_state,
    wait_for_tf_match,
    wait_for_world_match,
)
from rosbridge_client import Rosbridge


def test_fk_zero_config_link_poses_and_origins():
    with Rosbridge() as ros:
        # Robot with fixed-axis RPY rotations:
        # rpy = [pi/2, pi/2, 0] tests R = Ry(pi/2) * Rx(pi/2)
        # Point (0, 1, 0) should transform to (1, 0, 0)
        tree_urdf = (
            "<robot name='zero_cfg_bot'>"
            "<link name='base'/>"
            "<link name='link1'/>"
            "<link name='link2'/>"
            "<joint name='j1' type='fixed'><parent link='base'/><child link='link1'/>"
            "  <origin xyz='1 2 3' rpy='0 0 1.5707963267948966'/></joint>"
            "<joint name='j2' type='fixed'><parent link='link1'/><child link='link2'/>"
            "  <origin xyz='2 0 0' rpy='1.5707963267948966 1.5707963267948966 0'/></joint>"
            "</robot>"
        )
        load_robot(ros, tree_urdf)
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")
        _, matrices = wait_for_world_match(
            ros, lambda msg, m: {"base", "link1", "link2"}.issubset(m.keys())
        )

        # base at origin (0, 0, 0)
        base_t = (matrices["base"][12], matrices["base"][13], matrices["base"][14])
        assert_vector_near(base_t, (0.0, 0.0, 0.0), label="base translation")

        # link1 at (1, 2, 3)
        l1_t = (matrices["link1"][12], matrices["link1"][13], matrices["link1"][14])
        assert_vector_near(l1_t, (1.0, 2.0, 3.0), label="link1 translation")

        # link2: origin (2, 0, 0) in link1 rotated by 90 deg around z becomes (0, 2, 0).
        # Position in base frame is (1 + 0, 2 + 2, 3 + 0) = (1, 4, 3)
        l2_t = (matrices["link2"][12], matrices["link2"][13], matrices["link2"][14])
        assert_vector_near(l2_t, (1.0, 4.0, 3.0), label="link2 translation")


def test_fk_revolute_joint_rotation_direction_and_normalization():
    with Rosbridge() as ros:
        pi = math.pi
        # Revolute joint with non-unit axis (0, 0, 3). Spec requires normalizing to unit length (0, 0, 1).
        # Rotating point (1, 0, 0) by +pi/2 around +Z moves it to (0, 1, 0) (right-hand rule).
        urdf = (
            "<robot name='rev_norm_bot'>"
            "<link name='r_base'/>"
            "<link name='r_arm'/>"
            "<joint name='j_rev' type='revolute'><parent link='r_base'/><child link='r_arm'/>"
            "  <origin xyz='0 0 1'/><axis xyz='0 0 3'/><limit lower='-3.14' upper='3.14'/></joint>"
            "</robot>"
        )
        load_robot(ros, urdf)
        ros.subscribe("/tf", "tf/tfMessage")
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")

        # Rotate by pi/2 (90 degrees)
        publish_joint_state(ros, ["j_rev"], [pi / 2], sec=100, nanosec=1)

        # Wait for /tf to show 90 deg z-rotation quaternion (z ~ 0.7071, w ~ 0.7071)
        wait_for_tf_match(
            ros,
            lambda msg, tfs: "r_arm" in tfs and abs(abs(tfs["r_arm"]["transform"]["rotation"]["z"]) - math.sqrt(0.5)) < 1e-3
        )

        # Wait for /xform_world matching updated rotation
        _, matrices = wait_for_world_match(
            ros,
            lambda msg, m: "r_arm" in m and abs(m["r_arm"][1] - 1.0) < 1e-3
        )
        m = matrices["r_arm"]
        # Column 0 of rotation matrix is transformed X unit vector: [cos(pi/2), sin(pi/2), 0] = [0, 1, 0]
        assert_near(m[0], 0.0, 1e-3, "r_arm rot[0,0] cos(pi/2)")
        assert_near(m[1], 1.0, 1e-3, "r_arm rot[1,0] sin(pi/2) (verifies right-hand rule and unit axis)")
        assert_near(m[2], 0.0, 1e-3, "r_arm rot[2,0]")


def test_fk_prismatic_joint_translation_unnormalized():
    with Rosbridge() as ros:
        # Spec (ROBOT_DESCRIPTION.md): Prismatic joint axis is NOT normalized!
        # Axis (2, 0, 0) with q = 0.7 moves by 0.7 * (2, 0, 0) = (1.4, 0, 0).
        urdf = (
            "<robot name='prism_bot'>"
            "<link name='p_base'/>"
            "<link name='p_slider'/>"
            "<joint name='j_slide' type='prismatic'><parent link='p_base'/><child link='p_slider'/>"
            "  <origin xyz='1 2 0'/><axis xyz='2 0 0'/><limit lower='-10' upper='10'/></joint>"
            "</robot>"
        )
        load_robot(ros, urdf)
        ros.subscribe("/tf", "tf/tfMessage")

        # Translation should be origin.xyz + q * axis = (1.0 + 0.7 * 2.0, 2.0, 0.0) = (2.4, 2.0, 0.0)
        publish_joint_state(ros, ["j_slide"], [0.7], sec=200, nanosec=2)

        _, tfs = wait_for_tf_match(
            ros,
            lambda msg, tfs: "p_slider" in tfs and abs(tfs["p_slider"]["transform"]["translation"]["x"] - 2.4) < 1e-3
        )
        trans = tfs["p_slider"]["transform"]["translation"]
        assert_near(trans["x"], 2.4, 1e-3, "prismatic translation x (q * unnormalized axis)")
        assert_near(trans["y"], 2.0, 1e-3, "prismatic translation y")
        assert_near(trans["z"], 0.0, 1e-3, "prismatic translation z")


def test_fk_branching_tree_independence():
    with Rosbridge() as ros:
        pi = math.pi
        # Branch 1: base -> arm1 -> arm2
        # Branch 2: base -> camera
        branch_urdf = (
            "<robot name='branching_bot'>"
            "<link name='tree_base'/>"
            "<link name='arm_l1'/><link name='arm_l2'/>"
            "<link name='sensor_cam'/>"
            "<joint name='j_arm1' type='revolute'><parent link='tree_base'/><child link='arm_l1'/>"
            "  <origin xyz='1 0 0'/><axis xyz='0 0 1'/><limit lower='-3.14' upper='3.14'/></joint>"
            "<joint name='j_arm2' type='revolute'><parent link='arm_l1'/><child link='arm_l2'/>"
            "  <origin xyz='1 0 0'/><axis xyz='0 0 1'/><limit lower='-3.14' upper='3.14'/></joint>"
            "<joint name='j_cam' type='fixed'><parent link='tree_base'/><child link='sensor_cam'/>"
            "  <origin xyz='0 3 0'/></joint>"
            "</robot>"
        )
        load_robot(ros, branch_urdf)
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")

        # Move the arm joints
        publish_joint_state(ros, ["j_arm1", "j_arm2"], [pi / 2, -pi / 2], sec=300, nanosec=3)

        _, matrices = wait_for_world_match(
            ros,
            lambda msg, m: "sensor_cam" in m and "arm_l1" in m
        )

        cam_t = (matrices["sensor_cam"][12], matrices["sensor_cam"][13], matrices["sensor_cam"][14])
        # Camera MUST remain at (0, 3, 0) regardless of arm motion
        assert_vector_near(cam_t, (0.0, 3.0, 0.0), tol=1e-3, label="camera branch translation")
        # Camera orientation must remain identity
        assert_near(matrices["sensor_cam"][0], 1.0, 1e-3, "camera rot[0,0]")
        assert_near(matrices["sensor_cam"][5], 1.0, 1e-3, "camera rot[1,1]")
        assert_near(matrices["sensor_cam"][10], 1.0, 1e-3, "camera rot[2,2]")

        # Check arm1 at (1, 0, 0)
        arm1_t = (matrices["arm_l1"][12], matrices["arm_l1"][13], matrices["arm_l1"][14])
        assert_vector_near(arm1_t, (1.0, 0.0, 0.0), tol=1e-3, label="arm1 translation")


def test_fk_root_detection_when_declared_last():
    with Rosbridge() as ros:
        reordered_urdf = (
            "<robot name='reorder_test_bot'>"
            "<link name='child_effector'/>"
            "<joint name='j_attach' type='fixed'><parent link='actual_base'/><child link='child_effector'/>"
            "  <origin xyz='3 4 0'/></joint>"
            "<link name='actual_base'/>"
            "</robot>"
        )
        _, status = load_robot(ros, reordered_urdf)
        assert status["accepted"] is True, f"Failed to load robot with reordered links: {status}"
        assert status["root_link"] == "actual_base", (
            f"Parser erroneously chose '{status['root_link']}' instead of 'actual_base' as root link"
        )

        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")
        _, matrices = wait_for_world_match(
            ros,
            lambda msg, m: {"actual_base", "child_effector"}.issubset(m.keys())
        )
        base_t = (matrices["actual_base"][12], matrices["actual_base"][13], matrices["actual_base"][14])
        assert_vector_near(base_t, (0.0, 0.0, 0.0), label="actual_base translation")
        eff_t = (matrices["child_effector"][12], matrices["child_effector"][13], matrices["child_effector"][14])
        assert_vector_near(eff_t, (3.0, 4.0, 0.0), label="child_effector translation")
