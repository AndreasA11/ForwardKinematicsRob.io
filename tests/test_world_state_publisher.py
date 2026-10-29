"""Tests for robot_world_state_publisher (FK_API.md Task 8 & 9).
Verifies root link presence, global-pose composition, column-major matrix layout, dynamic reload, and large URDF support.
"""

from helpers import (
    assert_near,
    assert_vector_near,
    assert_column_major_matrix,
    load_robot,
    reset_global_pose,
    wait_for_world_match,
    wait_for_tf_match,
)
from rosbridge_client import Rosbridge


def test_world_state_root_link_present_and_identity():
    with Rosbridge() as ros:
        bot = (
            "<robot name='root_test_bot'>"
            "<link name='base_plate'/><link name='probe'/>"
            "<joint name='j_probe' type='fixed'><parent link='base_plate'/><child link='probe'/>"
            "  <origin xyz='2 3 4'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")

        _, matrices = wait_for_world_match(
            ros, lambda msg, m: "base_plate" in m and "probe" in m
        )

        # Root link MUST be present in /xform_world
        assert "base_plate" in matrices, "Root link missing from /xform_world"
        base_m = matrices["base_plate"]
        # Default global pose is identity: translation = (0, 0, 0)
        assert_vector_near((base_m[12], base_m[13], base_m[14]), (0.0, 0.0, 0.0),
                           label="root link default translation")
        # Identity matrix diagonal = 1, off-diagonal = 0
        identity = [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        ]
        for idx in range(16):
            assert_near(base_m[idx], identity[idx], 1e-4, f"base_plate matrix[{idx}]")


def test_world_state_global_pose_translation_and_rotation():
    with Rosbridge() as ros:
        bot = (
            "<robot name='pose_comp_bot'>"
            "<link name='p_root'/><link name='p_tip'/>"
            "<joint name='p_joint' type='fixed'><parent link='p_root'/><child link='p_tip'/>"
            "  <origin xyz='1 0 0'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")
        ros.advertise("/global_pose", "geometry_msgs/Pose")

        # Global pose: position (10, 0, 5), yaw = 90 deg (z=0.7071068, w=0.7071068)
        # Child link at local (1, 0, 0) rotated by 90 deg becomes (0, 1, 0), then translated to (10, 1, 5)
        ros.publish("/global_pose", {
            "position": {"x": 10.0, "y": 0.0, "z": 5.0},
            "orientation": {"x": 0.0, "y": 0.0, "z": 0.7071067811865476, "w": 0.7071067811865476}
        })

        _, matrices = wait_for_world_match(
            ros,
            lambda msg, m: "p_root" in m and abs(m["p_root"][12] - 10.0) < 1e-3
        )

        root_m = matrices["p_root"]
        assert_vector_near((root_m[12], root_m[13], root_m[14]), (10.0, 0.0, 5.0),
                           tol=1e-3, label="p_root global translation")

        tip_m = matrices["p_tip"]
        # Expected: T_global * T_local = (10, 1, 5)
        assert_vector_near((tip_m[12], tip_m[13], tip_m[14]), (10.0, 1.0, 5.0),
                           tol=1e-3, label="p_tip composed world translation (T_global * T_local)")

        # Reset global pose back to identity for subsequent tests
        reset_global_pose(ros)


def test_world_state_matrix_column_major_layout():
    with Rosbridge() as ros:
        reset_global_pose(ros)
        bot = (
            "<robot name='layout_bot'>"
            "<link name='l_base'/><link name='l_link'/>"
            "<joint name='j_layout' type='fixed'><parent link='l_base'/><child link='l_link'/>"
            "  <origin xyz='1.5 2.5 3.5' rpy='0.2 0.4 0.6'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")

        _, matrices = wait_for_world_match(
            ros, lambda msg, m: "l_link" in m
        )

        m = matrices["l_link"]
        assert_column_major_matrix(m, label="l_link column-major format")

        # Translation must be at indices 12, 13, 14 (NOT 3, 7, 11 as in row-major!)
        assert_vector_near((m[12], m[13], m[14]), (1.5, 2.5, 3.5), tol=1e-3,
                           label="Column-major translation vector")
        # Indices 3, 7, 11 must be 0
        assert_near(m[3], 0.0, 1e-4, "matrix[3]")
        assert_near(m[7], 0.0, 1e-4, "matrix[7]")
        assert_near(m[11], 0.0, 1e-4, "matrix[11]")
        assert_near(m[15], 1.0, 1e-4, "matrix[15]")


def test_world_state_dynamic_reload_clears_old_links():
    with Rosbridge() as ros:
        # Load robot A
        bot_a = (
            "<robot name='first_generation_bot'>"
            "<link name='gen_a_root'/><link name='gen_a_child'/>"
            "<joint name='j_a' type='fixed'><parent link='gen_a_root'/><child link='gen_a_child'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot_a)
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")
        wait_for_world_match(ros, lambda msg, m: "gen_a_root" in m)

        # Load robot B with completely disjoint link names
        bot_b = (
            "<robot name='second_generation_bot'>"
            "<link name='gen_b_root'/><link name='gen_b_child'/>"
            "<joint name='j_b' type='fixed'><parent link='gen_b_root'/><child link='gen_b_child'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot_b)

        # Wait until /xform_world publishes only gen_b links and no gen_a links
        _, matrices = wait_for_world_match(
            ros, lambda msg, m: "gen_b_root" in m and "gen_b_child" in m
        )
        assert "gen_a_root" not in matrices, "Old robot link 'gen_a_root' still present after reload!"
        assert "gen_a_child" not in matrices, "Old robot link 'gen_a_child' still present after reload!"


def test_world_state_fetch_and_large_urdf_support():
    with Rosbridge() as ros:
        # 1. Fetch real robot
        with open("fetch/fetch.urdf", "r", encoding="utf-8") as f:
            fetch_text = f.read()
        v_fetch, st_fetch = load_robot(ros, fetch_text, timeout=5.0)
        assert st_fetch["accepted"] is True, f"Fetch URDF was rejected: {st_fetch}"
        assert st_fetch["root_link"] == "base_link", f"Fetch root link mismatch: {st_fetch}"

        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")
        ros.subscribe("/tf", "tf/tfMessage")

        # Verify all 26 Fetch links appear in /xform_world
        wait_for_world_match(
            ros,
            lambda msg, m: len(m) == 26 and "base_link" in m and "gripper_link" in m,
            timeout=4.0
        )
        # Verify 25 Fetch joints appear in /tf
        wait_for_tf_match(
            ros,
            lambda msg, tfs: len(tfs) == 25 and "gripper_link" in tfs,
            timeout=4.0
        )

        # 2. Large URDF (>800 KiB) with comment padding to test buffer limits
        padding = "<!-- " + ("Z" * (850 * 1024)) + " -->"
        large_urdf = (
            f"<robot name='large_scale_bot'>{padding}"
            "<link name='huge_base'/><link name='huge_tip'/>"
            "<joint name='huge_j' type='fixed'><parent link='huge_base'/><child link='huge_tip'/></joint>"
            "</robot>"
        )
        v_large, st_large = load_robot(ros, large_urdf, timeout=5.0)
        assert st_large["accepted"] is True, f"Large URDF (>800 KiB) was rejected: {st_large}"
        assert st_large["root_link"] == "huge_base"

        # Check get_param returns the large text verbatim without truncation
        g_reply = ros.call_service("/param_server/get_param", {"name": "robot_description"}, timeout=5.0)
        assert g_reply.get("result") is True, f"get_param failed for large URDF: {g_reply}"
        assert len(g_reply["values"]["value"]) == len(large_urdf), (
            f"Retrieved large URDF length mismatch: expected {len(large_urdf)}, got {len(g_reply['values']['value'])}"
        )
