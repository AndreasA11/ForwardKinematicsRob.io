"""Tests for /joint_states subscription and /tf publishing (FK_API.md Task 7).
Verifies joint-state replacement semantics, timestamp propagation, rate, and entry completeness.
"""

import time
from helpers import (
    assert_near,
    assert_quaternion_unit,
    load_robot,
    publish_joint_state,
    wait_for_tf_match,
    get_next_tf,
)
from rosbridge_client import Rosbridge


def test_joint_state_replacement_resets_omitted_joints():
    with Rosbridge() as ros:
        bot = (
            "<robot name='movable_bot'>"
            "<link name='m_base'/>"
            "<link name='m_link1'/>"
            "<link name='m_link2'/>"
            "<joint name='joint1' type='revolute'><parent link='m_base'/><child link='m_link1'/>"
            "  <origin xyz='0 0 1'/><axis xyz='0 0 1'/><limit lower='-3.14' upper='3.14'/></joint>"
            "<joint name='joint2' type='prismatic'><parent link='m_link1'/><child link='m_link2'/>"
            "  <origin xyz='0 0 0'/><axis xyz='1 0 0'/><limit lower='-10' upper='10'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/tf", "tf/tfMessage")

        # Step 1: Set both joints to non-zero values
        publish_joint_state(ros, ["joint1", "joint2"], [1.5707963, 2.5], sec=10, nanosec=0)
        wait_for_tf_match(
            ros,
            lambda msg, tfs: "m_link2" in tfs and abs(tfs["m_link2"]["transform"]["translation"]["x"] - 2.5) < 1e-3
        )

        # Step 2: Publish message omitting joint2.
        # Spec: Each message replaces state; omitted movable joints MUST reset to 0.0.
        publish_joint_state(ros, ["joint1"], [0.0], sec=11, nanosec=0)
        _, tfs = wait_for_tf_match(
            ros,
            lambda msg, tfs: "m_link2" in tfs and abs(tfs["m_link2"]["transform"]["translation"]["x"] - 0.0) < 1e-3
        )
        assert_near(tfs["m_link2"]["transform"]["translation"]["x"], 0.0, 1e-3,
                    "Omitted joint2 must reset to 0.0 under replacement semantics")


def test_joint_state_timestamp_propagation():
    with Rosbridge() as ros:
        bot = (
            "<robot name='stamp_bot'>"
            "<link name='s_base'/><link name='s_tip'/>"
            "<joint name='s_joint' type='fixed'><parent link='s_base'/><child link='s_tip'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/tf", "tf/tfMessage")

        test_sec = 12345
        test_nanosec = 67890
        publish_joint_state(ros, ["s_joint"], [0.0], sec=test_sec, nanosec=test_nanosec)

        msg, _ = wait_for_tf_match(
            ros,
            lambda m, tfs: any(
                e.get("header", {}).get("stamp", {}).get("sec") == test_sec for e in m.get("transforms", [])
            )
        )
        for entry in msg.get("transforms", []):
            stamp = entry.get("header", {}).get("stamp", {})
            assert stamp.get("sec") == test_sec, (
                f"Stamp sec mismatch in /tf: expected {test_sec}, got {stamp.get('sec')}"
            )
            assert stamp.get("nanosec") == test_nanosec, (
                f"Stamp nanosec mismatch in /tf: expected {test_nanosec}, got {stamp.get('nanosec')}"
            )


def test_joint_state_ignores_unknown_and_fixed_joints():
    with Rosbridge() as ros:
        bot = (
            "<robot name='filter_bot'>"
            "<link name='f_base'/><link name='f_mid'/><link name='f_tip'/>"
            "<joint name='j_fixed' type='fixed'><parent link='f_base'/><child link='f_mid'/>"
            "  <origin xyz='1 0 0'/></joint>"
            "<joint name='j_rev' type='revolute'><parent link='f_mid'/><child link='f_tip'/>"
            "  <origin xyz='0 1 0'/><axis xyz='0 0 1'/><limit lower='-3.14' upper='3.14'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/tf", "tf/tfMessage")

        # Send unknown joint name and attempt to set position for fixed joint
        publish_joint_state(ros, ["j_rev", "j_fixed", "nonexistent_joint_abc"],
                            [1.5707963, 999.0, -123.0], sec=20, nanosec=0)

        _, tfs = wait_for_tf_match(
            ros,
            lambda msg, tfs: "f_tip" in tfs and abs(tfs["f_tip"]["transform"]["rotation"]["z"]) > 0.5
        )
        # Fixed joint must remain at origin translation (1, 0, 0)
        assert_near(tfs["f_mid"]["transform"]["translation"]["x"], 1.0, 1e-4, "Fixed joint x translation")
        assert_near(tfs["f_mid"]["transform"]["translation"]["y"], 0.0, 1e-4, "Fixed joint y translation")
        # Fixed joint must have identity rotation
        assert_near(tfs["f_mid"]["transform"]["rotation"]["w"], 1.0, 1e-4, "Fixed joint rot w")


def test_tf_completeness_and_rate():
    with Rosbridge() as ros:
        bot = (
            "<robot name='rate_bot'>"
            "<link name='base_link'/><link name='l1'/><link name='l2'/>"
            "<joint name='j1' type='revolute'><parent link='base_link'/><child link='l1'/>"
            "  <limit lower='-1' upper='1'/></joint>"
            "<joint name='j2' type='fixed'><parent link='l1'/><child link='l2'/></joint>"
            "</robot>"
        )
        load_robot(ros, bot)
        ros.subscribe("/tf", "tf/tfMessage")

        # Check completeness of message
        msg, tfs = wait_for_tf_match(ros, lambda m, tfs: len(tfs) == 2)
        assert "base_link" not in tfs, "Root link must NOT have an entry in /tf"
        assert "l1" in tfs and "l2" in tfs, "Every non-root child link must have an entry in /tf"

        # Check parent-child frame IDs
        assert tfs["l1"]["header"]["frame_id"] == "base_link"
        assert tfs["l2"]["header"]["frame_id"] == "l1"

        # Check rotation quaternion normalization
        assert_quaternion_unit(tfs["l1"]["transform"]["rotation"], label="j1 tf rotation")
        assert_quaternion_unit(tfs["l2"]["transform"]["rotation"], label="j2 tf rotation")

        # Rate check: at least 5 Hz (spec: FK_API.md line 220)
        t0 = time.monotonic()
        count = 0
        while count < 5 and (time.monotonic() - t0) < 2.0:
            get_next_tf(ros, timeout=1.0)
            count += 1
        elapsed = time.monotonic() - t0
        rate = (count - 1) / elapsed
        assert rate >= 4.5, f"Publishing rate on /tf is below 5 Hz: measured {rate:.1f} Hz"
