"""Tests for URDF validation and error rejection (ROBOT_DESCRIPTION.md & FK_API.md Task 6).
Verifies that invalid descriptions are rejected while preserving the loaded active robot.
"""

from helpers import load_robot, wait_for_status, get_next_world
from rosbridge_client import Rosbridge


def test_urdf_rejects_malformed_xml_and_wrong_root():
    with Rosbridge() as ros:
        cases = [
            ("non-string value", 12345),
            ("malformed unclosed XML", "<robot><link name='a'"),
            ("wrong root tag", "<machine><link name='a'/></machine>"),
            ("empty description", ""),
            ("whitespace only description", "   \r\n\t   "),
        ]
        for label, bad_val in cases:
            set_res = ros.call_service("/param_server/set_param", {"name": "robot_description", "value": bad_val})
            assert set_res.get("result") is True, f"set_param failed for {label}: {set_res}"
            ver = set_res["values"]["version"]
            st = wait_for_status(ros, ver)
            assert st["accepted"] is False, f"Expected {label} to be rejected by robot_state_publisher: {st}"
            assert bool(st.get("error")), f"Expected non-empty error string for {label}: {st}"


def test_urdf_rejects_missing_required_link_and_joint_attributes():
    with Rosbridge() as ros:
        cases = [
            ("robot with no direct-child link", "<robot></robot>"),
            ("link without name attribute", "<robot><link/></robot>"),
            ("joint without name",
             "<robot><link name='a'/><link name='b'/><joint type='fixed'><parent link='a'/><child link='b'/></joint></robot>"),
            ("joint without type",
             "<robot><link name='a'/><link name='b'/><joint name='j'><parent link='a'/><child link='b'/></joint></robot>"),
            ("joint without child element",
             "<robot><link name='a'/><joint name='j' type='fixed'><parent link='a'/></joint></robot>"),
            ("joint without parent element",
             "<robot><link name='a'/><joint name='j' type='fixed'><child link='a'/></joint></robot>"),
        ]
        for label, xml in cases:
            _, st = load_robot(ros, xml)
            assert st["accepted"] is False, f"Expected {label} to be rejected: {st}"
            assert bool(st.get("error")), f"Expected non-empty error message for {label}: {st}"


def test_urdf_rejects_invalid_joint_types_and_missing_limits():
    with Rosbridge() as ros:
        cases = [
            ("floating joint type",
             "<robot><link name='a'/><link name='b'/><joint name='j' type='floating'><parent link='a'/><child link='b'/></joint></robot>"),
            ("planar joint type",
             "<robot><link name='a'/><link name='b'/><joint name='j' type='planar'><parent link='a'/><child link='b'/></joint></robot>"),
            ("capitalized joint type",
             "<robot><link name='a'/><link name='b'/><joint name='j' type='Revolute'><parent link='a'/><child link='b'/><limit lower='-1' upper='1'/></joint></robot>"),
            ("revolute joint missing limit",
             "<robot><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/></joint></robot>"),
            ("prismatic joint missing limit",
             "<robot><link name='a'/><link name='b'/><joint name='j' type='prismatic'><parent link='a'/><child link='b'/></joint></robot>"),
            ("revolute joint non-numeric limit",
             "<robot><link name='a'/><link name='b'/><joint name='j' type='revolute'><parent link='a'/><child link='b'/><limit lower='invalid'/></joint></robot>"),
        ]
        for label, xml in cases:
            _, st = load_robot(ros, xml)
            assert st["accepted"] is False, f"Expected {label} to be rejected: {st}"
            assert bool(st.get("error")), f"Expected non-empty error for {label}: {st}"


def test_urdf_rejects_disconnected_links_multiple_roots_and_cycles():
    with Rosbridge() as ros:
        cases = [
            ("joint referencing missing parent link",
             "<robot><link name='tip'/><joint name='j' type='fixed'><parent link='ghost'/><child link='tip'/></joint></robot>"),
            ("joint referencing missing child link",
             "<robot><link name='base'/><joint name='j' type='fixed'><parent link='base'/><child link='ghost'/></joint></robot>"),
            ("multiple roots / forest of two disconnected links",
             "<robot><link name='base1'/><link name='base2'/></robot>"),
            ("kinematic cycle with no root",
             "<robot><link name='l1'/><link name='l2'/>"
             "<joint name='j1' type='fixed'><parent link='l1'/><child link='l2'/></joint>"
             "<joint name='j2' type='fixed'><parent link='l2'/><child link='l1'/></joint></robot>"),
        ]
        for label, xml in cases:
            _, st = load_robot(ros, xml)
            assert st["accepted"] is False, f"Expected {label} to be rejected: {st}"
            assert bool(st.get("error")), f"Expected non-empty error message for {label}: {st}"


def test_urdf_rejected_description_preserves_active_robot():
    with Rosbridge() as ros:
        # Load an initial valid robot
        valid_urdf = (
            "<robot name='preserved_bot'>"
            "<link name='pres_root'/><link name='pres_tip'/>"
            "<joint name='pres_joint' type='fixed'><parent link='pres_root'/><child link='pres_tip'/>"
            "<origin xyz='2.5 0 0'/></joint>"
            "</robot>"
        )
        v_good, st_good = load_robot(ros, valid_urdf)
        assert st_good["accepted"] is True, f"Failed to load valid initial robot: {st_good}"
        assert st_good["loaded_version"] == v_good, f"Status loaded_version mismatch: {st_good}"
        assert st_good["root_link"] == "pres_root", f"Status root_link mismatch: {st_good}"

        # Submit an invalid robot description
        invalid_urdf = "<robot><invalid_tag_should_fail/></robot>"
        v_bad, st_bad = load_robot(ros, invalid_urdf)
        assert st_bad["accepted"] is False, f"Invalid robot should be rejected: {st_bad}"
        assert bool(st_bad.get("error")), f"Expected non-empty error message on rejection: {st_bad}"
        assert st_bad["loaded_version"] == v_good, (
            f"loaded_version should retain previous version {v_good}, got {st_bad['loaded_version']}"
        )
        assert st_bad["root_link"] == "pres_root", (
            f"root_link should retain previous root 'pres_root', got {st_bad['root_link']}"
        )

        # Verify active robot continues streaming on /xform_world
        ros.subscribe("/xform_world", "xform_world/MatrixTransformArray")
        _, matrices = get_next_world(ros, timeout=2.0)
        assert "pres_root" in matrices and "pres_tip" in matrices, (
            f"Active robot links missing from /xform_world after failed reload attempt: {list(matrices.keys())}"
        )
