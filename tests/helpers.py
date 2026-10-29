"""Helper functions and assertions for Project 3 mutation challenge test suite.
Rule 2 compliance: File name does NOT start with 'test_', so run_tests.py ignores it.
"""

from __future__ import annotations

import math
import time
from typing import Any, Callable
from rosbridge_client import Rosbridge


def wait_for_status(ros: Rosbridge, min_version: int, timeout: float = 3.0) -> dict[str, Any]:
    """Poll /param_server/description_status until version >= min_version."""
    deadline = time.monotonic() + timeout
    last_val = None
    while time.monotonic() < deadline:
        reply = ros.call_service("/param_server/get_param",
                                 {"name": "/robot_state_publisher/description_status"},
                                 timeout=1.0)
        if reply.get("result"):
            val = reply.get("values", {}).get("value")
            if isinstance(val, dict):
                last_val = val
                if val.get("version", -1) >= min_version:
                    return val
        time.sleep(0.02)
    raise TimeoutError(f"Description status deadline missed for version {min_version}. Last status: {last_val}")


def load_robot(ros: Rosbridge, urdf_text: str, timeout: float = 3.0) -> tuple[int, dict[str, Any]]:
    """Set robot_description on param_server and wait for description_status."""
    set_reply = ros.call_service("/param_server/set_param",
                                 {"name": "robot_description", "value": urdf_text},
                                 timeout=2.0)
    assert set_reply.get("result") is True, f"Failed to set robot_description: {set_reply}"
    version = set_reply["values"]["version"]
    status = wait_for_status(ros, version, timeout=timeout)
    return version, status


def publish_joint_state(ros: Rosbridge,
                        names: list[str],
                        positions: list[float],
                        sec: int = 1,
                        nanosec: int = 0) -> None:
    """Publish a valid sensor_msgs/JointState with required header stamp."""
    ros.advertise("/joint_states", "sensor_msgs/JointState")
    ros.publish("/joint_states", {
        "header": {"stamp": {"sec": sec, "nanosec": nanosec}, "frame_id": ""},
        "name": names,
        "position": positions
    })


def reset_global_pose(ros: Rosbridge) -> None:
    """Reset /global_pose to identity (origin)."""
    ros.advertise("/global_pose", "geometry_msgs/Pose")
    ros.publish("/global_pose", {
        "position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    })


def get_next_tf(ros: Rosbridge, timeout: float = 3.0) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    """Receive next /tf message. Returns (msg, {child_frame_id: transform_entry})."""
    msg = ros.next_message("/tf", timeout=timeout)
    transforms = {e["child_frame_id"]: e for e in msg.get("transforms", [])}
    return msg, transforms


def get_next_world(ros: Rosbridge, timeout: float = 3.0) -> tuple[dict[str, Any], dict[str, list[float]]]:
    """Receive next /xform_world message. Returns (msg, {child_frame_id: 16-element matrix})."""
    msg = ros.next_message("/xform_world", timeout=timeout)
    matrices = {e["child_frame_id"]: e["matrix"] for e in msg.get("transforms", [])}
    return msg, matrices


def wait_for_tf_match(ros: Rosbridge,
                      predicate: Callable[[dict[str, Any], dict[str, dict[str, Any]]], bool],
                      timeout: float = 3.0) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    """Wait for a /tf message that satisfies predicate(msg, transforms_by_child)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg, tfs = get_next_tf(ros, timeout=max(0.05, deadline - time.monotonic()))
        if predicate(msg, tfs):
            return msg, tfs
    raise TimeoutError("Deadline missed waiting for matching /tf message")


def wait_for_world_match(ros: Rosbridge,
                         predicate: Callable[[dict[str, Any], dict[str, list[float]]], bool],
                         timeout: float = 3.0) -> tuple[dict[str, Any], dict[str, list[float]]]:
    """Wait for a /xform_world message that satisfies predicate(msg, matrices_by_child)."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        msg, matrices = get_next_world(ros, timeout=max(0.05, deadline - time.monotonic()))
        if predicate(msg, matrices):
            return msg, matrices
    raise TimeoutError("Deadline missed waiting for matching /xform_world message")


def assert_near(actual: float, expected: float, tol: float = 1e-4, label: str = "") -> None:
    diff = abs(actual - expected)
    assert diff <= tol, f"{label} expected {expected:g}, got {actual:g} (diff: {diff:g} > tol {tol:g})"


def assert_vector_near(actual: list[float] | tuple[float, ...],
                       expected: list[float] | tuple[float, ...],
                       tol: float = 1e-4,
                       label: str = "") -> None:
    assert len(actual) == 3 and len(expected) == 3, f"{label} vectors must have length 3"
    for i, axis in enumerate(("x", "y", "z")):
        assert_near(actual[i], expected[i], tol=tol, label=f"{label} [{axis}]")


def assert_quaternion_unit(quat: dict[str, float], tol: float = 1e-4, label: str = "") -> None:
    norm = math.sqrt(quat["x"]**2 + quat["y"]**2 + quat["z"]**2 + quat["w"]**2)
    assert abs(norm - 1.0) <= tol, f"{label} quaternion not unit: norm is {norm:g} for {quat}"


def assert_column_major_matrix(matrix: list[float], label: str = "") -> None:
    assert len(matrix) == 16, f"{label} matrix must have exactly 16 elements, got {len(matrix)}"
    # Homogeneous bottom row in column-major: indices 3, 7, 11, 15
    assert_near(matrix[3], 0.0, 1e-4, f"{label} matrix[3] (bottom row col 0)")
    assert_near(matrix[7], 0.0, 1e-4, f"{label} matrix[7] (bottom row col 1)")
    assert_near(matrix[11], 0.0, 1e-4, f"{label} matrix[11] (bottom row col 2)")
    assert_near(matrix[15], 1.0, 1e-4, f"{label} matrix[15] (bottom row col 3)")

    # Columns of 3x3 rotation block
    c0 = [matrix[0], matrix[1], matrix[2]]
    c1 = [matrix[4], matrix[5], matrix[6]]
    c2 = [matrix[8], matrix[9], matrix[10]]

    # Check unit length of rotation columns
    assert_near(math.sqrt(sum(x**2 for x in c0)), 1.0, 1e-4, f"{label} rot col 0 norm")
    assert_near(math.sqrt(sum(x**2 for x in c1)), 1.0, 1e-4, f"{label} rot col 1 norm")
    assert_near(math.sqrt(sum(x**2 for x in c2)), 1.0, 1e-4, f"{label} rot col 2 norm")

    # Check orthogonality
    assert_near(sum(a * b for a, b in zip(c0, c1)), 0.0, 1e-4, f"{label} rot col 0 . col 1")
    assert_near(sum(a * b for a, b in zip(c0, c2)), 0.0, 1e-4, f"{label} rot col 0 . col 2")
    assert_near(sum(a * b for a, b in zip(c1, c2)), 0.0, 1e-4, f"{label} rot col 1 . col 2")

    # Determinant of 3x3 rotation block
    det = (c0[0] * (c1[1] * c2[2] - c1[2] * c2[1]) -
           c1[0] * (c0[1] * c2[2] - c0[2] * c2[1]) +
           c2[0] * (c0[1] * c1[2] - c0[2] * c1[1]))
    assert_near(det, 1.0, 1e-4, f"{label} rot determinant")
