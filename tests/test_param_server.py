"""Tests for parameter server (FK_API.md Task 2 / spec).
Verifies value storage, JSON types, independent version counters, and error rejection.
"""

from rosbridge_client import Rosbridge


def test_param_set_and_get_values():
    with Rosbridge() as ros:
        test_payloads = [
            ("num_int", 42),
            ("num_float", -3.14159),
            ("str_val", "Hello Robotics EECS 367"),
            ("bool_true", True),
            ("bool_false", False),
            ("null_val", None),
            ("nested_dict", {"joints": ["j1", "j2"], "speed": 1.25, "active": True}),
            ("list_val", [1, "two", 3.0, [4, 5]]),
        ]
        for name, value in test_payloads:
            s_reply = ros.call_service("/param_server/set_param", {"name": name, "value": value})
            assert s_reply.get("result") is True, f"set_param failed for {name}: {s_reply}"
            g_reply = ros.call_service("/param_server/get_param", {"name": name})
            assert g_reply.get("result") is True, f"get_param failed for {name}: {g_reply}"
            assert g_reply["values"]["value"] == value, (
                f"Value mismatch for {name}: expected {value}, got {g_reply['values']['value']}"
            )


def test_param_independent_versioning():
    with Rosbridge() as ros:
        r1 = ros.call_service("/param_server/set_param", {"name": "ver_param_a", "value": "A1"})
        assert r1.get("result") is True, f"Failed initial set of ver_param_a: {r1}"
        v_a1 = r1["values"]["version"]

        r2 = ros.call_service("/param_server/set_param", {"name": "ver_param_b", "value": "B1"})
        assert r2.get("result") is True, f"Failed initial set of ver_param_b: {r2}"
        v_b1 = r2["values"]["version"]

        # Updating param a must increment its version independently
        r3 = ros.call_service("/param_server/set_param", {"name": "ver_param_a", "value": "A2"})
        assert r3.get("result") is True, f"Failed second set of ver_param_a: {r3}"
        assert r3["values"]["version"] == v_a1 + 1, (
            f"ver_param_a version was not incremented by 1: previous {v_a1}, new {r3['values']['version']}"
        )

        # Param b must retain its original version
        g_b = ros.call_service("/param_server/get_param", {"name": "ver_param_b"})
        assert g_b.get("result") is True, f"Failed get_param for ver_param_b: {g_b}"
        assert g_b["values"]["version"] == v_b1, (
            f"ver_param_b version leaked across parameters: expected {v_b1}, got {g_b['values']['version']}"
        )


def test_param_unset_returns_false_and_defaults():
    with Rosbridge() as ros:
        reply = ros.call_service("/param_server/get_param", {"name": "completely_unset_parameter_98765"})
        assert reply.get("result") is False, (
            f"Expected get_param for unset parameter to return result: false, got {reply}"
        )
        assert bool(reply.get("status")), (
            f"Expected non-empty status message for unset parameter, got {reply.get('status')}"
        )
        assert reply.get("values", {}).get("value") is None, (
            f"Expected null value for unset parameter, got {reply.get('values')}"
        )
        assert reply.get("values", {}).get("version") == 0, (
            f"Expected version 0 for unset parameter, got {reply.get('values')}"
        )


def test_param_invalid_name_rejected():
    with Rosbridge() as ros:
        # Missing name field
        r_no_name = ros.call_service("/param_server/set_param", {"value": "payload"})
        assert r_no_name.get("result") is False, f"set_param without name should be rejected: {r_no_name}"
        assert bool(r_no_name.get("status")), "Status message required when rejecting set_param without name"

        # Non-string name
        r_int_name = ros.call_service("/param_server/set_param", {"name": 12345, "value": "payload"})
        assert r_int_name.get("result") is False, f"set_param with numeric name should be rejected: {r_int_name}"
        assert bool(r_int_name.get("status")), "Status message required when rejecting non-string name"
