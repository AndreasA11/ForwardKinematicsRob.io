"""Comprehensive black-box integration test suite for Project 3.
Verifies all external TCP/JSON contracts on 127.0.0.1:9095 according to spec/FK_API.md
and Task 12 requirements. Run after `make build`.
"""

import json
import os
import signal
import socket
import subprocess
import time


class Client:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b""
        self.next_id = 0

    def send(self, message):
        self.sock.sendall((json.dumps(message) + "\n").encode("utf-8"))

    def read(self, timeout=2.0):
        deadline = time.monotonic() + timeout
        while b"\n" not in self.buffer:
            rem = deadline - time.monotonic()
            if rem <= 0:
                raise TimeoutError("read timeout")
            self.sock.settimeout(max(0.001, rem))
            try:
                chunk = self.sock.recv(65536)
            except (socket.timeout, TimeoutError):
                raise TimeoutError("read timeout")
            if not chunk:
                raise RuntimeError("gateway closed the connection")
            self.buffer += chunk
        line, self.buffer = self.buffer.split(b"\n", 1)
        return json.loads(line)

    def call(self, service, args):
        self.next_id += 1
        call_id = f"call-{self.next_id}"
        self.send({"op": "call_service", "service": service, "id": call_id, "args": args})
        while True:
            response = self.read()
            if response.get("op") == "service_response" and response.get("id") == call_id:
                return response

    def subscribe(self, topic, message_type=""):
        self.send({"op": "subscribe", "topic": topic, "type": message_type})
        while self.read().get("op") != "status":
            pass

    def next_publish(self, topic, timeout=2.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.read(deadline - time.monotonic())
            if message.get("op") == "publish" and message.get("topic") == topic:
                return message["msg"]
        raise AssertionError(f"no publication on {topic}")

    def assert_no_publish(self, topic, timeout=0.5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                message = self.read(max(0.01, deadline - time.monotonic()))
                if message.get("op") == "publish" and message.get("topic") == topic:
                    raise AssertionError(f"unexpected publication on {topic}")
            except (TimeoutError, socket.timeout):
                break


def connect(process):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        try:
            return Client(socket.create_connection(("127.0.0.1", 9095), 0.5))
        except OSError:
            if process.poll() is not None:
                raise RuntimeError("make run exited before the gateway was ready")
            time.sleep(0.05)
    raise AssertionError("gateway was not ready within 10 seconds")


def status_for(client, version):
    deadline = time.monotonic() + 3
    while time.monotonic() < deadline:
        reply = client.call("/param_server/get_param",
                            {"name": "/robot_state_publisher/description_status"})
        if reply.get("result"):
            val = reply.get("values", {}).get("value", {})
            if isinstance(val, dict) and val.get("version", -1) >= version:
                return val
        time.sleep(0.02)
    raise AssertionError(f"description status deadline missed for version {version}")


def children(message):
    return {entry["child_frame_id"] for entry in message.get("transforms", [])}


def test_1_parameter_server(caller):
    print("Testing Item 1: Parameter set/get behavior, independent versions, rejection...")
    r1 = caller.call("/param_server/set_param", {"name": "test_param_a", "value": "val_a"})
    assert r1["result"], "set param a failed"
    v_a1 = r1["values"]["version"]

    r2 = caller.call("/param_server/set_param", {"name": "test_param_b", "value": 42})
    assert r2["result"], "set param b failed"
    v_b1 = r2["values"]["version"]

    # Independent versions
    r3 = caller.call("/param_server/set_param", {"name": "test_param_a", "value": {"nested": [True, None]}})
    assert r3["result"]
    assert r3["values"]["version"] == v_a1 + 1, "version was not incremented independently"

    # Get parameters
    g_a = caller.call("/param_server/get_param", {"name": "test_param_a"})
    assert g_a["result"]
    assert g_a["values"]["value"] == {"nested": [True, None]}
    assert g_a["values"]["version"] == v_a1 + 1

    g_b = caller.call("/param_server/get_param", {"name": "test_param_b"})
    assert g_b["result"]
    assert g_b["values"]["value"] == 42
    assert g_b["values"]["version"] == v_b1

    # Unset parameter
    g_unset = caller.call("/param_server/get_param", {"name": "non_existent_param_99999"})
    assert not g_unset["result"]
    assert g_unset.get("status")
    assert g_unset["values"]["value"] is None
    assert g_unset["values"]["version"] == 0

    # Missing name
    s_bad = caller.call("/param_server/set_param", {"value": "no_name"})
    assert not s_bad["result"] and s_bad.get("status")

    # Non-string name
    s_bad2 = caller.call("/param_server/set_param", {"name": 12345, "value": "numeric_name"})
    assert not s_bad2["result"] and s_bad2.get("status")
    print("  Item 1 passed.")


def test_2_rejection_conditions(caller):
    print("Testing Item 2: Description acceptance and rejection conditions...")
    # Non-string
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": 12345})
    st = status_for(caller, r["values"]["version"])
    assert not st["accepted"] and st["error"] != "", "non-string description was not rejected"

    # Malformed XML
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": "<robot><link"})
    st = status_for(caller, r["values"]["version"])
    assert not st["accepted"] and st["error"] != "", "malformed XML was not rejected"

    # Wrong root
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": "<machine><link name='a'/></machine>"})
    st = status_for(caller, r["values"]["version"])
    assert not st["accepted"] and st["error"] != "", "non-robot root was not rejected"

    # Link without name
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": "<robot><link/></robot>"})
    st = status_for(caller, r["values"]["version"])
    assert not st["accepted"] and st["error"] != "", "nameless link was not rejected"

    # Joint referencing missing link
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value":
        "<robot><link name='l1'/><joint name='j' type='fixed'><parent link='l1'/><child link='missing'/></joint></robot>"})
    st = status_for(caller, r["values"]["version"])
    assert not st["accepted"] and st["error"] != "", "joint referencing missing link was not rejected"

    # Revolute joint without limit
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value":
        "<robot><link name='l1'/><link name='l2'/><joint name='j' type='revolute'><parent link='l1'/><child link='l2'/></joint></robot>"})
    st = status_for(caller, r["values"]["version"])
    assert not st["accepted"] and st["error"] != "", "revolute joint without limit was not rejected"
    print("  Item 2 passed.")


def test_3_preserve_on_failed_load(caller, tf_sub, world_sub):
    print("Testing Item 3: Failed description loads preserving active robot...")
    valid_urdf = (
        "<robot name='preserved_bot'>"
        "<link name='pres_root'/><link name='pres_tip'/>"
        "<joint name='pres_joint' type='fixed'><parent link='pres_root'/><child link='pres_tip'/><origin xyz='2 0 0'/></joint>"
        "</robot>"
    )
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": valid_urdf})
    v_good = r["values"]["version"]
    st = status_for(caller, v_good)
    assert st["accepted"] and st["loaded_version"] == v_good and st["root_link"] == "pres_root"

    # Send invalid description
    r_bad = caller.call("/param_server/set_param", {"name": "robot_description", "value": "<robot><invalid/></robot>"})
    v_bad = r_bad["values"]["version"]
    st_bad = status_for(caller, v_bad)
    assert not st_bad["accepted"]
    assert st_bad["loaded_version"] == v_good
    assert st_bad["root_link"] == "pres_root"

    # Verify TF and WorldState still stream preserved_bot
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        w_msg = world_sub.next_publish("/xform_world")
        if children(w_msg) == {"pres_root", "pres_tip"}:
            break
    else:
        raise AssertionError("pres_root/pres_tip was not preserved")
    print("  Item 3 passed.")


def test_4_zero_config_fk(caller, tf_sub, world_sub):
    print("Testing Item 4: Zero-configuration FK with chains, fixed joints, branching trees...")
    tree_urdf = (
        "<robot name='tree_bot'>"
        "<link name='base'/>"
        "<link name='arm1'/>"
        "<link name='arm2'/>"
        "<link name='cam'/>"
        "<joint name='j_arm1' type='revolute'><parent link='base'/><child link='arm1'/>"
        "  <origin xyz='0 0 1' rpy='0 0 0'/><axis xyz='0 0 1'/><limit lower='-3.14' upper='3.14'/></joint>"
        "<joint name='j_arm2' type='fixed'><parent link='arm1'/><child link='arm2'/>"
        "  <origin xyz='1 0 0' rpy='0 0 0'/></joint>"
        "<joint name='j_cam' type='fixed'><parent link='base'/><child link='cam'/>"
        "  <origin xyz='0 1 0' rpy='0 0 1.5707963267948966'/></joint>"
        "</robot>"
    )
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": tree_urdf})
    v = r["values"]["version"]
    st = status_for(caller, v)
    assert st["accepted"] and st["root_link"] == "base"

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        w_msg = world_sub.next_publish("/xform_world")
        if children(w_msg) == {"base", "arm1", "arm2", "cam"}:
            matrices = {e["child_frame_id"]: e["matrix"] for e in w_msg["transforms"]}
            # base at (0, 0, 0)
            assert matrices["base"][12:15] == [0.0, 0.0, 0.0]
            # arm1 at (0, 0, 1)
            assert abs(matrices["arm1"][12] - 0.0) < 1e-4
            assert abs(matrices["arm1"][13] - 0.0) < 1e-4
            assert abs(matrices["arm1"][14] - 1.0) < 1e-4
            # arm2 at (1, 0, 1)
            assert abs(matrices["arm2"][12] - 1.0) < 1e-4
            assert abs(matrices["arm2"][13] - 0.0) < 1e-4
            assert abs(matrices["arm2"][14] - 1.0) < 1e-4
            # cam at (0, 1, 0) rotated by 90 deg around z
            assert abs(matrices["cam"][12] - 0.0) < 1e-4
            assert abs(matrices["cam"][13] - 1.0) < 1e-4
            assert abs(matrices["cam"][14] - 0.0) < 1e-4
            # Check rotation matrix of cam: column 0 = [cos, sin, 0, 0] = [0, 1, 0, 0]
            assert abs(matrices["cam"][0] - 0.0) < 1e-4
            assert abs(matrices["cam"][1] - 1.0) < 1e-4
            break
    else:
        raise AssertionError("Tree bot world transforms not verified")
    print("  Item 4 passed.")


def test_5_root_detection_last(caller, world_sub):
    print("Testing Item 5: Root detection when root link is defined last...")
    reordered_urdf = (
        "<robot name='reorder_bot'>"
        "<link name='child_tip'/>"
        "<joint name='j_rev' type='fixed'><parent link='actual_root'/><child link='child_tip'/></joint>"
        "<link name='actual_root'/>"
        "</robot>"
    )
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": reordered_urdf})
    v = r["values"]["version"]
    st = status_for(caller, v)
    assert st["accepted"] and st["root_link"] == "actual_root"
    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        w_msg = world_sub.next_publish("/xform_world")
        if children(w_msg) == {"actual_root", "child_tip"}:
            break
    else:
        raise AssertionError("Root not discovered correctly when declared last")
    print("  Item 5 passed.")


def test_6_joint_state_replacement(caller, tf_sub, world_sub):
    print("Testing Item 6: Joint-state replacement, ignored names, non-unit axes, timestamps...")
    bot = (
        "<robot name='movable_bot'>"
        "<link name='m_base'/>"
        "<link name='m_link1'/>"
        "<link name='m_link2'/>"
        "<joint name='joint1' type='revolute'><parent link='m_base'/><child link='m_link1'/>"
        "  <origin xyz='0 0 1'/><axis xyz='0 0 3'/><limit lower='-3.14' upper='3.14'/></joint>"
        "<joint name='joint2' type='prismatic'><parent link='m_link1'/><child link='m_link2'/>"
        "  <origin xyz='0 0 0'/><axis xyz='1 0 0'/><limit lower='-10' upper='10'/></joint>"
        "</robot>"
    )
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": bot})
    v = r["values"]["version"]
    status_for(caller, v)

    caller.send({"op": "advertise", "topic": "/joint_states", "type": "sensor_msgs/JointState"})

    # Send positions + ignored name + exact timestamp
    caller.send({"op": "publish", "topic": "/joint_states", "msg": {
        "header": {"stamp": {"sec": 1234, "nanosec": 5678}},
        "name": ["joint1", "joint2", "non_existent_joint"],
        "position": [1.5707963267948966, 2.5, 999.0]
    }})

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        tf_msg = tf_sub.next_publish("/tf")
        transforms = {e["child_frame_id"]: e for e in tf_msg.get("transforms", [])}
        if "m_link2" in transforms and abs(transforms["m_link2"]["transform"]["translation"]["x"] - 2.5) < 1e-4:
            assert transforms["m_link2"]["header"]["stamp"]["sec"] == 1234
            assert transforms["m_link2"]["header"]["stamp"]["nanosec"] == 5678
            # Non-unit axis normalized: axis 0 0 3, angle pi/2 -> 90 deg z-rotation
            rot1 = transforms["m_link1"]["transform"]["rotation"]
            assert abs(abs(rot1["z"]) - 0.70710678) < 1e-4
            assert abs(abs(rot1["w"]) - 0.70710678) < 1e-4
            break
    else:
        raise AssertionError("Joint states not reflected on /tf")

    # Joint state replacement: omit joint2 -> joint2 resets to 0.0!
    caller.send({"op": "publish", "topic": "/joint_states", "msg": {
        "header": {"stamp": {"sec": 1235, "nanosec": 0}},
        "name": ["joint1"],
        "position": [0.0]
    }})

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        tf_msg = tf_sub.next_publish("/tf")
        transforms = {e["child_frame_id"]: e for e in tf_msg.get("transforms", [])}
        if "m_link2" in transforms and abs(transforms["m_link2"]["transform"]["translation"]["x"] - 0.0) < 1e-4:
            assert transforms["m_link2"]["header"]["stamp"]["sec"] == 1235
            break
    else:
        raise AssertionError("Omitted joint was not reset to 0 under replacement semantics")
    print("  Item 6 passed.")


def test_7_global_pose_composition(caller, world_sub):
    print("Testing Item 7: Global-pose composition into every link's world transform...")
    bot = (
        "<robot name='pose_bot'>"
        "<link name='p_root'/><link name='p_tip'/>"
        "<joint name='p_edge' type='fixed'><parent link='p_root'/><child link='p_tip'/><origin xyz='1 0 0'/></joint>"
        "</robot>"
    )
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": bot})
    status_for(caller, r["values"]["version"])

    caller.send({"op": "advertise", "topic": "/global_pose", "type": "geometry_msgs/Pose"})
    caller.send({"op": "publish", "topic": "/global_pose", "msg": {
        "position": {"x": 10, "y": 0, "z": 1},
        "orientation": {"x": 0, "y": 0, "z": 0.7071067811865476, "w": 0.7071067811865476}
    }})

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        posed = world_sub.next_publish("/xform_world")
        matrices = {item["child_frame_id"]: item["matrix"] for item in posed.get("transforms", [])}
        if "p_root" in matrices and abs(matrices["p_root"][12] - 10) < 1e-4:
            assert abs(matrices["p_tip"][12] - 10) < 1e-4
            assert abs(matrices["p_tip"][13] - 1) < 1e-4
            assert abs(matrices["p_tip"][14] - 1) < 1e-4
            break
    else:
        raise AssertionError("Global pose was not reflected in /xform_world")
    print("  Item 7 passed.")


def test_8_publication_frequency_and_late_sub(process, caller, tf_sub, world_sub):
    print("Testing Item 8: Publication frequency, complete messages, late subscribers...")
    started = time.monotonic()
    tf_count = world_count = 0
    while time.monotonic() - started < 0.55:
        tf_sub.next_publish("/tf")
        world_sub.next_publish("/xform_world")
        tf_count += 1
        world_count += 1
    assert tf_count >= 3 and world_count >= 3, f"Rate below 5 Hz: tf={tf_count}, world={world_count}"

    late = connect(process)
    late.subscribe("/xform_world", "xform_world/MatrixTransformArray")
    joined = time.monotonic()
    msg = late.next_publish("/xform_world", 0.3)
    assert "p_root" in children(msg) and "p_tip" in children(msg)
    assert time.monotonic() - joined < 0.3
    late.sock.close()
    print("  Item 8 passed.")


def test_9_reload_replaces_old(caller, tf_sub, world_sub):
    print("Testing Item 9: Reload replaces old links and joints...")
    new_robot = (
        "<robot name='second_bot'>"
        "<link name='brand_new_root'/><link name='brand_new_tip'/>"
        "<joint name='brand_new_joint' type='fixed'><parent link='brand_new_root'/><child link='brand_new_tip'/></joint>"
        "</robot>"
    )
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": new_robot})
    status_for(caller, r["values"]["version"])

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        w_msg = world_sub.next_publish("/xform_world")
        ch = children(w_msg)
        if ch == {"brand_new_root", "brand_new_tip"}:
            break
    else:
        raise AssertionError("Old links not replaced after reload")
    print("  Item 9 passed.")


def test_10_fetch_and_large_urdf(caller, tf_sub, world_sub):
    print("Testing Item 10: Fetch URDF and large URDF (>800 KiB)...")
    # Fetch URDF
    with open("fetch/fetch.urdf", "r", encoding="utf-8") as f:
        fetch_text = f.read()
    r = caller.call("/param_server/set_param", {"name": "robot_description", "value": fetch_text})
    st = status_for(caller, r["values"]["version"])
    assert st["accepted"] and st["root_link"] == "base_link"

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        w_msg = world_sub.next_publish("/xform_world")
        if len(w_msg.get("transforms", [])) == 26:
            assert "base_link" in children(w_msg)
            assert "gripper_link" in children(w_msg)
            break
    else:
        raise AssertionError("Fetch links not received on /xform_world")

    deadline = time.monotonic() + 2
    while time.monotonic() < deadline:
        tf_msg = tf_sub.next_publish("/tf")
        if len(tf_msg.get("transforms", [])) == 25:
            break
    else:
        raise AssertionError("Fetch joints not received on /tf")

    # Large URDF approaching 1 MiB (850 KiB comment)
    padding = "<!-- " + ("A" * (850 * 1024)) + " -->"
    large_urdf = (
        f"<robot name='large_bot'>{padding}"
        "<link name='big_root'/><link name='big_tip'/>"
        "<joint name='big_joint' type='fixed'><parent link='big_root'/><child link='big_tip'/></joint>"
        "</robot>"
    )
    r_large = caller.call("/param_server/set_param", {"name": "robot_description", "value": large_urdf})
    st_large = status_for(caller, r_large["values"]["version"])
    assert st_large["accepted"] and st_large["root_link"] == "big_root"

    # Get param retrieves large text verbatim
    g_large = caller.call("/param_server/get_param", {"name": "robot_description"})
    assert g_large["result"]
    assert len(g_large["values"]["value"]) == len(large_urdf)
    print("  Item 10 passed.")


def test_11_repeated_restarts():
    print("Testing Item 11: Repeated runtime startups, shutdowns, and rejected requests...")
    for cycle in range(3):
        p = subprocess.Popen(["make", "run"], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        try:
            c = connect(p)
            # Unset param returns false
            res = c.call("/param_server/get_param", {"name": "any_param"})
            assert not res["result"]
            # Invalid op
            c.send({"op": "unsupported_op_xyz"})
            # Subsequent call works cleanly
            res2 = c.call("/param_server/set_param", {"name": f"restart_param_{cycle}", "value": cycle})
            assert res2["result"]
            c.sock.close()
        finally:
            try:
                os.killpg(p.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            p.wait(timeout=5)
    print("  Item 11 passed.")


def test_12_run_vs_demo_isolation():
    print("Testing Item 12: make run does not start demo publishers, make demo starts them...")
    # 1. Under `make run`
    p_run = subprocess.Popen(["make", "run"], start_new_session=True,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        c = connect(p_run)
        c.subscribe("/joint_states", "sensor_msgs/JointState")
        c.subscribe("/joint_trajectory", "trajectory_msgs/JointTrajectory")
        c.subscribe("/fsm/status", "fsm/Status")

        c.assert_no_publish("/joint_states", 0.4)
        c.assert_no_publish("/joint_trajectory", 0.4)
        c.assert_no_publish("/fsm/status", 0.4)

        # Demo services fail under make run
        r_reset = c.call("/joint_state_publisher/reset", {})
        assert not r_reset["result"], "reset service should not exist under make run"
        r_fsm = c.call("/fsm/get_logic", {})
        assert not r_fsm["result"], "fsm service should not exist under make run"
        c.sock.close()
    finally:
        try:
            os.killpg(p_run.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        p_run.wait(timeout=5)

    # 2. Under `make demo`
    p_demo = subprocess.Popen(["make", "demo"], start_new_session=True,
                              stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    try:
        c_demo = connect(p_demo)
        c_demo.subscribe("/fsm/status", "fsm/Status")
        status_msg = c_demo.next_publish("/fsm/status", 2.0)
        assert "current_state" in status_msg
        assert "running" in status_msg

        r_reset = c_demo.call("/joint_state_publisher/reset", {})
        assert r_reset["result"], "reset service should exist under make demo"
        c_demo.sock.close()
    finally:
        try:
            os.killpg(p_demo.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        p_demo.wait(timeout=5)
    print("  Item 12 passed.")


def run_full_suite():
    print("=== Starting Project 3 Black-Box TCP Integration Suite ===")

    # Test repeated restarts first (Item 11)
    test_11_repeated_restarts()

    # Test make run vs make demo isolation (Item 12)
    test_12_run_vs_demo_isolation()

    # Start main make run process for Items 1 - 10
    process = subprocess.Popen(["make", "run"], start_new_session=True,
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    clients = []
    try:
        caller = connect(process)
        tf_sub = connect(process)
        world_sub = connect(process)
        clients.extend([caller, tf_sub, world_sub])

        tf_sub.subscribe("/tf", "tf/tfMessage")
        world_sub.subscribe("/xform_world", "xform_world/MatrixTransformArray")

        # Silence before robot load
        tf_sub.assert_no_publish("/tf", 0.25)
        world_sub.assert_no_publish("/xform_world", 0.25)

        test_1_parameter_server(caller)
        test_2_rejection_conditions(caller)
        test_3_preserve_on_failed_load(caller, tf_sub, world_sub)
        test_4_zero_config_fk(caller, tf_sub, world_sub)
        test_5_root_detection_last(caller, world_sub)
        test_6_joint_state_replacement(caller, tf_sub, world_sub)
        test_7_global_pose_composition(caller, world_sub)
        test_8_publication_frequency_and_late_sub(process, caller, tf_sub, world_sub)
        test_9_reload_replaces_old(caller, tf_sub, world_sub)
        test_10_fetch_and_large_urdf(caller, tf_sub, world_sub)

        print("\nAll 12 Black-Box Verification Items Passed Successfully!")
    finally:
        for c in clients:
            c.sock.close()
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        process.wait(timeout=5)


if __name__ == "__main__":
    run_full_suite()
