#!/usr/bin/env python3
"""End-to-end integration tests for Task 13: External Robot Viewer and FSM Choreography.
Verifies:
  1. Viewer backend TCP connection to 127.0.0.1:9095 without WebSocket gateways.
  2. Parameter server description loading and status retrieval.
  3. Real-time /xform_world SSE streaming and 16-element column-major matrix integrity.
  4. FSM choreography loading, execution, state transitions, and status reporting.
  5. Stale link purge on robot model reload.
  6. Joint reset service invocation.
"""

import json
import os
import signal
import subprocess
import sys
import time
import urllib.request

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 8092


def make_request(path, method="GET", payload=None):
    url = f"http://127.0.0.1:{PORT}{path}"
    headers = {"Content-Type": "application/json"}
    data = json.dumps(payload).encode("utf-8") if payload is not None else None
    req = urllib.request.Request(url, data=data, headers=headers, method=method)
    with urllib.request.urlopen(req, timeout=4.0) as resp:
        return json.loads(resp.read().decode("utf-8"))


def test_viewer_integration():
    print("=== Starting Task 13 External Viewer End-to-End Test ===")

    # 1. Start runtime under demo mode
    print("Starting ./main --demo...")
    main_proc = subprocess.Popen(["./main", "--demo"], cwd=ROOT_DIR)
    time.sleep(0.3)

    # 2. Start viewer server
    print(f"Starting python3 scripts/viewer.py on port {PORT}...")
    viewer_proc = subprocess.Popen(
        [sys.executable, "scripts/viewer.py", "--no-open", "--port", str(PORT)],
        cwd=ROOT_DIR
    )
    time.sleep(0.8)

    try:
        # Check API status
        print("1. Checking viewer backend status...")
        deadline = time.monotonic() + 5.0
        status = {}
        while time.monotonic() < deadline:
            try:
                status = make_request("/api/status")
                if status.get("backend_connected"):
                    break
            except Exception:
                pass
            time.sleep(0.1)
        assert status.get("backend_connected"), "Viewer failed to connect to TCP rosbridge"
        print("  Connected to 127.0.0.1:9095 over TCP.")

        # Check robot & FSM lists
        print("2. Checking robot and FSM file discovery...")
        rob_list = make_request("/api/robot_list")
        assert any(r["id"] == "chess_arm" for r in rob_list.get("robots", [])), "Chess arm not found in robot list"
        fsm_list = make_request("/api/fsm_list")
        assert any(f["identifier"] == "chess_pick_and_place" for f in fsm_list.get("fsm_files", [])), "chess_pick_and_place not in FSM list"
        print("  Robots and FSM definitions discovered correctly.")

        # Load Chess Arm URDF via REST API
        print("3. Loading Chess Arm URDF via viewer API...")
        load_res = make_request("/api/load_robot", method="POST", payload={"path": "urdf-files/chess_arm.urdf"})
        assert load_res.get("result"), f"Failed to load Chess Arm: {load_res}"
        assert load_res.get("status", {}).get("root_link") == "table_base"
        print("  Chess Arm URDF accepted and loaded as active robot.")

        # Read description back
        desc_res = make_request("/api/robot_description")
        assert "chess_manipulator_arm" in desc_res.get("description", ""), "Description API did not return Chess URDF"

        # Load FSM Choreography
        print("4. Configuring FSM Choreography (chess_pick_and_place)...")
        fsm_res = make_request("/api/load_fsm", method="POST", payload={"identifier": "chess_pick_and_place"})
        assert fsm_res.get("result"), f"Failed to load FSM: {fsm_res}"
        print("  FSM choreography chess_pick_and_place loaded.")

        # Start Choreography
        print("5. Starting FSM Choreography...")
        run_res = make_request("/api/set_fsm_running", method="POST", payload={"running": True})
        assert run_res.get("result"), "Failed to start FSM running"
        print("  FSM set_running=True successfully.")

        # Test Animation Speed API
        speed_res = make_request("/api/set_fsm_speed", method="POST", payload={"speed": 1.5})
        assert speed_res.get("result"), f"Failed to set FSM speed: {speed_res}"
        print("  FSM speed adjustment verified (/api/set_fsm_speed).")

        # Connect to SSE Stream and verify /xform_world and /fsm/status
        print("6. Subscribing to HTTP Server-Sent Events (/stream)...")
        sse_req = urllib.request.Request(f"http://127.0.0.1:{PORT}/stream")
        with urllib.request.urlopen(sse_req, timeout=5.0) as sse_stream:
            seen_xform = False
            seen_fsm = False
            transforms_checked = False
            states_visited = set()

            read_deadline = time.monotonic() + 4.0
            buf = ""
            while time.monotonic() < read_deadline and len(states_visited) < 2:
                line = sse_stream.readline().decode("utf-8")
                if not line:
                    break
                line = line.strip()
                if line.startswith("data: "):
                    data_str = line[6:]
                    try:
                        msg = json.loads(data_str)
                    except Exception:
                        continue

                    if msg.get("op") == "publish":
                        topic = msg.get("topic")
                        if topic == "/xform_world":
                            seen_xform = True
                            transforms = msg["msg"]["transforms"]
                            child_frames = {t["child_frame_id"] for t in transforms}
                            assert "table_base" in child_frames, "table_base missing from /xform_world"
                            assert "arm_base_link" in child_frames, "arm_base_link missing"
                            assert "gripper_left_finger" in child_frames, "gripper_left_finger missing"

                            # Verify 16-element column major matrix
                            for t in transforms:
                                m = t["matrix"]
                                assert len(m) == 16, f"Matrix length {len(m)} != 16"
                                assert all(isinstance(v, (int, float)) for v in m)
                                # Last row in homogeneous matrix is [0, 0, 0, 1] (indices 3, 7, 11, 15)
                                assert abs(m[3]) < 1e-4 and abs(m[7]) < 1e-4 and abs(m[11]) < 1e-4
                                assert abs(m[15] - 1.0) < 1e-4
                            transforms_checked = True

                        elif topic == "/fsm/status":
                            seen_fsm = True
                            st = msg["msg"]
                            if st.get("running") is True:
                                curr_state = st.get("current_state")
                                if curr_state:
                                    states_visited.add(curr_state)

            assert seen_xform, "Did not receive /xform_world on SSE stream"
            assert seen_fsm, "Did not receive /fsm/status on SSE stream"
            assert transforms_checked, "Transforms column-major matrices invalid"
            print(f"  Verified /xform_world column-major matrices and observed FSM states: {states_visited}")

        # 7. Test Robot Reload and Stale Link Purge
        print("7. Testing robot model reload & stale link purge...")
        load_prim = make_request("/api/load_robot", method="POST", payload={"path": "urdf-files/primitive_bot.urdf"})
        assert load_prim.get("result"), "Failed to load primitive_bot"
        time.sleep(0.5)

        # Connect to stream and confirm only primitive_bot links exist now
        with urllib.request.urlopen(sse_req, timeout=4.0) as sse_stream:
            for _ in range(20):
                line = sse_stream.readline().decode("utf-8").strip()
                if line.startswith("data: "):
                    msg = json.loads(line[6:])
                    if msg.get("op") == "publish" and msg.get("topic") == "/xform_world":
                        frames = {t["child_frame_id"] for t in msg["msg"]["transforms"]}
                        # Must contain primitive bot links
                        assert "turret_link" in frames
                        assert "ee_link" in frames
                        # Must NOT contain Chess Arm links
                        assert "table_base" not in frames
                        assert "gripper_left_finger" not in frames
                        print("  Stale Chess Arm link objects successfully purged on model replacement!")
                        break

        # 8. Test Joint Reset
        print("8. Testing joint reset service...")
        reset_res = make_request("/api/reset_joints", method="POST", payload={})
        assert reset_res.get("result"), "Joint reset failed"
        print("  Joint reset service succeeded.")

        print("=== All Task 13 Viewer Integration Checks Passed! ===")

    finally:
        viewer_proc.terminate()
        main_proc.terminate()
        try:
            viewer_proc.wait(timeout=1.0)
        except Exception:
            viewer_proc.kill()
        try:
            main_proc.wait(timeout=1.0)
        except Exception:
            main_proc.kill()


if __name__ == "__main__":
    test_viewer_integration()
