#!/usr/bin/env python3
"""Project 3 Interactive Portfolio & Chess Arm Demonstration Launcher.

Automates the complete workflow:
1. Launches `./main --demo` (C++ runtime with joint_state_publisher and finite_state_machine).
2. Launches `scripts/viewer.py` on port 8092.
3. Loads `urdf-files/chess_arm.urdf` as the active robot description.
4. Loads and activates the `chess_pick_and_place` FSM choreography.
5. Opens the Course Portfolio page (and 3D Robot Viewer) in the user's default browser.
6. Keeps the demo running and handles clean shutdown on Ctrl+C.
"""

import json
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.request
import webbrowser

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
VIEWER_PORT = 8092
ROSBRIDGE_PORT = 9095


def wait_for_tcp(port, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                return True
        except (ConnectionRefusedError, OSError):
            time.sleep(0.05)
    return False


def wait_for_http(url, timeout=5.0):
    start = time.time()
    while time.time() - start < timeout:
        try:
            with urllib.request.urlopen(url, timeout=0.5) as res:
                if res.status == 200:
                    return True
        except Exception:
            time.sleep(0.05)
    return False


def main():
    print("=" * 70)
    print(" PROJECT 3: CHESS ROBOT ARM & PORTFOLIO DEMONSTRATION LAUNCHER")
    print("=" * 70)

    # 1. Ensure runtime executable exists
    main_bin = os.path.join(ROOT_DIR, "main")
    if not os.path.exists(main_bin):
        print("Compiling runtime main...")
        subprocess.run(["make", "build"], cwd=ROOT_DIR, check=True)

    # 2. Launch runtime under demo mode
    print("1. Launching C++ runtime (./main --demo)...")
    main_proc = subprocess.Popen([main_bin, "--demo"], cwd=ROOT_DIR)

    # 3. Launch 3D viewer server
    print(f"2. Launching 3D viewer bridge (scripts/viewer.py --port {VIEWER_PORT})...")
    viewer_proc = subprocess.Popen(
        [sys.executable, "scripts/viewer.py", "--no-open", "--port", str(VIEWER_PORT)],
        cwd=ROOT_DIR
    )

    def cleanup():
        print("\nStopping demo processes...")
        for p in (viewer_proc, main_proc):
            try:
                p.terminate()
                p.wait(timeout=2.0)
            except Exception:
                try:
                    p.kill()
                except Exception:
                    pass
        print("Demo shutdown complete.")

    def sig_handler(sig, frame):
        cleanup()
        sys.exit(0)

    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    try:
        # Wait for services
        if not wait_for_tcp(ROSBRIDGE_PORT, timeout=4.0):
            print(f"Error: Could not connect to C++ rosbridge at 127.0.0.1:{ROSBRIDGE_PORT}")
            cleanup()
            sys.exit(1)

        status_url = f"http://127.0.0.1:{VIEWER_PORT}/api/status"
        if not wait_for_http(status_url, timeout=4.0):
            print(f"Error: Could not connect to viewer server at 127.0.0.1:{VIEWER_PORT}")
            cleanup()
            sys.exit(1)

        print("  Connected to TCP rosbridge & HTTP viewer successfully.")

        # 4. Load Chess Manipulator Arm URDF
        print("3. Loading Chess Manipulator Arm description (urdf-files/chess_arm.urdf)...")
        chess_urdf_path = "urdf-files/chess_arm.urdf"
        load_robot_req = urllib.request.Request(
            f"http://127.0.0.1:{VIEWER_PORT}/api/load_robot",
            data=json.dumps({"path": chess_urdf_path}).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(load_robot_req) as res:
            res_data = json.loads(res.read().decode("utf-8"))
            if not res_data.get("result"):
                print(f"Warning: Failed to load robot description: {res_data}")
            else:
                print("  Robot description loaded and accepted.")

        time.sleep(0.4)

        # 5. Load and start Chess FSM choreography
        print("4. Configuring Chess Pick-and-Place Choreography (fsm/chess_pick_and_place.json)...")
        load_fsm_req = urllib.request.Request(
            f"http://127.0.0.1:{VIEWER_PORT}/api/load_fsm",
            data=json.dumps({"identifier": "chess_pick_and_place"}).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(load_fsm_req) as res:
            res_data = json.loads(res.read().decode("utf-8"))
            if not res_data.get("result"):
                print(f"Warning: Failed to load FSM: {res_data}")
            else:
                print("  FSM loaded and activated.")

        time.sleep(0.2)

        print("5. Starting FSM Choreography execution...")
        start_fsm_req = urllib.request.Request(
            f"http://127.0.0.1:{VIEWER_PORT}/api/set_fsm_running",
            data=json.dumps({"running": True}).encode("utf-8"),
            headers={"Content-Type": "application/json"}
        )
        with urllib.request.urlopen(start_fsm_req) as res:
            pass

        portfolio_url = f"http://127.0.0.1:{VIEWER_PORT}/portfolio/index.html"
        viewer_url = f"http://127.0.0.1:{VIEWER_PORT}/index.html"

        print("=" * 70)
        print(" CHESS ROBOT DEMONSTRATION RUNNING")
        print(f" * Portfolio Showcase: {portfolio_url}")
        print(f" * 3D Robot Viewer:    {viewer_url}")
        print(" * FSM Sequence:       1. e4 King's Pawn Pick-and-Place")
        print("=" * 70)
        print("Opening 3D Robot Viewer in default browser...")
        try:
            webbrowser.open(viewer_url)
        except Exception:
            pass

        print("\nPress Ctrl+C to stop the demonstration...\n")
        while True:
            time.sleep(1.0)

    except KeyboardInterrupt:
        cleanup()


if __name__ == "__main__":
    main()
