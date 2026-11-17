#!/usr/bin/env python3
"""Rosbridge Protocol Client for Trossen RX200 Robot Arm.

Interfaces with a real Trossen RX200 robot arm over a rosbridge connection
(supporting both WebSocket and raw TCP newline-delimited JSON transports).
Designed according to the ROB380 tutorial (https://github.com/tianyiy-tim/rx200-rob380)
and the Project 3 rosbridge specification (spec/ROSBRIDGE_PROTOCOL.md).

Joints of the Trossen RX200:
  - waist: revolute [-pi, pi]
  - shoulder: revolute [-1.88, 1.97]
  - elbow: revolute [-1.88, 1.62]
  - wrist_angle: revolute [-1.75, 2.15]
  - wrist_rotate: revolute [-pi, pi]
  - left_finger: prismatic [0.015, 0.037]
  - right_finger: prismatic [-0.037, -0.015]
"""

import base64
import json
import os
import select
import socket
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple, Any

from bridges.mbot_client import WebSocketClient


class RosbridgeArmClient:
    """Client for rosbridge interfacing with a Trossen RX200 robot arm."""

    RX200_JOINTS = [
        "waist",
        "shoulder",
        "elbow",
        "wrist_angle",
        "wrist_rotate",
        "left_finger",
        "right_finger",
    ]

    # Standard presets (in radians / meters)
    PRESETS = {
        "home": {
            "waist": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "wrist_angle": 0.0,
            "wrist_rotate": 0.0,
            "left_finger": 0.026,
            "right_finger": -0.026,
        },
        "sleep": {
            "waist": 0.0,
            "shoulder": -1.80,
            "elbow": 1.55,
            "wrist_angle": 0.80,
            "wrist_rotate": 0.0,
            "left_finger": 0.015,
            "right_finger": -0.015,
        },
        "upright": {
            "waist": 0.0,
            "shoulder": 0.0,
            "elbow": -1.57,
            "wrist_angle": 0.0,
            "wrist_rotate": 0.0,
            "left_finger": 0.026,
            "right_finger": -0.026,
        },
        "chess_hover_e2": {
            "waist": 0.090,
            "shoulder": 0.250,
            "elbow": 1.250,
            "wrist_angle": 0.850,
            "wrist_rotate": 0.0,
            "left_finger": 0.035,
            "right_finger": -0.035,
        },
        "chess_grasp_e2": {
            "waist": 0.090,
            "shoulder": 0.520,
            "elbow": 1.480,
            "wrist_angle": 0.420,
            "wrist_rotate": 0.0,
            "left_finger": 0.018,
            "right_finger": -0.018,
        },
        "chess_lift_e2": {
            "waist": 0.090,
            "shoulder": 0.250,
            "elbow": 1.250,
            "wrist_angle": 0.850,
            "wrist_rotate": 0.0,
            "left_finger": 0.018,
            "right_finger": -0.018,
        },
        "chess_place_e4": {
            "waist": 0.065,
            "shoulder": 0.650,
            "elbow": 1.300,
            "wrist_angle": 0.480,
            "wrist_rotate": 0.0,
            "left_finger": 0.018,
            "right_finger": -0.018,
        },
    }

    def __init__(self, host: str = "127.0.0.1", port: int = 9090, transport: str = "auto"):
        self.host = host
        self.port = port
        self.transport = transport  # 'auto', 'ws', or 'tcp'
        self.connected = False
        self.running = False
        self._lock = threading.Lock()

        self._ws_client: Optional[WebSocketClient] = None
        self._tcp_sock: Optional[socket.socket] = None
        self._rx_thread: Optional[threading.Thread] = None

        self.latest_joint_states: Dict[str, float] = {}
        self._on_joint_callbacks: List[Callable[[Dict[str, float]], None]] = []

    def connect(self, timeout: float = 3.0) -> bool:
        with self._lock:
            if self.connected:
                return True

            mode = self.transport
            if mode == "auto":
                # Default port 9090 is WebSocket; 9095 is raw TCP; otherwise try WebSocket first, fallback to TCP
                mode = "ws" if self.port in (9090, 8080) else "tcp"

            # Attempt preferred transport first
            transports_to_try = [mode]
            fallback = "tcp" if mode == "ws" else "ws"
            transports_to_try.append(fallback)

            for t in transports_to_try:
                if t == "ws":
                    ws = WebSocketClient(self.host, self.port, timeout=timeout)
                    if ws.connect():
                        self._ws_client = ws
                        self._active_transport = "ws"
                        self.connected = True
                        break
                elif t == "tcp":
                    try:
                        s = socket.create_connection((self.host, self.port), timeout=timeout)
                        s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        self._tcp_sock = s
                        self._active_transport = "tcp"
                        self.connected = True
                        break
                    except Exception:
                        pass

            if not self.connected:
                return False

            self.running = True
            # Subscribe to joint state topics
            self._subscribe_topics()

            # Launch listener thread
            self._rx_thread = threading.Thread(target=self._listen_loop, daemon=True)
            self._rx_thread.start()
            return True

    def disconnect(self):
        self.running = False
        with self._lock:
            if self._ws_client:
                try:
                    self._ws_client.close()
                except Exception:
                    pass
                self._ws_client = None
            if self._tcp_sock:
                try:
                    self._tcp_sock.close()
                except Exception:
                    pass
                self._tcp_sock = None
            self.connected = False

    def is_connected(self) -> bool:
        return self.connected

    def add_joint_callback(self, cb: Callable[[Dict[str, float]], None]):
        self._on_joint_callbacks.append(cb)

    def _send_raw(self, msg_dict: Dict[str, Any]) -> bool:
        try:
            line = json.dumps(msg_dict)
            if self._active_transport == "ws" and self._ws_client:
                self._ws_client.send_text(line)
                return True
            elif self._active_transport == "tcp" and self._tcp_sock:
                data = (line + "\n").encode("utf-8")
                self._tcp_sock.sendall(data)
                return True
        except Exception:
            self.connected = False
        return False

    def _subscribe_topics(self):
        # Subscribe to standard /joint_states and robot-specific /rx200/joint_states
        topics = ["/rx200/joint_states", "/joint_states"]
        for top in topics:
            msg = {
                "op": "subscribe",
                "topic": top,
                "type": "sensor_msgs/JointState",
            }
            self._send_raw(msg)

    def _listen_loop(self):
        buf = b""
        while self.running and self.connected:
            try:
                if self._active_transport == "ws" and self._ws_client:
                    text = self._ws_client.recv_text(timeout=1.0)
                    if text:
                        self._handle_incoming_text(text)
                elif self._active_transport == "tcp" and self._tcp_sock:
                    chunk = self._tcp_sock.recv(4096)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        raw_line, buf = buf.split(b"\n", 1)
                        line = raw_line.strip()
                        if line:
                            self._handle_incoming_text(line.decode("utf-8", errors="replace"))
            except (socket.timeout, TimeoutError):
                continue
            except Exception:
                break
        self.connected = False

    def _handle_incoming_text(self, text: str):
        try:
            msg = json.loads(text)
        except Exception:
            return

        op = msg.get("op")
        if op == "publish":
            topic = msg.get("topic", "")
            if "joint_states" in topic:
                body = msg.get("msg", {})
                names = body.get("name", [])
                positions = body.get("position", [])
                if len(names) == len(positions):
                    state_map = {}
                    for n, p in zip(names, positions):
                        # Normalize joint name (e.g. 'rx200/waist' -> 'waist')
                        short_name = n.split("/")[-1]
                        state_map[short_name] = float(p)
                    self.latest_joint_states.update(state_map)
                    for cb in self._on_joint_callbacks:
                        try:
                            cb(self.latest_joint_states)
                        except Exception:
                            pass

    def send_joint_positions(self, positions_dict: Dict[str, float], duration: float = 2.0) -> bool:
        """Send joint position targets to the robot arm."""
        names = []
        positions = []
        for name, val in positions_dict.items():
            names.append(name)
            positions.append(float(val))

        # Build trajectory_msgs/JointTrajectory message
        traj_msg = {
            "op": "publish",
            "topic": "/rx200/joint_trajectory",
            "msg": {
                "joint_names": names,
                "points": [
                    {
                        "positions": positions,
                        "time_from_start": {"sec": int(duration), "nanosec": int((duration % 1) * 1e9)},
                    }
                ],
            },
        }
        # Also publish to /joint_trajectory and /rx200/arm_controller/joint_trajectory
        res = self._send_raw(traj_msg)
        traj_msg["topic"] = "/rx200/arm_controller/joint_trajectory"
        self._send_raw(traj_msg)
        traj_msg["topic"] = "/joint_trajectory"
        self._send_raw(traj_msg)
        return res

    def go_to_preset(self, preset_name: str, duration: float = 2.0) -> bool:
        if preset_name in self.PRESETS:
            return self.send_joint_positions(self.PRESETS[preset_name], duration=duration)
        return False

    def release_gripper(self) -> bool:
        """Open the parallel gripper fingers."""
        return self.send_joint_positions({"left_finger": 0.035, "right_finger": -0.035}, duration=1.0)

    def grasp_gripper(self) -> bool:
        """Close the parallel gripper fingers to grasp an object."""
        return self.send_joint_positions({"left_finger": 0.018, "right_finger": -0.018}, duration=1.0)

    def execute_chess_pick_and_place(self, duration_per_step: float = 1.4, callback: Optional[Callable[[str], None]] = None) -> bool:
        """Execute the complete chess piece pick-and-place choreography on the RX200 arm in a background thread.
        
        Move: White Pawn on e2 advanced to e4 (1. e4)
        """
        steps = [
            ("Home", {"waist": 0.000, "shoulder": -0.200, "elbow": 0.800, "wrist_angle": 1.000, "wrist_rotate": 0.0, "left_finger": 0.035, "right_finger": -0.035}),
            ("Hover e2", {"waist": 0.090, "shoulder": 0.250, "elbow": 1.250, "wrist_angle": 0.850, "wrist_rotate": 0.0, "left_finger": 0.035, "right_finger": -0.035}),
            ("Descend e2", {"waist": 0.090, "shoulder": 0.520, "elbow": 1.480, "wrist_angle": 0.420, "wrist_rotate": 0.0, "left_finger": 0.035, "right_finger": -0.035}),
            ("Grasp Pawn", {"waist": 0.090, "shoulder": 0.520, "elbow": 1.480, "wrist_angle": 0.420, "wrist_rotate": 0.0, "left_finger": 0.018, "right_finger": -0.018}),
            ("Lift Pawn", {"waist": 0.090, "shoulder": 0.250, "elbow": 1.250, "wrist_angle": 0.850, "wrist_rotate": 0.0, "left_finger": 0.018, "right_finger": -0.018}),
            ("Carry to e4", {"waist": 0.065, "shoulder": 0.480, "elbow": 1.050, "wrist_angle": 0.950, "wrist_rotate": 0.0, "left_finger": 0.018, "right_finger": -0.018}),
            ("Place on e4", {"waist": 0.065, "shoulder": 0.650, "elbow": 1.300, "wrist_angle": 0.480, "wrist_rotate": 0.0, "left_finger": 0.018, "right_finger": -0.018}),
            ("Release Pawn", {"waist": 0.065, "shoulder": 0.650, "elbow": 1.300, "wrist_angle": 0.480, "wrist_rotate": 0.0, "left_finger": 0.035, "right_finger": -0.035}),
            ("Retract Ascent", {"waist": 0.065, "shoulder": 0.400, "elbow": 1.000, "wrist_angle": 0.950, "wrist_rotate": 0.0, "left_finger": 0.035, "right_finger": -0.035}),
            ("Home", {"waist": 0.000, "shoulder": -0.200, "elbow": 0.800, "wrist_angle": 1.000, "wrist_rotate": 0.0, "left_finger": 0.035, "right_finger": -0.035}),
        ]

        def runner():
            for name, pos in steps:
                if not self.connected:
                    break
                if callback:
                    try:
                        callback(name)
                    except Exception:
                        pass
                self.send_joint_positions(pos, duration=duration_per_step)
                time.sleep(duration_per_step + 0.1)

        t = threading.Thread(target=runner, daemon=True)
        t.start()
        return True


if __name__ == "__main__":
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9090
    client = RosbridgeArmClient(host, port)
    print(f"Connecting to ROSBridge robot arm at {host}:{port}...")
    if client.connect():
        print("Connected! Listening for joint states... (Ctrl+C to quit)")
        try:
            while True:
                time.sleep(1.0)
                if client.latest_joint_states:
                    print("Joints:", client.latest_joint_states)
        except KeyboardInterrupt:
            client.disconnect()
            print("Disconnected.")
    else:
        print("Connection failed.")
