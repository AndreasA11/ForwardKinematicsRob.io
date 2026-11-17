#!/usr/bin/env python3
"""Project 3 External 3D Robot Viewer Server.
Connects to the C++ runtime over raw TCP on 127.0.0.1:9095 using newline-delimited JSON
(the project's supported protocol) without adding a WebSocket gateway.

Provides:
  1. Multi-threaded HTTP server for web/ (Three.js 3D UI) and portfolio/.
  2. Server-Sent Events (SSE) streaming /xform_world and /fsm/status to browser clients.
  3. REST API bridging browser requests to C++ rosbridge services over TCP.
"""

import argparse
import itertools
import json
import math
import os
import select
import signal
import socket
import sys
import threading
import time
import urllib.parse
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn
from typing import Any, Dict, List, Optional, Tuple

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if ROOT_DIR not in sys.path:
    sys.path.insert(0, ROOT_DIR)

from bridges.mbot_client import MbotClient
from bridges.rosbridge_arm_client import RosbridgeArmClient


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def handle_error(self, request, client_address):
        ex_type, _, _ = sys.exc_info()
        if ex_type in (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
            return
        super().handle_error(request, client_address)


class TcpRosbridgeClient:
    """Manages an external TCP connection to the Project 3 runtime on 127.0.0.1:9095."""

    def __init__(self, host="127.0.0.1", port=9095):
        self.host = host
        self.port = port
        self.sock = None
        self.lock = threading.Lock()
        self.call_lock = threading.Lock()
        self.connected = False
        self.running = True
        self.call_ids = itertools.count(1)
        self.pending_calls = {}  # call_id -> (threading.Event, response_holder)
        self.subscribers = set()  # set of client queues for SSE
        self.subscribers_lock = threading.Lock()
        self.latest_robot_description = ""
        self.latest_description_status = {}
        self.latest_xform_world = None
        self.latest_fsm_status = None
        self.latest_joint_states = None

        self.worker_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.worker_thread.start()

    def add_sse_client(self, q):
        with self.subscribers_lock:
            self.subscribers.add(q)
            # Send immediate snapshots to new client
            if self.latest_xform_world:
                q.put(self.latest_xform_world)
            if self.latest_fsm_status:
                q.put(self.latest_fsm_status)

    def remove_sse_client(self, q):
        with self.subscribers_lock:
            self.subscribers.discard(q)

    def _broadcast(self, msg_dict):
        with self.subscribers_lock:
            for q in list(self.subscribers):
                try:
                    q.put_nowait(msg_dict)
                except Exception:
                    pass

    def _connect(self):
        try:
            s = socket.create_connection((self.host, self.port), timeout=1.5)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.sock = s
            self.connected = True
            print(f"[Viewer Backend] Connected to TCP rosbridge at {self.host}:{self.port}")
            self._send_raw({"op": "subscribe", "topic": "/xform_world", "type": "xform_world/MatrixTransformArray"})
            self._send_raw({"op": "subscribe", "topic": "/fsm/status", "type": "fsm/Status"})
            self._send_raw({"op": "subscribe", "topic": "/joint_states", "type": "sensor_msgs/JointState"})
            self._send_raw({"op": "subscribe", "topic": "/joint_trajectory", "type": "trajectory_msgs/JointTrajectory"})

            # Auto-prime default robot description if parameter server is unpopulated
            def _init_default_urdf():
                time.sleep(0.15)
                ok, vals = self.call_service("/param_server/get_param", {"name": "robot_description"}, timeout=1.5)
                if not ok or not vals or not vals.get("value"):
                    cands = [
                        os.path.join(ROOT_DIR, "urdf-files", "rx200_chess.urdf"),
                        os.path.join(ROOT_DIR, "robots", "rx200", "rx200_chess.urdf"),
                        os.path.join(ROOT_DIR, "urdf-files", "chess_arm.urdf"),
                    ]
                    for cand in cands:
                        if os.path.exists(cand):
                            try:
                                with open(cand, "r", encoding="utf-8") as f:
                                    content = f.read()
                                self.call_service("/param_server/set_param", {"name": "robot_description", "value": content}, timeout=2.0)
                                self.latest_robot_description = content
                                print(f"[Viewer Backend] Primed initial robot description from {os.path.basename(cand)}")
                                break
                            except Exception:
                                pass
            threading.Thread(target=_init_default_urdf, daemon=True).start()
            return True
        except Exception as e:
            self.connected = False
            self.sock = None
            return False

    def _send_raw(self, msg_dict):
        try:
            line = (json.dumps(msg_dict) + "\n").encode("utf-8")
            with self.lock:
                if self.sock:
                    self.sock.sendall(line)
                    return True
        except Exception:
            self.connected = False
            return False
        return False

    def call_service(self, service, args=None, timeout=3.0):
        if args is None:
            args = {}
        if not self.connected or not self.sock:
            return False, {"error": "backend not connected"}

        call_id = f"viewer-{next(self.call_ids)}"
        event = threading.Event()
        holder = [None]
        with self.call_lock:
            self.pending_calls[call_id] = (event, holder)

        msg = {"op": "call_service", "service": service, "id": call_id, "args": args}
        if not self._send_raw(msg):
            with self.call_lock:
                self.pending_calls.pop(call_id, None)
            return False, {"error": "send failed"}

        if event.wait(timeout):
            resp = holder[0]
            if resp:
                return resp.get("result", False), resp.get("values", {})
            return False, {"error": "empty response"}
        else:
            with self.call_lock:
                self.pending_calls.pop(call_id, None)
            return False, {"error": f"timeout calling {service}"}

    def _connection_loop(self):
        buf = b""
        while self.running:
            if not self.connected:
                if not self._connect():
                    time.sleep(0.5)
                    continue

            # Read stream
            try:
                chunk = self.sock.recv(65536)
                if not chunk:
                    print("[Viewer Backend] Connection closed by remote host. Reconnecting...")
                    self.connected = False
                    if self.sock:
                        try:
                            self.sock.close()
                        except Exception:
                            pass
                    self.sock = None
                    continue

                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    raw_line = raw_line.strip()
                    if not raw_line:
                        continue
                    try:
                        msg = json.loads(raw_line.decode("utf-8", errors="replace"))
                        self._handle_incoming_message(msg)
                    except Exception as e:
                        pass
            except (socket.timeout, TimeoutError):
                continue
            except Exception as e:
                self.connected = False
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                self.sock = None
                time.sleep(0.5)

    def _handle_incoming_message(self, msg):
        op = msg.get("op")
        if op == "publish":
            topic = msg.get("topic")
            if topic == "/xform_world":
                self.latest_xform_world = msg
                self._broadcast(msg)
            elif topic == "/fsm/status":
                self.latest_fsm_status = msg
                self._broadcast(msg)
            elif topic == "/joint_states":
                self.latest_joint_states = msg
                self._broadcast(msg)
            elif topic == "/joint_trajectory":
                self._broadcast(msg)
                # Forward trajectory setpoints to physical RX200 arm if connected
                if external_bridge and external_bridge.arm_client and external_bridge.status == "connected":
                    body = msg.get("msg", {})
                    names = body.get("joint_names", [])
                    points = body.get("points", [])
                    if points and names:
                        pt = points[0]
                        positions = pt.get("positions", [])
                        duration = 1.4
                        tf = pt.get("time_from_start", {})
                        if isinstance(tf, dict):
                            dur = tf.get("sec", 0) + tf.get("nanosec", 0) * 1e-9
                            if dur > 0.1:
                                duration = dur
                        arm_pos = {}
                        for n, p in zip(names, positions):
                            if n in RosbridgeArmClient.RX200_JOINTS:
                                arm_pos[n] = float(p)
                        if arm_pos:
                            external_bridge.arm_client.send_joint_positions(arm_pos, duration=duration)
        elif op == "service_response":
            call_id = msg.get("id")
            if call_id:
                with self.call_lock:
                    pending = self.pending_calls.pop(call_id, None)
                if pending:
                    event, holder = pending
                    holder[0] = msg
                    event.set()

    def fetch_current_robot_description(self):
        if self.latest_robot_description:
            return self.latest_robot_description
        ok, vals = self.call_service("/param_server/get_param", {"name": "robot_description"}, timeout=1.5)
        if ok and vals and vals.get("value"):
            self.latest_robot_description = str(vals.get("value"))
            return self.latest_robot_description
        return ""

    def fetch_description_status(self):
        ok, vals = self.call_service("/param_server/get_param", {"name": "/robot_state_publisher/description_status"})
        if ok and vals and vals.get("value"):
            self.latest_description_status = vals.get("value")
            return self.latest_description_status
        return {}


# Global TCP client and External Robot Bridge
tcp_client = None
external_bridge = None


class ExternalRobotBridge:
    """Manages an active external robot connection (MBot Bridge or ROSBridge).

    Enforces the constraint: only one external robot client is active at a time.
    """

    def __init__(self, local_client: Optional[TcpRosbridgeClient] = None):
        self.local_client = local_client
        self.mode = "none"  # "none", "mbot", "rosbridge"
        self.host = "127.0.0.1"
        self.port = 5005
        self.status = "disconnected"  # "disconnected", "connecting", "connected", "error"
        self.last_error = ""
        self.lock = threading.Lock()

        self.mbot_client: Optional[MbotClient] = None
        self.arm_client: Optional[RosbridgeArmClient] = None

        self.latest_mbot_pose = None
        self.latest_arm_joints: Dict[str, float] = {}

    def get_status(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "mode": self.mode,
                "host": self.host,
                "port": self.port,
                "connected": self.status == "connected",
                "status": self.status,
                "last_error": self.last_error,
                "mbot_pose": self.latest_mbot_pose,
                "arm_joints": self.latest_arm_joints,
            }

    def disconnect(self):
        with self.lock:
            if self.mbot_client:
                try:
                    self.mbot_client.disconnect()
                except Exception:
                    pass
                self.mbot_client = None
            if self.arm_client:
                try:
                    self.arm_client.disconnect()
                except Exception:
                    pass
                self.arm_client = None
            self.mode = "none"
            self.status = "disconnected"
            self.last_error = ""
            print("[External Robot Bridge] Disconnected.")

    def connect(self, mode: str, host: str, port: int) -> Tuple[bool, str]:
        # Disconnect any existing external connection first (only one active at a time)
        self.disconnect()

        with self.lock:
            self.mode = mode
            self.host = str(host).strip()
            self.port = int(port)
            self.status = "connecting"
            self.last_error = ""

        print(f"[External Robot Bridge] Connecting to {mode} at {self.host}:{self.port}...")

        if mode == "mbot":
            client = MbotClient(self.host, self.port)
            if not client.connect():
                err = f"Could not connect to MBot Bridge at {self.host}:{self.port}"
                with self.lock:
                    self.status = "error"
                    self.last_error = err
                print(f"[External Robot Bridge] {err}")
                return False, err

            def on_mbot_pose(x, y, theta):
                self.latest_mbot_pose = {"x": x, "y": y, "theta": theta}
                # 1. Forward to local C++ runtime /global_pose
                z_half = theta / 2.0
                pose_msg = {
                    "position": {"x": x, "y": y, "z": 0.0},
                    "orientation": {"x": 0.0, "y": 0.0, "z": math.sin(z_half), "w": math.cos(z_half)},
                }
                if self.local_client and self.local_client.connected:
                    self.local_client._send_raw({
                        "op": "publish",
                        "topic": "/global_pose",
                        "msg": pose_msg,
                    })
                # 2. Broadcast via SSE to browser
                if self.local_client:
                    self.local_client._broadcast({
                        "topic": "/mbot/pose",
                        "data": self.latest_mbot_pose,
                    })

            client.add_pose_callback(on_mbot_pose)
            client.start_pose_stream(rate_hz=10.0)

            with self.lock:
                self.mbot_client = client
                self.status = "connected"
            print(f"[External Robot Bridge] Successfully connected to MBot Bridge at {self.host}:{self.port}")
            return True, "Connected to MBot Bridge"

        elif mode == "rosbridge":
            client = RosbridgeArmClient(self.host, self.port)
            if not client.connect():
                err = f"Could not connect to ROSBridge arm at {self.host}:{self.port}"
                with self.lock:
                    self.status = "error"
                    self.last_error = err
                print(f"[External Robot Bridge] {err}")
                return False, err

            def on_joints(joint_dict):
                self.latest_arm_joints = joint_dict
                # Forward to local C++ runtime /joint_states
                names = list(joint_dict.keys())
                positions = list(joint_dict.values())
                js_msg = {
                    "name": names,
                    "position": positions,
                }
                if self.local_client and self.local_client.connected:
                    self.local_client._send_raw({
                        "op": "publish",
                        "topic": "/joint_states",
                        "msg": js_msg,
                    })
                if self.local_client:
                    self.local_client._broadcast({
                        "topic": "/joint_states",
                        "msg": js_msg,
                    })

            client.add_joint_callback(on_joints)

            with self.lock:
                self.arm_client = client
                self.status = "connected"
            print(f"[External Robot Bridge] Successfully connected to ROSBridge at {self.host}:{self.port}")
            return True, "Connected to ROSBridge robot arm"

        else:
            with self.lock:
                self.mode = "none"
                self.status = "disconnected"
            return False, f"Unknown mode: {mode}"

    def mbot_navigate(self, waypoints: List[Dict[str, float]]) -> Tuple[bool, str]:
        with self.lock:
            if not self.mbot_client or self.status != "connected":
                return False, "MBot Bridge is not connected"
            ok = self.mbot_client.send_path(waypoints)
            return ok, "Path sent to MBot" if ok else "Failed to send path"

    def mbot_stop(self) -> Tuple[bool, str]:
        with self.lock:
            if not self.mbot_client or self.status != "connected":
                return False, "MBot Bridge is not connected"
            ok = self.mbot_client.stop()
            return ok, "Stop command sent to MBot" if ok else "Failed to stop MBot"

    def mbot_get_map(self) -> Tuple[bool, Any]:
        with self.lock:
            if not self.mbot_client or self.status != "connected":
                return False, "MBot Bridge is not connected"
            m = self.mbot_client.read_map()
            return (True, m) if m is not None else (False, "Failed to read map")

    def arm_command(self, action: str, data: Any = None) -> Tuple[bool, str]:
        with self.lock:
            if action in ("chess_pick_and_place", "chess_move"):
                # 1. Trigger autonomous chess pick and place sequence directly on physical arm if connected
                if self.arm_client and self.status == "connected":
                    self.arm_client.execute_chess_pick_and_place(duration_per_step=1.4)
                # 2. Also start the local C++ FSM so the 3D visualizer animates in sync
                if self.local_client and self.local_client.connected:
                    ident = "rx200_chess_pick_and_place"
                    cand = os.path.join(ROOT_DIR, "fsm", f"{ident}.json")
                    if not os.path.exists(cand):
                        ident = "rx200_chess_arm_only"
                    ok1, res1 = self.local_client.call_service("/fsm/load_from_file", {"identifier": ident})
                    if ok1:
                        self.local_client.call_service("/fsm/update_logic", res1)
                        self.local_client.call_service("/fsm/set_running", {"data": True})
                        return True, f"Started {ident} on RX200 and visualizer"
                if self.arm_client and self.status == "connected":
                    return True, "Executing Chess Pick & Place on physical RX200"
                return False, "Could not start chess FSM"

            if not self.arm_client or self.status != "connected":
                return False, "ROSBridge arm is not connected"
            if action in ("home", "sleep", "upright"):
                ok = self.arm_client.go_to_preset(action)
                return ok, f"Preset {action} sent" if ok else "Failed to send preset"
            elif action == "release":
                ok = self.arm_client.release_gripper()
                return ok, "Gripper released" if ok else "Failed to release gripper"
            elif action == "grasp":
                ok = self.arm_client.grasp_gripper()
                return ok, "Gripper grasped" if ok else "Failed to grasp gripper"
            elif action == "positions" and isinstance(data, dict):
                ok = self.arm_client.send_joint_positions(data)
                return ok, "Joint positions sent" if ok else "Failed to send positions"
            return False, f"Unknown arm action: {action}"


class ViewerRequestHandler(BaseHTTPRequestHandler):
    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError):
            pass

    def log_message(self, format, *args):
        # Mute routine SSE poll logs
        if "200" in format % args and "stream" in self.path:
            return
        if "404" in format % args or "500" in format % args:
            super().log_message(format, *args)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        # 1. SSE Stream: Real-time event stream directly to browser (Pure HTTP/1.1 SSE, NO WebSockets)
        if path in ("/stream", "/events"):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache, no-transform")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            import queue
            client_queue = queue.Queue(maxsize=100)
            tcp_client.add_sse_client(client_queue)

            try:
                # Send an initial connect event
                init_evt = json.dumps({"op": "connected", "backend_connected": tcp_client.connected})
                self.wfile.write(f"data: {init_evt}\n\n".encode("utf-8"))
                self.wfile.flush()

                while True:
                    try:
                        msg = client_queue.get(timeout=20.0)
                        data = f"data: {json.dumps(msg)}\n\n".encode("utf-8")
                        self.wfile.write(data)
                        self.wfile.flush()
                    except queue.Empty:
                        # Keep-alive heartbeat comment
                        self.wfile.write(b": heartbeat\n\n")
                        self.wfile.flush()
            except (ConnectionResetError, BrokenPipeError):
                pass
            finally:
                tcp_client.remove_sse_client(client_queue)
            return

        # 2. REST API: Backend & Robot status
        if path == "/api/status":
            desc_status = tcp_client.fetch_description_status()
            res = {
                "backend_connected": tcp_client.connected,
                "host": tcp_client.host,
                "port": tcp_client.port,
                "description_status": desc_status,
                "has_robot": bool(desc_status.get("accepted", False)),
                "loaded_version": desc_status.get("loaded_version", 0),
                "root_link": desc_status.get("root_link", ""),
                "latest_fsm_status": tcp_client.latest_fsm_status.get("msg") if tcp_client.latest_fsm_status else None,
                "external_robot": external_bridge.get_status() if external_bridge else None,
            }
            self._send_json(200, res)
            return

        if path == "/api/external_robot/status":
            st = external_bridge.get_status() if external_bridge else {"mode": "none", "connected": False}
            self._send_json(200, st)
            return

        if path == "/api/robot_description":
            desc = tcp_client.fetch_current_robot_description()
            if not desc:
                # Default to rx200_chess.urdf if available, else chess_arm.urdf, else fetch
                rx200_chess = os.path.join(ROOT_DIR, "urdf-files", "rx200_chess.urdf")
                chess_path = os.path.join(ROOT_DIR, "urdf-files", "chess_arm.urdf")
                fetch_path = os.path.join(ROOT_DIR, "fetch", "fetch.urdf")
                candidate = rx200_chess if os.path.exists(rx200_chess) else (chess_path if os.path.exists(chess_path) else fetch_path)
                if os.path.exists(candidate):
                    with open(candidate, "r", encoding="utf-8") as f:
                        desc = f.read()
                        tcp_client.latest_robot_description = desc
            self._send_json(200, {"description": desc})
            return

        if path == "/api/robot_list":
            robots = []
            rx200_chess_path = os.path.join(ROOT_DIR, "urdf-files", "rx200_chess.urdf")
            if os.path.exists(rx200_chess_path):
                robots.append({"id": "rx200_chess", "name": "Trossen RX200 (Chess Manipulator Arm)", "path": "urdf-files/rx200_chess.urdf"})
            rx200_path = os.path.join(ROOT_DIR, "urdf-files", "rx200.urdf")
            if os.path.exists(rx200_path):
                robots.append({"id": "rx200", "name": "Trossen RX200 (Standalone Arm)", "path": "urdf-files/rx200.urdf"})
            chess_path = os.path.join(ROOT_DIR, "urdf-files", "chess_arm.urdf")
            if os.path.exists(chess_path):
                robots.append({"id": "chess_arm", "name": "Chess Manipulator Arm (7-DOF)", "path": "urdf-files/chess_arm.urdf"})
            mbot_path = os.path.join(ROOT_DIR, "robots", "mbot", "mbot.urdf")
            if os.path.exists(mbot_path):
                robots.append({"id": "mbot_omni", "name": "MBot Omni (Mobile Robot)", "path": "robots/mbot/mbot.urdf"})
            fetch_path = os.path.join(ROOT_DIR, "fetch", "fetch.urdf")
            if os.path.exists(fetch_path):
                robots.append({"id": "fetch", "name": "Fetch Mobile Manipulator", "path": "fetch/fetch.urdf"})
            urdf_dir = os.path.join(ROOT_DIR, "urdf-files")
            if os.path.exists(urdf_dir):
                for f in sorted(os.listdir(urdf_dir)):
                    if (f.endswith(".urdf") or f.endswith(".xml")) and f not in ("chess_arm.urdf", "rx200.urdf", "rx200_chess.urdf", "mbot.urdf"):
                        robots.append({"id": f, "name": f, "path": os.path.join("urdf-files", f)})
            self._send_json(200, {"robots": robots})
            return

        if path == "/api/fsm_list":
            fsm_dir = os.path.join(ROOT_DIR, "fsm")
            fsm_files = []
            if os.path.exists(fsm_dir):
                priority = [
                    "rx200_chess_pick_and_place.json",
                    "rx200_chess_arm_only.json",
                    "chess_pick_and_place.json"
                ]
                for p in priority:
                    if os.path.exists(os.path.join(fsm_dir, p)):
                        fsm_files.append({"identifier": p[:-5], "filename": p})
                for f in sorted(os.listdir(fsm_dir)):
                    if f.endswith(".json") and f not in priority:
                        fsm_files.append({"identifier": f[:-5], "filename": f})
            self._send_json(200, {"fsm_files": fsm_files})
            return

        # 3. Static Files
        web_dir = os.path.join(ROOT_DIR, "web")
        portfolio_dir = os.path.join(ROOT_DIR, "portfolio")

        clean_path = path.split("?")[0].lstrip("/")
        full_path = None

        if clean_path == "" or clean_path == "index.html":
            full_path = os.path.join(web_dir, "index.html")
        elif clean_path == "portfolio" or clean_path == "portfolio/":
            full_path = os.path.join(portfolio_dir, "index.html")
        elif clean_path.startswith("portfolio/"):
            sub = clean_path[len("portfolio/"):]
            if not sub or sub == "index.html":
                full_path = os.path.join(portfolio_dir, "index.html")
            else:
                cand_p = os.path.normpath(os.path.join(portfolio_dir, sub))
                if cand_p.startswith(portfolio_dir) and os.path.exists(cand_p) and not os.path.isdir(cand_p):
                    full_path = cand_p
                else:
                    # Fallback to web_dir if web assets were requested with /portfolio/ prefix
                    cand_w = os.path.normpath(os.path.join(web_dir, sub))
                    if cand_w.startswith(web_dir) and os.path.exists(cand_w) and not os.path.isdir(cand_w):
                        full_path = cand_w
        elif clean_path.startswith("web/"):
            sub = clean_path[len("web/"):]
            cand_w = os.path.normpath(os.path.join(web_dir, sub))
            if cand_w.startswith(web_dir) and os.path.exists(cand_w) and not os.path.isdir(cand_w):
                full_path = cand_w
        else:
            # Check web_dir first, then portfolio_dir, then root_dir (for robots, meshes, bridges)
            cand_w = os.path.normpath(os.path.join(web_dir, clean_path))
            if cand_w.startswith(web_dir) and os.path.exists(cand_w) and not os.path.isdir(cand_w):
                full_path = cand_w
            else:
                cand_p = os.path.normpath(os.path.join(portfolio_dir, clean_path))
                if cand_p.startswith(portfolio_dir) and os.path.exists(cand_p) and not os.path.isdir(cand_p):
                    full_path = cand_p
                else:
                    cand_r = os.path.normpath(os.path.join(ROOT_DIR, clean_path))
                    if cand_r.startswith(ROOT_DIR) and os.path.exists(cand_r) and not os.path.isdir(cand_r):
                        full_path = cand_r

        if not full_path or not os.path.exists(full_path) or os.path.isdir(full_path):
            self.send_error(404, f"File Not Found: {path}")
            return

        mime_types = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".json": "application/json; charset=utf-8",
            ".png": "image/png",
            ".jpg": "image/jpeg",
            ".svg": "image/svg+xml",
            ".webm": "video/webm",
            ".mp4": "video/mp4",
            ".glb": "model/gltf-binary",
            ".gltf": "model/gltf+json",
            ".stl": "model/stl",
            ".urdf": "application/xml; charset=utf-8",
            ".xml": "application/xml; charset=utf-8",
        }
        _, ext = os.path.splitext(full_path)
        content_type = mime_types.get(ext.lower(), "application/octet-stream")

        try:
            with open(full_path, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache, must-revalidate")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(content)
        except (ConnectionResetError, BrokenPipeError):
            pass
        except Exception as e:
            try:
                self.send_error(500, f"Error reading file: {e}")
            except Exception:
                pass

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        content_length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(content_length).decode("utf-8") if content_length > 0 else "{}"
        try:
            payload = json.loads(body)
        except Exception:
            payload = {}

        if path == "/api/external_robot/connect":
            mode = payload.get("mode", "mbot")
            host = payload.get("host", "127.0.0.1")
            port = int(payload.get("port", 5005 if mode == "mbot" else 9090))
            ok, msg = external_bridge.connect(mode, host, port)
            self._send_json(200, {"result": ok, "message": msg, "status": external_bridge.get_status()})
            return

        if path == "/api/external_robot/disconnect":
            external_bridge.disconnect()
            self._send_json(200, {"result": True, "status": external_bridge.get_status()})
            return

        if path == "/api/external_robot/mbot/navigate":
            waypoints = payload.get("waypoints")
            if not waypoints and "goal" in payload:
                goal = payload.get("goal")
                waypoints = [goal]
            if not waypoints:
                self._send_json(400, {"result": False, "error": "No waypoints provided"})
                return
            ok, msg = external_bridge.mbot_navigate(waypoints)
            self._send_json(200, {"result": ok, "message": msg})
            return

        if path == "/api/external_robot/mbot/stop":
            ok, msg = external_bridge.mbot_stop()
            self._send_json(200, {"result": ok, "message": msg})
            return

        if path == "/api/external_robot/mbot/get_map":
            ok, res = external_bridge.mbot_get_map()
            if ok:
                self._send_json(200, {"result": True, "map": res})
            else:
                self._send_json(500, {"result": False, "error": str(res)})
            return

        if path == "/api/external_robot/arm/command":
            action = payload.get("action", "")
            data = payload.get("data")
            ok, msg = external_bridge.arm_command(action, data)
            self._send_json(200, {"result": ok, "message": msg})
            return

        if path == "/api/call_service":
            service = payload.get("service")
            args = payload.get("args", {})
            timeout = float(payload.get("timeout", 4.0))
            ok, values = tcp_client.call_service(service, args, timeout=timeout)
            self._send_json(200, {"result": ok, "values": values})
            return

        if path == "/api/load_robot":
            # Load robot from path or content
            urdf_text = payload.get("urdf_text")
            rel_path = payload.get("path")
            if not urdf_text and rel_path:
                abs_p = os.path.join(ROOT_DIR, rel_path)
                if os.path.exists(abs_p):
                    with open(abs_p, "r", encoding="utf-8") as f:
                        urdf_text = f.read()
            if not urdf_text:
                self._send_json(400, {"result": False, "error": "No URDF content provided"})
                return

            ok, res = tcp_client.call_service("/param_server/set_param", {"name": "robot_description", "value": urdf_text})
            if not ok:
                self._send_json(500, {"result": False, "error": "set_param failed"})
                return
            version = res.get("version", 0)

            # Wait for description status
            deadline = time.monotonic() + 3.0
            st = {}
            while time.monotonic() < deadline:
                st = tcp_client.fetch_description_status()
                if st.get("version", -1) >= version:
                    break
                time.sleep(0.05)

            tcp_client.latest_robot_description = urdf_text
            # Broadcast description update to all connected visualizer clients
            tcp_client._broadcast({"op": "publish", "topic": "/robot_description", "msg": {"description": urdf_text}})
            self._send_json(200, {"result": st.get("accepted", False), "status": st, "version": version})
            return

        if path == "/api/load_fsm":
            identifier = payload.get("identifier")
            if not identifier:
                self._send_json(400, {"result": False, "error": "Missing identifier"})
                return
            # Call /fsm/load_from_file
            ok, res = tcp_client.call_service("/fsm/load_from_file", {"identifier": identifier})
            if not ok:
                self._send_json(400, {"result": False, "error": "load_from_file failed", "values": res})
                return
            # Activate via /fsm/update_logic
            ok2, res2 = tcp_client.call_service("/fsm/update_logic", res)
            self._send_json(200, {"result": ok2, "values": res2})
            return

        if path == "/api/set_fsm_speed":
            speed = float(payload.get("speed", 1.0))
            if speed <= 0.05 or speed > 10.0:
                self._send_json(400, {"result": False, "error": "Invalid speed value"})
                return

            ok, logic = tcp_client.call_service("/fsm/get_logic", {})
            if not ok or not logic or not logic.get("states"):
                self._send_json(400, {"result": False, "error": "No active logic loaded"})
                return

            # Keep base dwell times if not stored, then rescale
            states = logic.get("states", [])
            for st in states:
                if "_base_dwell" not in st:
                    st["_base_dwell"] = float(st.get("dwell_time", 1.2))
                base = float(st.get("_base_dwell", 1.2))
                st["dwell_time"] = round(base / speed, 3)

            ok2, res2 = tcp_client.call_service("/fsm/update_logic", logic)
            self._send_json(200, {"result": ok2, "speed": speed, "values": res2})
            return

        if path == "/api/set_fsm_running":
            running = bool(payload.get("running", payload.get("data", True)))
            ok, res = tcp_client.call_service("/fsm/set_running", {"data": running})
            self._send_json(200, {"result": ok, "values": res})
            return

        if path == "/api/fire_transition":
            transition = payload.get("transition", "")
            ok, res = tcp_client.call_service("/fsm/fire_transition", {"transition": transition})
            self._send_json(200, {"result": ok, "values": res})
            return

        if path == "/api/reset_joints":
            ok, res = tcp_client.call_service("/joint_state_publisher/reset", {})
            self._send_json(200, {"result": ok, "values": res})
            return

        if path == "/api/save_recording":
            # Save raw recorded video blob from browser
            video_bytes = self.rfile.read(content_length)
            portfolio_dir = os.path.join(ROOT_DIR, "portfolio")
            os.makedirs(portfolio_dir, exist_ok=True)
            webm_path = os.path.join(portfolio_dir, "fetch_choreography.webm")
            mp4_path = os.path.join(portfolio_dir, "fetch_choreography.mp4")
            poster_path = os.path.join(portfolio_dir, "poster.png")

            with open(webm_path, "wb") as f:
                f.write(video_bytes)
            print(f"[Viewer] Saved {len(video_bytes)} bytes to {webm_path}")

            # Transcode with ffmpeg if available
            import subprocess
            try:
                subprocess.run(
                    ["ffmpeg", "-y", "-i", webm_path, "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4_path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
                )
                subprocess.run(
                    ["ffmpeg", "-y", "-ss", "00:00:01", "-i", mp4_path, "-vframes", "1", poster_path],
                    stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False
                )
                print("[Viewer] Transcoded fetch_choreography.mp4 and created poster.png successfully")
            except Exception as e:
                print(f"[Viewer] ffmpeg notice: {e}")

            self._send_json(200, {"result": True})
            return

        if path == "/api/save_frame":
            import base64
            frame_idx = int(payload.get("index", 0))
            data_url = payload.get("data", "")
            if "," in data_url:
                b64_data = data_url.split(",", 1)[1]
                frame_bytes = base64.b64decode(b64_data)
                frames_dir = os.path.join(ROOT_DIR, "portfolio", "frames")
                os.makedirs(frames_dir, exist_ok=True)
                frame_path = os.path.join(frames_dir, f"frame_{frame_idx:04d}.png")
                with open(frame_path, "wb") as f:
                    f.write(frame_bytes)
            self._send_json(200, {"result": True})
            return

        if path == "/api/compile_frames":
            import subprocess
            portfolio_dir = os.path.join(ROOT_DIR, "portfolio")
            frames_pattern = os.path.join(portfolio_dir, "frames", "frame_%04d.png")
            mp4_path = os.path.join(portfolio_dir, "fetch_choreography.mp4")
            webm_path = os.path.join(portfolio_dir, "fetch_choreography.webm")
            poster_path = os.path.join(portfolio_dir, "poster.png")

            try:
                # Compile MP4
                subprocess.run(
                    ["ffmpeg", "-y", "-framerate", "20", "-i", frames_pattern,
                     "-c:v", "libx264", "-pix_fmt", "yuv420p", mp4_path],
                    check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                # Compile WebM
                subprocess.run(
                    ["ffmpeg", "-y", "-i", mp4_path,
                     "-c:v", "libvpx-vp9", "-b:v", "1M", webm_path],
                    check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
                )
                # Save poster
                f0 = os.path.join(portfolio_dir, "frames", "frame_0025.png")
                if os.path.exists(f0):
                    import shutil
                    shutil.copyfile(f0, poster_path)
                print(f"[Viewer] Compiled video deliverable successfully: {mp4_path} ({os.path.getsize(mp4_path)} bytes)")
            except Exception as e:
                print(f"[Viewer] ffmpeg compile error: {e}")

            self._send_json(200, {"result": True})
            return

        self.send_error(404, "Endpoint not found")

    def _send_json(self, status, obj):
        body = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)


def main():
    global tcp_client, external_bridge
    parser = argparse.ArgumentParser(description="Project 3 3D Robot Viewer Server")
    parser.add_argument("--port", type=int, default=8080, help="Web server port (default: 8080)")
    parser.add_argument("--rosbridge-port", type=int, default=9095, help="C++ TCP rosbridge port (default: 9095)")
    parser.add_argument("--rosbridge-host", type=str, default="127.0.0.1", help="C++ TCP rosbridge host (default: 127.0.0.1)")
    parser.add_argument("--mbot-host", type=str, default="", help="MBot Bridge host to connect on startup")
    parser.add_argument("--mbot-port", type=int, default=5005, help="MBot Bridge port (default: 5005)")
    parser.add_argument("--external-rosbridge-host", type=str, default="", help="External ROSBridge arm host to connect on startup")
    parser.add_argument("--external-rosbridge-port", type=int, default=9090, help="External ROSBridge arm port (default: 9090)")
    parser.add_argument("--no-open", action="store_true", help="Do not automatically open web browser")
    args = parser.parse_args()

    tcp_client = TcpRosbridgeClient(host=args.rosbridge_host, port=args.rosbridge_port)
    external_bridge = ExternalRobotBridge(local_client=tcp_client)

    # Optional auto-connect from command line
    if args.mbot_host:
        def auto_connect_mbot():
            time.sleep(1.0)
            external_bridge.connect("mbot", args.mbot_host, args.mbot_port)
        threading.Thread(target=auto_connect_mbot, daemon=True).start()
    elif args.external_rosbridge_host:
        def auto_connect_arm():
            time.sleep(1.0)
            external_bridge.connect("rosbridge", args.external_rosbridge_host, args.external_rosbridge_port)
        threading.Thread(target=auto_connect_arm, daemon=True).start()

    try:
        server = ThreadedHTTPServer(("127.0.0.1", args.port), ViewerRequestHandler)
    except OSError as e:
        if e.errno == 48 or "Address already in use" in str(e):
            print(f"\n[Viewer Error] Port {args.port} is already in use by another process.")
            print(f"[Viewer Hint] Free the port using 'lsof -ti :{args.port} | xargs kill -9' or pass '--port <ANOTHER_PORT>'.\n")
            sys.exit(1)
        raise

    def shutdown_handler(signum, frame):
        print("\n[Viewer] Shutting down...")
        if external_bridge:
            external_bridge.disconnect()
        tcp_client.running = False
        try:
            server.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    print("=================================================================")
    print(" PROJECT 3 3D ROBOT VIEWER SERVER")
    print(f" Web UI:      http://127.0.0.1:{args.port}")
    print(f" Portfolio:   http://127.0.0.1:{args.port}/portfolio/index.html")
    print(f" TCP Backend: {args.rosbridge_host}:{args.rosbridge_port} (raw TCP/JSON)")
    print(f" Protocol:    HTTP/1.1 + Server-Sent Events (NO WebSocket gateway)")
    if args.mbot_host:
        print(f" MBot Bridge: Auto-connecting to ws://{args.mbot_host}:{args.mbot_port}")
    if args.external_rosbridge_host:
        print(f" Ext ROSBridge: Auto-connecting to {args.external_rosbridge_host}:{args.external_rosbridge_port}")
    print("=================================================================")

    if not args.no_open:
        def open_browser():
            time.sleep(0.5)
            import webbrowser
            webbrowser.open(f"http://127.0.0.1:{args.port}")
        threading.Thread(target=open_browser, daemon=True).start()

    try:
        server.serve_forever()
    finally:
        if external_bridge:
            external_bridge.disconnect()
        tcp_client.running = False
        server.server_close()


if __name__ == "__main__":
    main()
