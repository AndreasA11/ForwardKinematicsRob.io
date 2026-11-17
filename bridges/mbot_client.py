#!/usr/bin/env python3
"""MBot Bridge WebSocket Protocol Client.

Implements the client side of the MBot Bridge protocol defined in
bridges/mbot_websocket_spec.md using only Python standard library (no external
pip dependencies).

Handles:
  1. RFC 6455 WebSocket framing over raw TCP.
  2. Reading current SLAM_POSE (x, y, theta).
  3. Reading current SLAM_MAP (occupancy grid).
  4. Publishing paths to CONTROLLER_PATH.
  5. Publishing emergency stop velocities to MBOT_VEL_CMD.
  6. Background pose streaming callback.
"""

import base64
import hashlib
import json
import os
import select
import socket
import struct
import threading
import time
from typing import Callable, Dict, List, Optional, Tuple, Any


class WebSocketClient:
    """Minimal, self-contained RFC 6455 WebSocket client (zero dependencies)."""

    def __init__(self, host: str, port: int, timeout: float = 5.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self._lock = threading.Lock()

    def connect(self) -> bool:
        with self._lock:
            if self.connected and self.sock:
                return True
            try:
                s = socket.create_connection((self.host, self.port), timeout=self.timeout)
                s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                # Generate random 16-byte key
                sec_key = base64.b64encode(os.urandom(16)).decode("ascii")
                headers = (
                    f"GET / HTTP/1.1\r\n"
                    f"Host: {self.host}:{self.port}\r\n"
                    f"Upgrade: websocket\r\n"
                    f"Connection: Upgrade\r\n"
                    f"Sec-WebSocket-Key: {sec_key}\r\n"
                    f"Sec-WebSocket-Version: 13\r\n\r\n"
                )
                s.sendall(headers.encode("ascii"))

                # Read handshake response
                resp = b""
                while b"\r\n\r\n" not in resp:
                    chunk = s.recv(4096)
                    if not chunk:
                        raise ConnectionError("Server closed connection during WebSocket handshake")
                    resp += chunk

                header_part = resp.split(b"\r\n\r\n")[0].decode("latin-1", errors="replace")
                if "101" not in header_part.splitlines()[0]:
                    raise ConnectionError(f"WebSocket upgrade rejected: {header_part.splitlines()[0]}")

                self.sock = s
                self.connected = True
                return True
            except Exception as e:
                self.connected = False
                if self.sock:
                    try:
                        self.sock.close()
                    except Exception:
                        pass
                    self.sock = None
                return False

    def close(self):
        with self._lock:
            if self.sock:
                try:
                    # Send close frame
                    self._send_frame(opcode=0x8, payload=b"")
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            self.connected = False

    def _send_frame(self, opcode: int, payload: bytes):
        if not self.sock:
            raise ConnectionError("Socket not connected")

        # Client-to-server frames MUST be masked (RFC 6455 5.1)
        length = len(payload)
        header = bytearray()
        header.append(0x80 | (opcode & 0x0F))  # FIN + opcode

        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack("!Q", length))

        mask_key = os.urandom(4)
        header.extend(mask_key)

        masked_payload = bytearray(length)
        for i in range(length):
            masked_payload[i] = payload[i] ^ mask_key[i % 4]

        self.sock.sendall(header + masked_payload)

    def send_text(self, text: str):
        with self._lock:
            self._send_frame(opcode=0x1, payload=text.encode("utf-8"))

    def recv_text(self, timeout: Optional[float] = None) -> Optional[str]:
        if not self.sock or not self.connected:
            return None

        orig_timeout = self.sock.gettimeout()
        if timeout is not None:
            self.sock.settimeout(timeout)

        try:
            while True:
                # Read 2-byte frame header
                header = self._recv_exact(2)
                if not header:
                    self.close()
                    return None

                fin = bool(header[0] & 0x80)
                opcode = header[0] & 0x0F
                masked = bool(header[1] & 0x80)
                payload_len = header[1] & 0x7F

                if payload_len == 126:
                    ext = self._recv_exact(2)
                    if not ext:
                        return None
                    payload_len = struct.unpack("!H", ext)[0]
                elif payload_len == 127:
                    ext = self._recv_exact(8)
                    if not ext:
                        return None
                    payload_len = struct.unpack("!Q", ext)[0]

                mask_key = b""
                if masked:
                    mask_key = self._recv_exact(4)
                    if not mask_key:
                        return None

                payload = self._recv_exact(payload_len) if payload_len > 0 else b""
                if len(payload) < payload_len:
                    return None

                if masked and mask_key:
                    unmasked = bytearray(payload_len)
                    for i in range(payload_len):
                        unmasked[i] = payload[i] ^ mask_key[i % 4]
                    payload = bytes(unmasked)

                if opcode == 0x8:  # Close
                    self.close()
                    return None
                elif opcode == 0x9:  # Ping -> reply Pong
                    with self._lock:
                        if self.sock:
                            self._send_frame(opcode=0xA, payload=payload)
                    continue
                elif opcode == 0xA:  # Pong
                    continue
                elif opcode == 0x1:  # Text frame
                    return payload.decode("utf-8", errors="replace")
                else:
                    # Ignore other opcodes
                    continue
        except (socket.timeout, TimeoutError):
            return None
        except Exception:
            self.close()
            return None
        finally:
            if self.sock:
                try:
                    self.sock.settimeout(orig_timeout)
                except Exception:
                    pass

    def _recv_exact(self, num_bytes: int) -> bytes:
        buf = bytearray()
        while len(buf) < num_bytes:
            chunk = self.sock.recv(num_bytes - len(buf))
            if not chunk:
                break
            buf.extend(chunk)
        return bytes(buf)


class MbotClient:
    """High-level client for the MBot Bridge WebSocket protocol."""

    def __init__(self, host: str = "127.0.0.1", port: int = 5005):
        self.host = host
        self.port = port
        self.ws = WebSocketClient(host, port)
        self.latest_pose: Optional[Dict[str, float]] = None
        self.latest_map: Optional[Dict[str, Any]] = None
        self.running = False
        self.stream_thread: Optional[threading.Thread] = None
        self._on_pose_callbacks: List[Callable[[float, float, float], None]] = []

    def connect(self) -> bool:
        return self.ws.connect()

    def disconnect(self):
        self.stop_pose_stream()
        self.ws.close()

    def is_connected(self) -> bool:
        return self.ws.connected

    def add_pose_callback(self, cb: Callable[[float, float, float], None]):
        self._on_pose_callbacks.append(cb)

    def read_pose(self, timeout: float = 2.0) -> Optional[Tuple[float, float, float]]:
        """Fetch latest pose (x, y, theta) from SLAM_POSE."""
        if not self.ws.connected:
            if not self.connect():
                return None

        req = {
            "type": "request",
            "channel": "SLAM_POSE",
            "dtype": "pose2D_t",
            "as_bytes": False,
        }
        try:
            self.ws.send_text(json.dumps(req))
            resp_str = self.ws.recv_text(timeout=timeout)
            if not resp_str:
                return None
            resp = json.loads(resp_str)
            if resp.get("type") == "response" and resp.get("channel") == "SLAM_POSE":
                data = resp.get("data", {})
                x = float(data.get("x", 0.0))
                y = float(data.get("y", 0.0))
                theta = float(data.get("theta", 0.0))
                self.latest_pose = {"x": x, "y": y, "theta": theta}
                for cb in self._on_pose_callbacks:
                    try:
                        cb(x, y, theta)
                    except Exception:
                        pass
                return (x, y, theta)
            elif resp.get("type") == "error":
                return None
        except Exception:
            pass
        return None

    def read_map(self, timeout: float = 3.0) -> Optional[Dict[str, Any]]:
        """Fetch latest occupancy grid map from SLAM_MAP."""
        if not self.ws.connected:
            if not self.connect():
                return None

        req = {
            "type": "request",
            "channel": "SLAM_MAP",
            "dtype": "occupancy_grid_t",
            "as_bytes": False,
        }
        try:
            self.ws.send_text(json.dumps(req))
            resp_str = self.ws.recv_text(timeout=timeout)
            if not resp_str:
                return None
            resp = json.loads(resp_str)
            if resp.get("type") == "response" and resp.get("channel") == "SLAM_MAP":
                data = resp.get("data", {})
                self.latest_map = data
                return data
        except Exception:
            pass
        return None

    def send_path(self, waypoints: List[Dict[str, float]]) -> bool:
        """Publish a path to CONTROLLER_PATH.
        
        waypoints: list of dicts with 'x', 'y', and optional 'theta'.
        """
        if not self.ws.connected:
            if not self.connect():
                return False

        formatted_path = []
        for wp in waypoints:
            formatted_path.append({
                "x": float(wp.get("x", 0.0)),
                "y": float(wp.get("y", 0.0)),
                "theta": float(wp.get("theta", 0.0)),
            })

        msg = {
            "type": "publish",
            "channel": "CONTROLLER_PATH",
            "dtype": "path2D_t",
            "data": {
                "path_length": len(formatted_path),
                "path": formatted_path,
            },
        }
        try:
            self.ws.send_text(json.dumps(msg))
            return True
        except Exception:
            return False

    def stop(self) -> bool:
        """Publish zero velocity to MBOT_VEL_CMD to halt the robot."""
        if not self.ws.connected:
            if not self.connect():
                return False

        msg = {
            "type": "publish",
            "channel": "MBOT_VEL_CMD",
            "dtype": "twist2D_t",
            "data": {"vx": 0.0, "vy": 0.0, "wz": 0.0},
        }
        try:
            self.ws.send_text(json.dumps(msg))
            return True
        except Exception:
            return False

    def start_pose_stream(self, rate_hz: float = 10.0):
        """Start background thread polling SLAM_POSE at given frequency."""
        if self.running:
            return
        self.running = True
        interval = 1.0 / max(rate_hz, 1.0)

        def _loop():
            while self.running:
                try:
                    self.read_pose(timeout=interval)
                except Exception:
                    pass
                time.sleep(interval)

        self.stream_thread = threading.Thread(target=_loop, daemon=True)
        self.stream_thread.start()

    def stop_pose_stream(self):
        self.running = False
        if self.stream_thread and self.stream_thread.is_alive():
            self.stream_thread.join(timeout=1.0)
        self.stream_thread = None


if __name__ == "__main__":
    import sys
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 5005
    client = MbotClient(host, port)
    print(f"Connecting to MBot Bridge at {host}:{port}...")
    if client.connect():
        print("Connected!")
        pose = client.read_pose()
        print("Current Pose:", pose)
        client.disconnect()
    else:
        print("Failed to connect.")
