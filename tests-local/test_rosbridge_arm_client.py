#!/usr/bin/env python3
"""Unit tests for RosbridgeArmClient (supporting TCP and WebSocket transports)."""

import json
import socket
import sys
import threading
import time
import unittest

from bridges.rosbridge_arm_client import RosbridgeArmClient


class MockTcpRosbridgeServer:
    def __init__(self, host="127.0.0.1", port=0):
        self.server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_sock.bind((host, port))
        self.host, self.port = self.server_sock.getsockname()
        self.server_sock.listen(5)
        self.running = True
        self.received_messages = []
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def _run(self):
        while self.running:
            try:
                self.server_sock.settimeout(0.5)
                sock, _ = self.server_sock.accept()
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                t = threading.Thread(target=self._handle_client, args=(sock,), daemon=True)
                t.start()
            except (socket.timeout, OSError):
                continue

    def _handle_client(self, sock):
        buf = b""
        try:
            # Send sample joint state message to client
            js_msg = {
                "op": "publish",
                "topic": "/rx200/joint_states",
                "msg": {
                    "name": ["rx200/waist", "rx200/shoulder", "rx200/elbow"],
                    "position": [0.25, -0.5, 0.75],
                },
            }
            sock.sendall((json.dumps(js_msg) + "\n").encode("utf-8"))

            while self.running:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                buf += chunk
                while b"\n" in buf:
                    raw_line, buf = buf.split(b"\n", 1)
                    line = raw_line.strip()
                    if line:
                        msg = json.loads(line.decode("utf-8"))
                        self.received_messages.append(msg)
        except Exception:
            pass
        finally:
            try:
                sock.close()
            except Exception:
                pass

    def stop(self):
        self.running = False
        try:
            self.server_sock.close()
        except Exception:
            pass
        self.thread.join(timeout=1.0)


class TestRosbridgeArmClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = MockTcpRosbridgeServer()

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()

    def test_tcp_connect_and_receive_joint_states(self):
        client = RosbridgeArmClient(self.server.host, self.server.port, transport="tcp")
        self.assertTrue(client.connect())
        # Wait for joint state receipt
        deadline = time.time() + 2.0
        while time.time() < deadline and "waist" not in client.latest_joint_states:
            time.sleep(0.05)

        self.assertIn("waist", client.latest_joint_states)
        self.assertAlmostEqual(client.latest_joint_states["waist"], 0.25)
        self.assertAlmostEqual(client.latest_joint_states["shoulder"], -0.5)
        self.assertAlmostEqual(client.latest_joint_states["elbow"], 0.75)

        # Test command sending
        self.assertTrue(client.go_to_preset("home"))
        time.sleep(0.1)

        found = False
        for msg in self.server.received_messages:
            if msg.get("topic") in ("/rx200/joint_trajectory", "/joint_trajectory"):
                found = True
                break
        self.assertTrue(found)

        # Test chess pick-and-place execution
        steps_recorded = []
        client.execute_chess_pick_and_place(duration_per_step=0.05, callback=lambda s: steps_recorded.append(s))
        time.sleep(0.8)
        self.assertGreater(len(steps_recorded), 3)
        self.assertIn("Grasp Pawn", steps_recorded)

        client.disconnect()


if __name__ == "__main__":
    unittest.main()
