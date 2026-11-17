#!/usr/bin/env python3
"""Unit test for MBot Bridge client protocol implementation."""

import base64
import hashlib
import json
import socket
import struct
import sys
import threading
import time
import unittest

from bridges.mbot_client import MbotClient, WebSocketClient


class MockMbotBridgeServer:
    """Mock MBot Bridge server implementing the WebSocket protocol from mbot_websocket_spec.md."""

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
                client_sock, _ = self.server_sock.accept()
                client_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                t = threading.Thread(target=self._handle_client, args=(client_sock,), daemon=True)
                t.start()
            except (socket.timeout, OSError):
                continue

    def _handle_client(self, sock):
        try:
            # Read handshake
            req = b""
            while b"\r\n\r\n" not in req:
                chunk = sock.recv(1024)
                if not chunk:
                    return
                req += chunk

            lines = req.decode("latin-1").splitlines()
            sec_key = None
            for line in lines:
                if line.lower().startswith("sec-websocket-key:"):
                    sec_key = line.split(":", 1)[1].strip()
                    break

            if not sec_key:
                sock.close()
                return

            # Compute accept key
            accept = base64.b64encode(hashlib.sha1((sec_key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode("ascii")).digest()).decode("ascii")
            resp = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept}\r\n\r\n"
            )
            sock.sendall(resp.encode("ascii"))

            # Read frames
            while self.running:
                # Read 2-byte header
                h = self._recv_exact(sock, 2)
                if not h:
                    break
                opcode = h[0] & 0x0F
                masked = bool(h[1] & 0x80)
                plen = h[1] & 0x7F
                if plen == 126:
                    plen = struct.unpack("!H", self._recv_exact(sock, 2))[0]
                elif plen == 127:
                    plen = struct.unpack("!Q", self._recv_exact(sock, 8))[0]

                mask = self._recv_exact(sock, 4) if masked else b""
                data = self._recv_exact(sock, plen) if plen > 0 else b""
                if masked and mask:
                    data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))

                if opcode == 0x8:  # Close
                    break
                elif opcode == 0x1:  # Text
                    text = data.decode("utf-8")
                    msg = json.loads(text)
                    self.received_messages.append(msg)
                    self._respond(sock, msg)
        except Exception:
            pass
        finally:
            try:
                sock.close()
            except Exception:
                pass

    def _recv_exact(self, sock, n):
        buf = bytearray()
        while len(buf) < n:
            chunk = sock.recv(n - len(buf))
            if not chunk:
                break
            buf.extend(chunk)
        return bytes(buf)

    def _respond(self, sock, msg):
        msg_type = msg.get("type")
        channel = msg.get("channel")

        if msg_type == "request":
            if channel == "SLAM_POSE":
                resp = {
                    "type": "response",
                    "channel": "SLAM_POSE",
                    "dtype": "pose2D_t",
                    "data": {"utime": 12345, "x": 1.25, "y": -0.75, "theta": 0.5},
                }
                self._send_text_frame(sock, json.dumps(resp))
            elif channel == "SLAM_MAP":
                resp = {
                    "type": "response",
                    "channel": "SLAM_MAP",
                    "dtype": "occupancy_grid_t",
                    "data": {
                        "utime": 12345,
                        "origin_x": -2.0,
                        "origin_y": -2.0,
                        "meters_per_cell": 0.05,
                        "width": 10,
                        "height": 10,
                        "num_cells": 100,
                        "cells": base64.b64encode(bytes([0] * 100)).decode("ascii"),
                    },
                }
                self._send_text_frame(sock, json.dumps(resp))

    def _send_text_frame(self, sock, text):
        data = text.encode("utf-8")
        header = bytearray([0x81])  # FIN + Text, unmasked from server
        length = len(data)
        if length < 126:
            header.append(length)
        elif length < 65536:
            header.append(126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(127)
            header.extend(struct.pack("!Q", length))
        sock.sendall(header + data)

    def stop(self):
        self.running = False
        try:
            self.server_sock.close()
        except Exception:
            pass
        self.thread.join(timeout=1.0)


class TestMbotClient(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = MockMbotBridgeServer()

    @classmethod
    def tearDownClass(cls):
        cls.server.stop()

    def test_connect_and_read_pose(self):
        client = MbotClient(self.server.host, self.server.port)
        self.assertTrue(client.connect())
        pose = client.read_pose(timeout=2.0)
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose[0], 1.25)
        self.assertAlmostEqual(pose[1], -0.75)
        self.assertAlmostEqual(pose[2], 0.5)
        client.disconnect()

    def test_read_map(self):
        client = MbotClient(self.server.host, self.server.port)
        self.assertTrue(client.connect())
        m = client.read_map(timeout=2.0)
        self.assertIsNotNone(m)
        self.assertEqual(m["width"], 10)
        self.assertEqual(m["height"], 10)
        self.assertEqual(m["meters_per_cell"], 0.05)
        client.disconnect()

    def test_send_path(self):
        client = MbotClient(self.server.host, self.server.port)
        self.assertTrue(client.connect())
        wps = [
            {"x": 0.0, "y": 0.0, "theta": 0.0},
            {"x": 1.0, "y": 0.5, "theta": 0.2},
        ]
        self.assertTrue(client.send_path(wps))
        time.sleep(0.1)
        # Verify server received CONTROLLER_PATH publish
        found = False
        for msg in self.server.received_messages:
            if msg.get("channel") == "CONTROLLER_PATH" and msg.get("type") == "publish":
                found = True
                self.assertEqual(msg["data"]["path_length"], 2)
                break
        self.assertTrue(found)
        client.disconnect()

    def test_stop_command(self):
        client = MbotClient(self.server.host, self.server.port)
        self.assertTrue(client.connect())
        self.assertTrue(client.stop())
        time.sleep(0.1)
        found = False
        for msg in self.server.received_messages:
            if msg.get("channel") == "MBOT_VEL_CMD" and msg.get("type") == "publish":
                found = True
                self.assertEqual(msg["data"]["vx"], 0.0)
                self.assertEqual(msg["data"]["vy"], 0.0)
                self.assertEqual(msg["data"]["wz"], 0.0)
                break
        self.assertTrue(found)
        client.disconnect()


if __name__ == "__main__":
    unittest.main()
