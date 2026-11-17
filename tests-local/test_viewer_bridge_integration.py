#!/usr/bin/env python3
"""Integration test for scripts/viewer.py external robot endpoints."""

import importlib
import json
import socket
import sys
import threading
import time
import unittest
import urllib.request

from scripts.viewer import TcpRosbridgeClient, ExternalRobotBridge, ThreadedHTTPServer, ViewerRequestHandler
import scripts.viewer as viewer_mod

test_mbot_mod = importlib.import_module("tests-local.test_mbot_client")
MockMbotBridgeServer = test_mbot_mod.MockMbotBridgeServer

test_arm_mod = importlib.import_module("tests-local.test_rosbridge_arm_client")
MockTcpRosbridgeServer = test_arm_mod.MockTcpRosbridgeServer


class TestViewerBridgeIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.mock_mbot = MockMbotBridgeServer()
        cls.mock_arm = MockTcpRosbridgeServer()

        # Start viewer HTTP server on ephemeral port
        cls.http_server = ThreadedHTTPServer(("127.0.0.1", 0), ViewerRequestHandler)
        cls.port = cls.http_server.server_port
        cls.http_thread = threading.Thread(target=cls.http_server.serve_forever, daemon=True)
        cls.http_thread.start()

        # Initialize viewer_mod globals
        viewer_mod.tcp_client = TcpRosbridgeClient("127.0.0.1", 9095)
        viewer_mod.external_bridge = ExternalRobotBridge(viewer_mod.tcp_client)

    @classmethod
    def tearDownClass(cls):
        viewer_mod.external_bridge.disconnect()
        viewer_mod.tcp_client.running = False
        cls.http_server.shutdown()
        cls.mock_mbot.stop()
        cls.mock_arm.stop()

    def _get(self, path):
        url = f"http://127.0.0.1:{self.port}{path}"
        with urllib.request.urlopen(url, timeout=2.0) as res:
            return json.loads(res.read().decode("utf-8"))

    def _post(self, path, payload):
        url = f"http://127.0.0.1:{self.port}{path}"
        data = json.dumps(payload).encode("utf-8")
        req = urllib.request.Request(url, data=data, headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=2.0) as res:
            return json.loads(res.read().decode("utf-8"))

    def test_robot_list_contains_mbot_and_rx200(self):
        res = self._get("/api/robot_list")
        self.assertIn("robots", res)
        ids = [r["id"] for r in res["robots"]]
        self.assertIn("mbot_omni", ids)
        self.assertIn("rx200", ids)

    def test_external_robot_status_and_connection_switching(self):
        # 1. Initial status
        st = self._get("/api/external_robot/status")
        self.assertFalse(st["connected"])

        # 2. Connect to MBot
        conn_res = self._post("/api/external_robot/connect", {
            "mode": "mbot",
            "host": self.mock_mbot.host,
            "port": self.mock_mbot.port,
        })
        self.assertTrue(conn_res["result"])
        st2 = self._get("/api/external_robot/status")
        self.assertTrue(st2["connected"])
        self.assertEqual(st2["mode"], "mbot")

        # 3. MBot actions: navigate & stop
        nav_res = self._post("/api/external_robot/mbot/navigate", {
            "waypoints": [{"x": 1.0, "y": 0.5, "theta": 0.0}]
        })
        self.assertTrue(nav_res["result"])

        stop_res = self._post("/api/external_robot/mbot/stop", {})
        self.assertTrue(stop_res["result"])

        # 4. Switch to ROSBridge arm (verifies only ONE connection active at a time!)
        conn_res2 = self._post("/api/external_robot/connect", {
            "mode": "rosbridge",
            "host": self.mock_arm.host,
            "port": self.mock_arm.port,
        })
        self.assertTrue(conn_res2["result"])
        st3 = self._get("/api/external_robot/status")
        self.assertTrue(st3["connected"])
        self.assertEqual(st3["mode"], "rosbridge")

        # 5. Arm command
        arm_res = self._post("/api/external_robot/arm/command", {"action": "home"})
        self.assertTrue(arm_res["result"])

        # 6. Disconnect
        disc_res = self._post("/api/external_robot/disconnect", {})
        self.assertTrue(disc_res["result"])
        st4 = self._get("/api/external_robot/status")
        self.assertFalse(st4["connected"])


if __name__ == "__main__":
    unittest.main()
