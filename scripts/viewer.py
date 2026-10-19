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

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


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
        ok, vals = self.call_service("/param_server/get_param", {"name": "robot_description"})
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


# Global TCP client
tcp_client = None


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
                "latest_fsm_status": tcp_client.latest_fsm_status.get("msg") if tcp_client.latest_fsm_status else None
            }
            self._send_json(200, res)
            return

        if path == "/api/robot_description":
            desc = tcp_client.fetch_current_robot_description()
            if not desc:
                # Default to chess_arm.urdf if available, else fetch/fetch.urdf
                chess_path = os.path.join(ROOT_DIR, "urdf-files", "chess_arm.urdf")
                fetch_path = os.path.join(ROOT_DIR, "fetch", "fetch.urdf")
                candidate = chess_path if os.path.exists(chess_path) else fetch_path
                if os.path.exists(candidate):
                    with open(candidate, "r", encoding="utf-8") as f:
                        desc = f.read()
            self._send_json(200, {"description": desc})
            return

        if path == "/api/robot_list":
            robots = []
            chess_path = os.path.join(ROOT_DIR, "urdf-files", "chess_arm.urdf")
            if os.path.exists(chess_path):
                robots.append({"id": "chess_arm", "name": "Chess Manipulator Arm (7-DOF)", "path": "urdf-files/chess_arm.urdf"})
            fetch_path = os.path.join(ROOT_DIR, "fetch", "fetch.urdf")
            if os.path.exists(fetch_path):
                robots.append({"id": "fetch", "name": "Fetch Mobile Manipulator", "path": "fetch/fetch.urdf"})
            urdf_dir = os.path.join(ROOT_DIR, "urdf-files")
            if os.path.exists(urdf_dir):
                for f in sorted(os.listdir(urdf_dir)):
                    if (f.endswith(".urdf") or f.endswith(".xml")) and f != "chess_arm.urdf":
                        robots.append({"id": f, "name": f, "path": os.path.join("urdf-files", f)})
            self._send_json(200, {"robots": robots})
            return

        if path == "/api/fsm_list":
            fsm_dir = os.path.join(ROOT_DIR, "fsm")
            fsm_files = []
            if os.path.exists(fsm_dir):
                # Put chess_pick_and_place first if present
                for f in sorted(os.listdir(fsm_dir)):
                    if f == "chess_pick_and_place.json":
                        fsm_files.insert(0, {"identifier": f[:-5], "filename": f})
                    elif f.endswith(".json"):
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
            # Check web_dir first, then portfolio_dir
            cand_w = os.path.normpath(os.path.join(web_dir, clean_path))
            if cand_w.startswith(web_dir) and os.path.exists(cand_w) and not os.path.isdir(cand_w):
                full_path = cand_w
            else:
                cand_p = os.path.normpath(os.path.join(portfolio_dir, clean_path))
                if cand_p.startswith(portfolio_dir) and os.path.exists(cand_p) and not os.path.isdir(cand_p):
                    full_path = cand_p

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
    global tcp_client
    parser = argparse.ArgumentParser(description="Project 3 3D Robot Viewer Server")
    parser.add_argument("--port", type=int, default=8080, help="Web server port (default: 8080)")
    parser.add_argument("--rosbridge-port", type=int, default=9095, help="C++ TCP rosbridge port (default: 9095)")
    parser.add_argument("--rosbridge-host", type=str, default="127.0.0.1", help="C++ TCP rosbridge host (default: 127.0.0.1)")
    parser.add_argument("--no-open", action="store_true", help="Do not automatically open web browser")
    args = parser.parse_args()

    tcp_client = TcpRosbridgeClient(host=args.rosbridge_host, port=args.rosbridge_port)

    server = ThreadedHTTPServer(("127.0.0.1", args.port), ViewerRequestHandler)

    def shutdown_handler(signum, frame):
        print("\n[Viewer] Shutting down...")
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
        tcp_client.running = False
        server.server_close()


if __name__ == "__main__":
    main()
