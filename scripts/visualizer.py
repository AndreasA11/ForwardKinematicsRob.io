#!/usr/bin/env python3
"""
Pendularm Visualizer Bridge & Web Server
----------------------------------------
Provides:
  1. Multi-threaded HTTP static file serving for web/ (UI frontend).
  2. RFC 6455 WebSocket gateway bridging browser connections to TCP 127.0.0.1:9095 (rosbridge).
  3. Automatic lifecycle management for the C++ build/arm_sim runtime.
  4. Mode switching API between 2-link and 3-link configurations.
"""

import os
import sys
import time
import socket
import select
import struct
import base64
import signal
import hashlib
import threading
import subprocess
import webbrowser
import urllib.parse
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True
    allow_reuse_address = True


class VisualizerManager:
    def __init__(self, rosbridge_host="127.0.0.1", rosbridge_port=9095, default_links=2):
        self.rosbridge_host = rosbridge_host
        self.rosbridge_port = rosbridge_port
        self.link_count = default_links
        self.proc = None
        self.lock = threading.Lock()
        self.auto_managed = False

    def is_backend_listening(self) -> bool:
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect((self.rosbridge_host, self.rosbridge_port))
            s.close()
            return True
        except (socket.error, ConnectionRefusedError, OSError):
            return False

    def start_backend(self, link_count=None):
        with self.lock:
            if link_count is not None:
                self.link_count = link_count
            if self.is_backend_listening():
                if not self.proc:
                    # External backend already running
                    return True
                else:
                    self.stop_backend()

            # Ensure binary exists
            root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            bin_path = os.path.join(root_dir, "build", "arm_sim")
            if not os.path.exists(bin_path):
                print("[Visualizer] Compiling build/arm_sim...")
                res = subprocess.run(["make", "build"], cwd=root_dir)
                if res.returncode != 0:
                    print("[Visualizer] Error: Failed to compile build/arm_sim", file=sys.stderr)
                    return False

            env = os.environ.copy()
            env["ARM_SIM_LINKS"] = str(self.link_count)
            env["ARM_SIM_PORT"] = str(self.rosbridge_port)
            print(f"[Visualizer] Launching backend (ARM_SIM_LINKS={self.link_count}, PORT={self.rosbridge_port})...")
            self.proc = subprocess.Popen([bin_path], env=env)
            self.auto_managed = True

            # Wait for port to open
            for _ in range(30):
                time.sleep(0.1)
                if self.is_backend_listening():
                    print(f"[Visualizer] Backend successfully listening on {self.rosbridge_host}:{self.rosbridge_port}")
                    return True
            print("[Visualizer] Warning: Backend did not open port in time", file=sys.stderr)
            return False

    def stop_backend(self):
        with self.lock:
            if self.proc:
                print("[Visualizer] Stopping backend...")
                try:
                    self.proc.terminate()
                    self.proc.wait(timeout=1.5)
                except (subprocess.TimeoutExpired, Exception):
                    try:
                        self.proc.kill()
                        self.proc.wait(timeout=0.5)
                    except Exception:
                        pass
                self.proc = None
                time.sleep(0.2)

    def restart_backend(self, link_count):
        with self.lock:
            self.link_count = link_count
        if self.proc:
            self.stop_backend()
        return self.start_backend(link_count)


manager = VisualizerManager()


def make_ws_text_frame(message: str) -> bytes:
    payload = message.encode("utf-8")
    length = len(payload)
    if length <= 125:
        header = bytes([0x81, length])
    elif length <= 65535:
        header = struct.pack("!BBH", 0x81, 126, length)
    else:
        header = struct.pack("!BBQ", 0x81, 127, length)
    return header + payload


def read_exact(sock, n):
    data = bytearray()
    while len(data) < n:
        chunk = sock.recv(n - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return data


def recv_ws_frame(sock):
    header = read_exact(sock, 2)
    if not header:
        return None, None
    b1, b2 = header[0], header[1]
    opcode = b1 & 0x0F
    is_masked = (b2 & 0x80) != 0
    length = b2 & 0x7F

    if length == 126:
        ext = read_exact(sock, 2)
        if not ext: return None, None
        length = struct.unpack("!H", ext)[0]
    elif length == 127:
        ext = read_exact(sock, 8)
        if not ext: return None, None
        length = struct.unpack("!Q", ext)[0]

    mask = read_exact(sock, 4) if is_masked else None
    if is_masked and not mask:
        return None, None

    data = read_exact(sock, length)
    if data is None:
        return None, None

    if is_masked and mask:
        for i in range(len(data)):
            data[i] ^= mask[i % 4]

    return opcode, data.decode("utf-8", errors="replace")


def handle_websocket_bridge(client_sock):
    """Bridge one WebSocket client to a fresh TCP connection on the rosbridge server."""
    backend_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        backend_sock.connect((manager.rosbridge_host, manager.rosbridge_port))
    except Exception as e:
        print(f"[Visualizer] Failed to connect to rosbridge backend on {manager.rosbridge_port}: {e}")
        try:
            close_frame = bytes([0x88, 0x02, 0x03, 0xEB])
            client_sock.sendall(close_frame)
            client_sock.close()
        except Exception:
            pass
        return

    running = True

    def ws_to_tcp():
        nonlocal running
        try:
            while running:
                opcode, msg = recv_ws_frame(client_sock)
                if opcode is None or opcode == 0x8:  # Close frame or EOF
                    break
                elif opcode == 0x9:  # Ping
                    pong_frame = bytes([0x8A, 0x00])
                    client_sock.sendall(pong_frame)
                elif opcode == 0x1:  # Text
                    # Forward newline-delimited JSON
                    line = msg.strip() + "\n"
                    backend_sock.sendall(line.encode("utf-8"))
        except Exception:
            pass
        finally:
            running = False
            try:
                backend_sock.close()
            except Exception:
                pass
            try:
                client_sock.close()
            except Exception:
                pass

    def tcp_to_ws():
        nonlocal running
        buf = ""
        try:
            while running:
                chunk = backend_sock.recv(4096).decode("utf-8", errors="replace")
                if not chunk:
                    break
                buf += chunk
                while "\n" in buf:
                    line, buf = buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        frame = make_ws_text_frame(line)
                        client_sock.sendall(frame)
        except Exception:
            pass
        finally:
            running = False
            try:
                client_sock.close()
            except Exception:
                pass
            try:
                backend_sock.close()
            except Exception:
                pass

    t1 = threading.Thread(target=ws_to_tcp, daemon=True)
    t2 = threading.Thread(target=tcp_to_ws, daemon=True)
    t1.start()
    t2.start()
    t1.join()
    t2.join()


class VisualizerRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        if "404" in format % args or "500" in format % args:
            super().log_message(format, *args)

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        path = parsed.path

        # WebSocket Upgrade check
        if path == "/ws" and self.headers.get("Upgrade", "").lower() == "websocket":
            key = self.headers.get("Sec-WebSocket-Key")
            if not key:
                self.send_error(400, "Missing Sec-WebSocket-Key")
                return

            accept_key = base64.b64encode(
                hashlib.sha1((key + WS_GUID).encode("utf-8")).digest()
            ).decode("utf-8")

            response = (
                "HTTP/1.1 101 Switching Protocols\r\n"
                "Upgrade: websocket\r\n"
                "Connection: Upgrade\r\n"
                f"Sec-WebSocket-Accept: {accept_key}\r\n"
                "\r\n"
            )
            self.wfile.write(response.encode("utf-8"))
            self.wfile.flush()

            # Delegate to bridge handler
            client_sock = self.request
            handle_websocket_bridge(client_sock)
            return

        # API endpoints
        if path == "/api/status":
            listening = manager.is_backend_listening()
            body = (
                f'{{"backend_listening": {str(listening).lower()}, '
                f'"links": {manager.link_count}, '
                f'"rosbridge_port": {manager.rosbridge_port}}}'
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return

        if path == "/api/backend/restart":
            query = urllib.parse.parse_qs(parsed.query)
            links = int(query.get("links", [manager.link_count])[0])
            success = manager.restart_backend(links)
            body = (
                f'{{"success": {str(success).lower()}, '
                f'"links": {manager.link_count}}}'
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(body)
            return

        # Static files
        if path == "/" or path == "":
            path = "/index.html"

        web_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "web")
        rel_path = path.lstrip("/")
        full_path = os.path.normpath(os.path.join(web_dir, rel_path))

        if not full_path.startswith(web_dir):
            self.send_error(403, "Forbidden")
            return

        if not os.path.exists(full_path) or os.path.isdir(full_path):
            self.send_error(404, "File Not Found")
            return

        mime_types = {
            ".html": "text/html; charset=utf-8",
            ".css": "text/css; charset=utf-8",
            ".js": "application/javascript; charset=utf-8",
            ".json": "application/json; charset=utf-8",
            ".png": "image/png",
            ".jpg": "image/jpeg",
            ".svg": "image/svg+xml",
            ".ico": "image/x-icon",
        }
        _, ext = os.path.splitext(full_path)
        content_type = mime_types.get(ext.lower(), "application/octet-stream")

        try:
            with open(full_path, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(content)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.end_headers()
            self.wfile.write(content)
        except Exception as e:
            self.send_error(500, f"Error reading file: {e}")


def main():
    import argparse
    parser = argparse.ArgumentParser(description="Pendularm Web Visualizer Server")
    parser.add_argument("--port", type=int, default=8080, help="Web server port (default: 8080)")
    parser.add_argument("--rosbridge-port", type=int, default=9095, help="C++ rosbridge port (default: 9095)")
    parser.add_argument("--links", type=int, default=int(os.environ.get("ARM_SIM_LINKS", "2")), choices=[2, 3], help="Initial link count (2 or 3)")
    parser.add_argument("--no-open", action="store_true", help="Do not automatically open web browser")
    args = parser.parse_args()

    manager.rosbridge_port = args.rosbridge_port
    manager.link_count = args.links

    server = ThreadedHTTPServer(("127.0.0.1", args.port), VisualizerRequestHandler)

    def shutdown_handler(signum, frame):
        print("\n[Visualizer] Shutting down...")
        manager.stop_backend()
        try:
            server.shutdown()
        except Exception:
            pass
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown_handler)
    signal.signal(signal.SIGTERM, shutdown_handler)

    if manager.is_backend_listening():
        print(f"[Visualizer] Connected to existing Pendularm backend on port {args.rosbridge_port}.")
    else:
        print(f"[Visualizer] No backend running on port {args.rosbridge_port}. Launching build/arm_sim...")
        if not manager.start_backend(args.links):
            print("[Visualizer] Warning: Proceeding without auto-launched backend.", file=sys.stderr)

    print("=================================================================")
    print(f" PENDULARM WEB VISUALIZER RUNNING")
    print(f" Web UI:      http://127.0.0.1:{args.port}")
    print(f" WebSocket:   ws://127.0.0.1:{args.port}/ws  ->  TCP 127.0.0.1:{args.rosbridge_port}")
    print(f" Mode:        {args.links}-Link Planar Arm")
    print("=================================================================")

    if not args.no_open:
        def open_browser():
            time.sleep(0.5)
            webbrowser.open(f"http://127.0.0.1:{args.port}")
        threading.Thread(target=open_browser, daemon=True).start()

    try:
        server.serve_forever()
    finally:
        manager.stop_backend()
        server.server_close()


if __name__ == "__main__":
    main()
