#!/usr/bin/env python3
"""Generates the official Project 3 choreography demonstration video for the course portfolio.

Connects to the C++ runtime over TCP on 127.0.0.1:9095, loads the Fetch robot URDF,
executes the FSM choreography (fetch_dance), captures high-definition 3D world-state
transforms from /xform_world, renders depth-shaded perspective frames, and compiles
them via ffmpeg into:
  - portfolio/fetch_choreography.mp4
  - portfolio/fetch_choreography.webm
  - portfolio/poster.png
"""

import itertools
import json
import math
import os
import shutil
import socket
import subprocess
import sys
import threading
import time

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORTFOLIO_DIR = os.path.join(ROOT_DIR, "portfolio")
FRAMES_DIR = os.path.join(PORTFOLIO_DIR, "frames")

WIDTH = 1280
HEIGHT = 720

# Camera Parameters
# World coordinates: +X forward, +Y left, +Z up
CAM_POS = (2.3, -2.4, 1.5)
CAM_TARGET = (0.0, 0.0, 0.65)
FOV = 45.0  # degrees

# Compute camera coordinate frame
dx, dy, dz = CAM_TARGET[0] - CAM_POS[0], CAM_TARGET[1] - CAM_POS[1], CAM_TARGET[2] - CAM_POS[2]
dist = math.sqrt(dx * dx + dy * dy + dz * dz)
F = (dx / dist, dy / dist, dz / dist)  # Forward
# Right = F x Up, Up = (0, 0, 1)
rx, ry, rz = F[1], -F[0], 0
rdist = math.sqrt(rx * rx + ry * ry)
R = (rx / rdist, ry / rdist, 0)
# Up = R x F
U = (R[1] * F[2] - R[2] * F[1], R[2] * F[0] - R[0] * F[2], R[0] * F[1] - R[1] * F[0])
FOCAL = (HEIGHT / 2.0) / math.tan(math.radians(FOV / 2.0))


def project_3d(x, y, z):
    """Projects world 3D coordinate (x, y, z) into screen (u, v, depth)."""
    px = x - CAM_POS[0]
    py = y - CAM_POS[1]
    pz = z - CAM_POS[2]
    xc = px * R[0] + py * R[1] + pz * R[2]
    yc = px * U[0] + py * U[1] + pz * U[2]
    zc = px * F[0] + py * F[1] + pz * F[2]
    if zc <= 0.1:
        return None
    u = int(WIDTH / 2.0 + (xc / zc) * FOCAL)
    v = int(HEIGHT / 2.0 - (yc / zc) * FOCAL)
    return (u, v, zc)


# Minimal 5x7 ASCII Bitmap Font for HUD telemetry
FONT_5X7 = {
    'A': [0x0C, 0x12, 0x12, 0x1E, 0x12, 0x12, 0x12],
    'B': [0x1C, 0x12, 0x12, 0x1C, 0x12, 0x12, 0x1C],
    'C': [0x0E, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0E],
    'D': [0x1C, 0x12, 0x12, 0x12, 0x12, 0x12, 0x1C],
    'E': [0x1E, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x1E],
    'F': [0x1E, 0x10, 0x10, 0x1C, 0x10, 0x10, 0x10],
    'G': [0x0E, 0x10, 0x10, 0x16, 0x12, 0x12, 0x0E],
    'H': [0x12, 0x12, 0x12, 0x1E, 0x12, 0x12, 0x12],
    'I': [0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E],
    'J': [0x02, 0x02, 0x02, 0x02, 0x12, 0x12, 0x0C],
    'K': [0x12, 0x14, 0x18, 0x10, 0x18, 0x14, 0x12],
    'L': [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1E],
    'M': [0x11, 0x1B, 0x15, 0x11, 0x11, 0x11, 0x11],
    'N': [0x12, 0x16, 0x1A, 0x12, 0x12, 0x12, 0x12],
    'O': [0x0C, 0x12, 0x12, 0x12, 0x12, 0x12, 0x0C],
    'P': [0x1C, 0x12, 0x12, 0x1C, 0x10, 0x10, 0x10],
    'Q': [0x0C, 0x12, 0x12, 0x12, 0x16, 0x12, 0x0D],
    'R': [0x1C, 0x12, 0x12, 0x1C, 0x14, 0x12, 0x11],
    'S': [0x0E, 0x10, 0x10, 0x0E, 0x02, 0x02, 0x1C],
    'T': [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
    'U': [0x12, 0x12, 0x12, 0x12, 0x12, 0x12, 0x0C],
    'V': [0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04],
    'W': [0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11],
    'X': [0x11, 0x0A, 0x04, 0x04, 0x04, 0x0A, 0x11],
    'Y': [0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04],
    'Z': [0x1F, 0x02, 0x04, 0x08, 0x10, 0x10, 0x1F],
    '0': [0x0E, 0x13, 0x15, 0x19, 0x11, 0x11, 0x0E],
    '1': [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
    '2': [0x0E, 0x11, 0x01, 0x06, 0x08, 0x10, 0x1F],
    '3': [0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E],
    '4': [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
    '5': [0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E],
    '6': [0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E],
    '7': [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
    '8': [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
    '9': [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E],
    ':': [0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00],
    '.': [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C],
    '-': [0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00],
    '+': [0x00, 0x04, 0x04, 0x1F, 0x04, 0x04, 0x00],
    '[': [0x0E, 0x08, 0x08, 0x08, 0x08, 0x08, 0x0E],
    ']': [0x0E, 0x02, 0x02, 0x02, 0x02, 0x02, 0x0E],
    '/': [0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10],
    '_': [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F],
    '(': [0x02, 0x04, 0x08, 0x08, 0x08, 0x04, 0x02],
    ')': [0x08, 0x04, 0x02, 0x02, 0x02, 0x04, 0x08],
    '©': [0x0E, 0x15, 0x17, 0x15, 0x0E, 0x00, 0x00],
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
}


class FrameBuffer:
    def __init__(self, w=WIDTH, h=HEIGHT):
        self.w = w
        self.h = h
        self.pixels = bytearray(w * h * 3)
        self.z_buffer = [float('inf')] * (w * h)
        self.clear((15, 17, 23))  # Dark navy background

    def clear(self, color):
        r, g, b = color
        for i in range(self.w * self.h):
            self.pixels[i * 3] = r
            self.pixels[i * 3 + 1] = g
            self.pixels[i * 3 + 2] = b
            self.z_buffer[i] = float('inf')

    def set_pixel(self, x, y, z, color):
        if 0 <= x < self.w and 0 <= y < self.h:
            idx = y * self.w + x
            if z <= self.z_buffer[idx]:
                self.z_buffer[idx] = z
                self.pixels[idx * 3] = color[0]
                self.pixels[idx * 3 + 1] = color[1]
                self.pixels[idx * 3 + 2] = color[2]

    def draw_line_2d(self, x0, y0, x1, y1, z, color, thickness=1):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        radius = thickness // 2

        while True:
            for ox in range(-radius, radius + 1):
                for oy in range(-radius, radius + 1):
                    self.set_pixel(x0 + ox, y0 + oy, z, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x0 += sx
            if e2 < dx:
                err += dx
                y0 += sy

    def draw_sphere_2d(self, cx, cy, z, radius, color):
        r_sq = radius * radius
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                dist_sq = dx * dx + dy * dy
                if dist_sq <= r_sq:
                    # Subtle shading
                    shade = 1.0 - (dist_sq / r_sq) * 0.4
                    c = (int(color[0] * shade), int(color[1] * shade), int(color[2] * shade))
                    self.set_pixel(cx + dx, cy + dy, z - 0.05, c)

    def draw_text(self, x, y, text, color=(240, 240, 240), scale=2):
        text = text.upper()
        curr_x = x
        for ch in text:
            bitmap = FONT_5X7.get(ch, FONT_5X7[' '])
            for row_idx, row_val in enumerate(bitmap):
                for col_idx in range(5):
                    if (row_val >> (4 - col_idx)) & 1:
                        for sx in range(scale):
                            for sy in range(scale):
                                px = curr_x + col_idx * scale + sx
                                py = y + row_idx * scale + sy
                                if 0 <= px < self.w and 0 <= py < self.h:
                                    idx = py * self.w + px
                                    self.pixels[idx * 3] = color[0]
                                    self.pixels[idx * 3 + 1] = color[1]
                                    self.pixels[idx * 3 + 2] = color[2]
            curr_x += (5 + 1) * scale

    def to_ppm_bytes(self):
        header = f"P6\n{self.w} {self.h}\n255\n".encode("ascii")
        return header + self.pixels


def draw_ground_grid(fb):
    grid_color = (35, 45, 65)
    axis_x_color = (220, 60, 60)
    axis_y_color = (40, 180, 100)

    # Grid lines from -2.0 to 2.0 m in XY plane
    for v in [x * 0.25 for x in range(-8, 9)]:
        # Lines parallel to X
        p0 = project_3d(-2.0, v, 0.0)
        p1 = project_3d(2.0, v, 0.0)
        if p0 and p1:
            col = axis_x_color if abs(v) < 1e-4 else grid_color
            fb.draw_line_2d(p0[0], p0[1], p1[0], p1[1], p0[2], col, thickness=2 if abs(v) < 1e-4 else 1)

        # Lines parallel to Y
        p0 = project_3d(v, -2.0, 0.0)
        p1 = project_3d(v, 2.0, 0.0)
        if p0 and p1:
            col = axis_y_color if abs(v) < 1e-4 else grid_color
            fb.draw_line_2d(p0[0], p0[1], p1[0], p1[1], p0[2], col, thickness=2 if abs(v) < 1e-4 else 1)

    # Z axis at origin
    p_orig = project_3d(0, 0, 0)
    p_z = project_3d(0, 0, 0.4)
    if p_orig and p_z:
        fb.draw_line_2d(p_orig[0], p_orig[1], p_z[0], p_z[1], p_orig[2], (60, 130, 246), thickness=3)


def render_robot_frame(fb, transforms_map, fsm_state_name, time_in_state, frame_idx):
    # 1. Background & Ground grid
    fb.clear((15, 17, 23))
    draw_ground_grid(fb)

    # Extract positions from matrix (translation at indices 12, 13, 14)
    def pos(name):
        m = transforms_map.get(name)
        return (m[12], m[13], m[14]) if m and len(m) == 16 else None

    # Colors
    c_base = (90, 95, 105)
    c_wheel = (25, 130, 200)
    c_torso = (220, 225, 235)
    c_shoulder = (240, 150, 40)
    c_arm = (70, 145, 230)
    c_forearm = (50, 180, 140)
    c_wrist = (180, 110, 240)
    c_gripper = (240, 200, 40)
    c_head = (235, 240, 245)

    # 1. Base pedestal
    p_base = pos("base_link")
    if p_base:
        p_screen = project_3d(*p_base)
        if p_screen:
            fb.draw_sphere_2d(p_screen[0], p_screen[1], p_screen[2], 26, c_base)

    # Left & Right wheels
    p_rw = pos("r_wheel_link")
    p_lw = pos("l_wheel_link")
    if p_rw and p_lw:
        s_rw = project_3d(*p_rw)
        s_lw = project_3d(*p_lw)
        if s_rw: fb.draw_sphere_2d(s_rw[0], s_rw[1], s_rw[2], 14, c_wheel)
        if s_lw: fb.draw_sphere_2d(s_lw[0], s_lw[1], s_lw[2], 14, c_wheel)

    # 2. Torso Lift
    p_torso = pos("torso_lift_link")
    if p_base and p_torso:
        s_b = project_3d(p_base[0], p_base[1], p_base[2] + 0.1)
        s_t = project_3d(*p_torso)
        if s_b and s_t:
            fb.draw_line_2d(s_b[0], s_b[1], s_t[0], s_t[1], s_t[2], c_torso, thickness=18)

    # 3. Head Pan & Tilt
    p_hpan = pos("head_pan_link")
    p_htilt = pos("head_tilt_link")
    if p_torso and p_hpan:
        s_t = project_3d(*p_torso)
        s_hp = project_3d(*p_hpan)
        if s_t and s_hp:
            fb.draw_line_2d(s_t[0], s_t[1], s_hp[0], s_hp[1], s_hp[2], (180, 185, 195), thickness=10)
    if p_hpan and p_htilt:
        s_hp = project_3d(*p_hpan)
        s_ht = project_3d(*p_htilt)
        if s_hp and s_ht:
            fb.draw_line_2d(s_hp[0], s_hp[1], s_ht[0], s_ht[1], s_ht[2], c_head, thickness=14)
            # Head visor
            fb.draw_sphere_2d(s_ht[0], s_ht[1], s_ht[2], 16, (30, 30, 35))

    # 4. Kinematic Arm Segments
    p_span = pos("shoulder_pan_link")
    p_slift = pos("shoulder_lift_link")
    p_uroll = pos("upperarm_roll_link")
    p_eflex = pos("elbow_flex_link")
    p_froll = pos("forearm_roll_link")
    p_wflex = pos("wrist_flex_link")
    p_wroll = pos("wrist_roll_link")
    p_grip = pos("gripper_link")
    p_rgrip = pos("r_gripper_finger_link")
    p_lgrip = pos("l_gripper_finger_link")

    # Draw chain
    def draw_segment(p1, p2, col, thick=8):
        if p1 and p2:
            s1 = project_3d(*p1)
            s2 = project_3d(*p2)
            if s1 and s2:
                fb.draw_line_2d(s1[0], s1[1], s2[0], s2[1], s1[2], col, thickness=thick)
                fb.draw_sphere_2d(s1[0], s1[1], s1[2], thick // 2 + 3, (255, 255, 255))
                fb.draw_sphere_2d(s2[0], s2[1], s2[2], thick // 2 + 3, (255, 255, 255))

    draw_segment(p_torso, p_span, c_shoulder, 12)
    draw_segment(p_span, p_slift, c_shoulder, 12)
    draw_segment(p_slift, p_uroll, c_arm, 10)
    draw_segment(p_uroll, p_eflex, c_arm, 10)
    draw_segment(p_eflex, p_froll, c_forearm, 9)
    draw_segment(p_froll, p_wflex, c_forearm, 9)
    draw_segment(p_wflex, p_wroll, c_wrist, 8)
    draw_segment(p_wroll, p_grip, c_wrist, 8)
    draw_segment(p_grip, p_rgrip, c_gripper, 5)
    draw_segment(p_grip, p_lgrip, c_gripper, 5)

    # 5. Professional HUD Overlays
    # Top Bar Header
    for y in range(48):
        for x in range(WIDTH):
            idx = y * WIDTH + x
            fb.pixels[idx * 3] = 22
            fb.pixels[idx * 3 + 1] = 26
            fb.pixels[idx * 3 + 2] = 36

    fb.draw_text(24, 16, "FETCH MOBILE MANIPULATOR // EECS 367 PROJECT 3", (255, 255, 255), scale=2)
    fb.draw_text(WIDTH - 380, 16, "AUTONOMOUS FSM CHOREOGRAPHY", (6, 182, 212), scale=2)

    # Left telemetry card
    card_x, card_y, card_w, card_h = 24, 70, 380, 190
    for y in range(card_y, card_y + card_h):
        for x in range(card_x, card_x + card_w):
            idx = y * WIDTH + x
            fb.pixels[idx * 3] = int(fb.pixels[idx * 3] * 0.35 + 26 * 0.65)
            fb.pixels[idx * 3 + 1] = int(fb.pixels[idx * 3 + 1] * 0.35 + 30 * 0.65)
            fb.pixels[idx * 3 + 2] = int(fb.pixels[idx * 3 + 2] * 0.35 + 42 * 0.65)

    fb.draw_text(card_x + 16, card_y + 16, "CHOREOGRAPHY STATUS", (148, 163, 184), scale=2)
    state_display = f"STATE: [{fsm_state_name.upper()}]"
    fb.draw_text(card_x + 16, card_y + 44, state_display, (16, 185, 129), scale=3)
    fb.draw_text(card_x + 16, card_y + 80, f"TIME IN STATE: {time_in_state:.2f} S", (240, 240, 240), scale=2)
    fb.draw_text(card_x + 16, card_y + 104, "TOPIC: /XFORM_WORLD (10 HZ)", (59, 130, 246), scale=2)

    # Torso height & wrist coordinates
    if p_torso:
        fb.draw_text(card_x + 16, card_y + 132, f"TORSO ELEVATION: {p_torso[2]:.3f} M", (245, 158, 11), scale=2)
    if p_grip:
        fb.draw_text(card_x + 16, card_y + 156, f"GRIPPER POSE: ({p_grip[0]:.2f}, {p_grip[1]:.2f}, {p_grip[2]:.2f})", (192, 132, 252), scale=2)

    # Bottom Attribution Banner (MANDATORY REQUIREMENT)
    for y in range(HEIGHT - 44, HEIGHT):
        for x in range(WIDTH):
            idx = y * WIDTH + x
            fb.pixels[idx * 3] = 16
            fb.pixels[idx * 3 + 1] = 19
            fb.pixels[idx * 3 + 2] = 27

    credit_text = "FETCH ROBOT DESCRIPTION (C) FETCH ROBOTICS, INC., CC BY-NC-SA 4.0"
    fb.draw_text(24, HEIGHT - 28, credit_text, (200, 210, 225), scale=2)
    fb.draw_text(WIDTH - 260, HEIGHT - 28, "FORWARD KINEMATICS", (148, 163, 184), scale=2)


def generate_portfolio_demonstration_video():
    print("=== Generating Official Portfolio Video Deliverable ===")
    os.makedirs(FRAMES_DIR, exist_ok=True)
    os.makedirs(PORTFOLIO_DIR, exist_ok=True)

    # 1. Start demo runtime
    print("Starting ./main --demo...")
    proc = subprocess.Popen(["./main", "--demo"], cwd=ROOT_DIR)
    time.sleep(0.3)

    sock = socket.create_connection(("127.0.0.1", 9095), 1.5)
    f_in = sock.makefile("r")
    f_out = sock.makefile("w")

    call_id_gen = itertools.count(1)

    def call(srv, args):
        cid = f"call-{next(call_id_gen)}"
        f_out.write(json.dumps({"op": "call_service", "service": srv, "id": cid, "args": args}) + "\n")
        f_out.flush()
        while True:
            line = f_in.readline()
            if not line: return {}
            m = json.loads(line)
            if m.get("op") == "service_response" and m.get("id") == cid:
                return m

    def send_publish(topic, msg):
        f_out.write(json.dumps({"op": "publish", "topic": topic, "msg": msg}) + "\n")
        f_out.flush()

    try:
        # Load Fetch URDF
        with open(os.path.join(ROOT_DIR, "fetch", "fetch.urdf"), "r", encoding="utf-8") as f:
            urdf = f.read()
        res = call("/param_server/set_param", {"name": "robot_description", "value": urdf})
        assert res.get("result"), "Failed to set robot_description"
        print("Fetch URDF loaded.")
        time.sleep(0.3)

        # Load fetch_dance FSM
        with open(os.path.join(ROOT_DIR, "fsm", "fetch_dance.json"), "r", encoding="utf-8") as f:
            fsm_doc = json.load(f)
        res_fsm = call("/fsm/update_logic", fsm_doc)
        assert res_fsm.get("result"), "Failed to update FSM logic"
        print("fetch_dance FSM loaded.")

        # Subscribe to /xform_world
        f_out.write(json.dumps({"op": "subscribe", "topic": "/xform_world", "type": "xform_world/MatrixTransformArray"}) + "\n")
        f_out.flush()

        running = True
        latest_transforms = {}
        tf_event = threading.Event()
        lock = threading.Lock()

        def bg_reader():
            while running:
                try:
                    line = f_in.readline()
                    if not line:
                        break
                    msg = json.loads(line)
                    if msg.get("op") == "publish" and msg.get("topic") == "/xform_world":
                        with lock:
                            for entry in msg["msg"]["transforms"]:
                                latest_transforms[entry["child_frame_id"]] = entry["matrix"]
                        tf_event.set()
                except Exception:
                    break

        reader_thread = threading.Thread(target=bg_reader, daemon=True)
        reader_thread.start()

        # Choreography Keyframes interpolation
        # 5 states: home, rise_and_reach, wave_right, wave_left, bow
        states = [s for s in fsm_doc["states"]]
        state_names = [s["name"] for s in states]

        # We will interpolate 20 frames between each state transition: 5 x 20 = 100 frames (5 seconds at 20 fps)
        fb = FrameBuffer(WIDTH, HEIGHT)
        frame_idx = 0

        total_transitions = len(states)
        steps_per_trans = 20

        print("Rendering 100 3D world-state frames from /xform_world...")

        for s_idx in range(total_transitions):
            s1 = states[s_idx]
            s2 = states[(s_idx + 1) % total_transitions]

            joints = s1["target"]["joint_names"]
            q1 = s1["target"]["positions"]
            q2 = s2["target"]["positions"]

            for step in range(steps_per_trans):
                alpha = step / float(steps_per_trans)
                # Smooth cosine interpolation (smoothstep)
                t = 0.5 - 0.5 * math.cos(alpha * math.pi)

                # Interpolate positions
                q_interp = [q1[i] + t * (q2[i] - q1[i]) for i in range(len(joints))]

                # Publish to /joint_states
                j_msg = {
                    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                    "name": joints,
                    "position": q_interp,
                    "velocity": [],
                    "effort": []
                }
                tf_event.clear()
                send_publish("/joint_states", j_msg)
                tf_event.wait(timeout=0.2)

                with lock:
                    transforms_map = dict(latest_transforms)

                time_in_state = (step * 0.05)
                state_label = s1["name"] if alpha < 0.5 else s2["name"]

                render_robot_frame(fb, transforms_map, state_label, time_in_state, frame_idx)

                # Save frame
                frame_filename = os.path.join(FRAMES_DIR, f"frame_{frame_idx:04d}.ppm")
                with open(frame_filename, "wb") as f_ppm:
                    f_ppm.write(fb.to_ppm_bytes())

                frame_idx += 1

        print(f"Rendered {frame_idx} frames successfully.")

        # Compile video using ffmpeg
        print("Compiling video via ffmpeg...")
        mp4_path = os.path.join(PORTFOLIO_DIR, "fetch_choreography.mp4")
        webm_path = os.path.join(PORTFOLIO_DIR, "fetch_choreography.webm")
        poster_path = os.path.join(PORTFOLIO_DIR, "poster.png")

        # Compile MP4
        cmd_mp4 = [
            "ffmpeg", "-y", "-framerate", "20",
            "-i", os.path.join(FRAMES_DIR, "frame_%04d.ppm"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-preset", "fast", "-crf", "22",
            mp4_path
        ]
        subprocess.run(cmd_mp4, check=True)

        # Compile WebM
        cmd_webm = [
            "ffmpeg", "-y", "-i", mp4_path,
            "-c:v", "libvpx-vp9", "-b:v", "1200k",
            webm_path
        ]
        subprocess.run(cmd_webm, check=True)

        # Poster image (frame 35: wave_right)
        poster_ppm = os.path.join(FRAMES_DIR, "frame_0035.ppm")
        if os.path.exists(poster_ppm):
            subprocess.run(["ffmpeg", "-y", "-i", poster_ppm, poster_path], check=True)

        print(f"Generated MP4 deliverable: {mp4_path} ({os.path.getsize(mp4_path):,} bytes)")
        print(f"Generated WebM deliverable: {webm_path} ({os.path.getsize(webm_path):,} bytes)")
        print(f"Generated Poster deliverable: {poster_path} ({os.path.getsize(poster_path):,} bytes)")

        # Cleanup PPM frames
        shutil.rmtree(FRAMES_DIR, ignore_errors=True)
        print("Temporary frame assets cleaned up.")
        return True

    finally:
        sock.close()
        proc.terminate()
        try:
            proc.wait(timeout=1.0)
        except Exception:
            proc.kill()


if __name__ == "__main__":
    generate_portfolio_demonstration_video()
