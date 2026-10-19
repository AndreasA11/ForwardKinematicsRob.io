#!/usr/bin/env python3
"""Generates the official Project 3 choreography demonstration video for the course portfolio.

Connects to the C++ runtime over TCP on 127.0.0.1:9095, loads the 7-DOF Chess Manipulator Arm URDF,
executes the FSM choreography (chess_pick_and_place), captures high-definition 3D world-state
transforms from /xform_world, renders depth-shaded perspective frames, and compiles
them via ffmpeg into:
  - portfolio/chess_arm_choreography.mp4 (and fetch_choreography.mp4)
  - portfolio/chess_arm_choreography.webm (and fetch_choreography.webm)
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
# Framed around table (x=0.15, y=0.0, z=0.0) and chess board (x=0.26, y=0.0)
CAM_POS = (0.78, -0.68, 0.48)
CAM_TARGET = (0.16, 0.0, 0.08)
FOV = 42.0  # degrees

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
    if zc <= 0.05:
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
    '|': [0x04, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
    ' ': [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
}


class FrameBuffer:
    def __init__(self, w=WIDTH, h=HEIGHT):
        self.w = w
        self.h = h
        self.pixels = bytearray(w * h * 3)
        self.z_buffer = [float('inf')] * (w * h)
        self.clear((15, 17, 23))

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
    grid_color = (30, 36, 50)
    for v in [x * 0.20 for x in range(-5, 6)]:
        p0 = project_3d(-0.6, v, -0.04)
        p1 = project_3d(0.8, v, -0.04)
        if p0 and p1:
            fb.draw_line_2d(p0[0], p0[1], p1[0], p1[1], p0[2], grid_color, thickness=1)
        p0 = project_3d(v, -0.6, -0.04)
        p1 = project_3d(v, 0.6, -0.04)
        if p0 and p1:
            fb.draw_line_2d(p0[0], p0[1], p1[0], p1[1], p0[2], grid_color, thickness=1)


def draw_table_and_board(fb):
    # 1. Table Top (center ~ 0.15, 0.0, -0.02)
    c_table = (45, 50, 60)
    p_t1 = project_3d(-0.25, -0.40, -0.02)
    p_t2 = project_3d(0.55, -0.40, -0.02)
    p_t3 = project_3d(0.55, 0.40, -0.02)
    p_t4 = project_3d(-0.25, 0.40, -0.02)
    if p_t1 and p_t2 and p_t3 and p_t4:
        fb.draw_line_2d(p_t1[0], p_t1[1], p_t2[0], p_t2[1], p_t1[2], c_table, thickness=3)
        fb.draw_line_2d(p_t2[0], p_t2[1], p_t3[0], p_t3[1], p_t2[2], c_table, thickness=3)
        fb.draw_line_2d(p_t3[0], p_t3[1], p_t4[0], p_t4[1], p_t3[2], c_table, thickness=3)
        fb.draw_line_2d(p_t4[0], p_t4[1], p_t1[0], p_t1[1], p_t4[2], c_table, thickness=3)

    # 2. Chess Board Border (center: x=0.26, y=0.0, z=0.007)
    c_border = (80, 50, 30)
    b_min_x, b_max_x = 0.26 - 0.17, 0.26 + 0.17
    b_min_y, b_max_y = -0.17, 0.17
    pb1 = project_3d(b_min_x, b_min_y, 0.01)
    pb2 = project_3d(b_max_x, b_min_y, 0.01)
    pb3 = project_3d(b_max_x, b_max_y, 0.01)
    pb4 = project_3d(b_min_x, b_max_y, 0.01)
    if pb1 and pb2 and pb3 and pb4:
        fb.draw_line_2d(pb1[0], pb1[1], pb2[0], pb2[1], pb1[2], c_border, thickness=5)
        fb.draw_line_2d(pb2[0], pb2[1], pb3[0], pb3[1], pb2[2], c_border, thickness=5)
        fb.draw_line_2d(pb3[0], pb3[1], pb4[0], pb4[1], pb3[2], c_border, thickness=5)
        fb.draw_line_2d(pb4[0], pb4[1], pb1[0], pb1[1], pb4[2], c_border, thickness=5)

    # 3. 8x8 Checkered Grid on Board
    col_light = (215, 205, 180)
    col_dark = (70, 48, 32)
    tile_size = 0.04
    start_x = 0.26 - 4 * tile_size
    start_y = -4 * tile_size

    for r in range(8):
        for c in range(8):
            x = start_x + (r + 0.5) * tile_size
            y = start_y + (c + 0.5) * tile_size
            pt = project_3d(x, y, 0.014)
            if pt:
                tile_col = col_light if (r + c) % 2 == 0 else col_dark
                fb.draw_sphere_2d(pt[0], pt[1], pt[2], 5, tile_col)


def render_robot_frame(fb, transforms_map, fsm_state_name, time_in_state, frame_idx):
    # 1. Background, Ground grid, Table & Board
    fb.clear((15, 17, 23))
    draw_ground_grid(fb)
    draw_table_and_board(fb)

    def pos(name):
        m = transforms_map.get(name)
        return (m[12], m[13], m[14]) if m and len(m) == 16 else None

    # Colors
    c_pedestal = (65, 70, 80)
    c_arm_pri = (40, 115, 215)
    c_arm_acc = (75, 80, 95)
    c_wrist = (35, 38, 45)
    c_finger = (235, 185, 45)
    c_white_piece = (245, 240, 225)
    c_black_piece = (35, 35, 40)

    # 2. Render Chess Pieces on the Board
    pieces = [
        ("piece_pawn_e2", c_white_piece, 10, 0.035),
        ("piece_pawn_d2", c_white_piece, 9, 0.035),
        ("piece_pawn_e7", c_black_piece, 9, 0.035),
        ("piece_king_white", c_white_piece, 12, 0.055),
        ("piece_king_black", c_black_piece, 12, 0.055),
        ("piece_knight_white", c_white_piece, 11, 0.048),
    ]
    for p_name, col, rad, h in pieces:
        p_pt = pos(p_name)
        if p_pt:
            p_base = project_3d(p_pt[0], p_pt[1], p_pt[2])
            p_top = project_3d(p_pt[0], p_pt[1], p_pt[2] + h)
            if p_base and p_top:
                thick = rad // 2 + 4 if p_name == "piece_pawn_e2" else rad // 2 + 3
                fb.draw_line_2d(p_base[0], p_base[1], p_top[0], p_top[1], p_base[2], col, thickness=thick)
                fb.draw_sphere_2d(p_top[0], p_top[1], p_top[2], rad, col)
                if p_name == "piece_pawn_e2":
                    p_mid = project_3d(p_pt[0], p_pt[1], p_pt[2] + h * 0.65)
                    if p_mid:
                        fb.draw_sphere_2d(p_mid[0], p_mid[1], p_mid[2], rad // 2 + 2, (235, 185, 45))

    # 3. Render Robot Arm Kinematic Chain
    p_base = pos("arm_base_link")
    p_sh = pos("shoulder_link")
    p_uarm = pos("upper_arm_link")
    p_farm = pos("forearm_link")
    p_wpitch = pos("wrist_pitch_link")
    p_wroll = pos("wrist_roll_link")
    p_grip = pos("gripper_base")
    p_lfing = pos("gripper_left_finger")
    p_rfing = pos("gripper_right_finger")

    # Pedestal base
    if p_base:
        s_base = project_3d(*p_base)
        if s_base:
            fb.draw_sphere_2d(s_base[0], s_base[1], s_base[2], 24, c_pedestal)

    def draw_link(p1, p2, col, thick=10):
        if p1 and p2:
            s1 = project_3d(*p1)
            s2 = project_3d(*p2)
            if s1 and s2:
                fb.draw_line_2d(s1[0], s1[1], s2[0], s2[1], s1[2], col, thickness=thick)
                fb.draw_sphere_2d(s1[0], s1[1], s1[2], thick // 2 + 3, (240, 245, 255))
                fb.draw_sphere_2d(s2[0], s2[1], s2[2], thick // 2 + 3, (240, 245, 255))

    draw_link(p_base, p_sh, c_arm_pri, 16)
    draw_link(p_sh, p_uarm, c_arm_acc, 14)
    draw_link(p_uarm, p_farm, c_arm_pri, 12)
    draw_link(p_farm, p_wpitch, c_arm_acc, 10)
    draw_link(p_wpitch, p_wroll, c_wrist, 9)
    draw_link(p_wroll, p_grip, c_wrist, 8)

    # Gripper Jaws
    if p_grip and p_lfing:
        draw_link(p_grip, p_lfing, c_finger, 7)
    if p_grip and p_rfing:
        draw_link(p_grip, p_rfing, c_finger, 7)

    # 4. Professional HUD Overlays
    # Top Bar Header
    for y in range(48):
        for x in range(WIDTH):
            idx = y * WIDTH + x
            fb.pixels[idx * 3] = 22
            fb.pixels[idx * 3 + 1] = 26
            fb.pixels[idx * 3 + 2] = 36

    fb.draw_text(24, 12, "CHESS MANIPULATOR ARM // AUTONOMOUS 1. E4 PICK-AND-PLACE", (100, 200, 255), scale=2)
    fb.draw_text(24, 30, "7-DOF ARTICULATED ARM | 10 HZ FORWARD KINEMATICS | COLUMN-MAJOR /XFORM_WORLD", (150, 160, 180), scale=1)

    # Status Cards in Top Right
    fb.draw_text(WIDTH - 340, 12, "CHOREOGRAPHY:", (160, 170, 185), scale=1)
    fb.draw_text(WIDTH - 220, 12, "CHESS_PICK_AND_PLACE", (50, 220, 140), scale=1)

    fb.draw_text(WIDTH - 340, 28, "FSM STATE:", (160, 170, 185), scale=1)
    fb.draw_text(WIDTH - 220, 28, f"{fsm_state_name.upper()} ({time_in_state:.2f}S)", (250, 200, 50), scale=1)

    # Bottom Telemetry Box
    bot_y = HEIGHT - 64
    for y in range(bot_y, HEIGHT):
        for x in range(WIDTH):
            idx = y * WIDTH + x
            fb.pixels[idx * 3] = 18
            fb.pixels[idx * 3 + 1] = 20
            fb.pixels[idx * 3 + 2] = 28

    if p_grip:
        grip_str = f"GRIPPER POSE: X={p_grip[0]:.3f} Y={p_grip[1]:.3f} Z={p_grip[2]:.3f} M"
        fb.draw_text(24, bot_y + 12, grip_str, (120, 220, 180), scale=1)

    fb.draw_text(24, bot_y + 32, "COORDINATES: Z-UP, X-FORWARD, Y-LEFT | DYNAMIC LINE BUFFERS (>1 MIB)", (160, 170, 185), scale=1)
    fb.draw_text(WIDTH - 300, bot_y + 20, "EECS 367 PROJECT 3 PORTFOLIO", (100, 180, 255), scale=2)


def generate_portfolio_demonstration_video(speed=0.5):
    print(f"=== Generating Chess Manipulator Arm Portfolio Video (speed={speed}x) ===")
    os.makedirs(PORTFOLIO_DIR, exist_ok=True)
    os.makedirs(FRAMES_DIR, exist_ok=True)

    # 1. Compile main if needed
    main_bin = os.path.join(ROOT_DIR, "main")
    if not os.path.exists(main_bin):
        print("Compiling main...")
        subprocess.run(["make", "build"], cwd=ROOT_DIR, check=True)

    # 2. Launch runtime under demo mode
    print("Launching ./main --demo...")
    proc = subprocess.Popen([main_bin, "--demo"], cwd=ROOT_DIR)
    time.sleep(0.4)

    sock = socket.create_connection(("127.0.0.1", 9095), timeout=5.0)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    f_in = sock.makefile("r", encoding="utf-8")
    f_out = sock.makefile("w", encoding="utf-8")

    call_id = 0

    def call(service, args):
        nonlocal call_id
        call_id += 1
        cid = f"call_{call_id}"
        req = {"op": "call_service", "id": cid, "service": service, "args": args}
        f_out.write(json.dumps(req) + "\n")
        f_out.flush()
        while True:
            line = f_in.readline()
            if not line:
                raise RuntimeError("EOF while waiting for service response")
            msg = json.loads(line)
            if msg.get("op") == "service_response" and msg.get("id") == cid:
                return msg

    def send_publish(topic, msg_val):
        pub = {"op": "publish", "topic": topic, "msg": msg_val}
        f_out.write(json.dumps(pub) + "\n")
        f_out.flush()

    try:
        # Load Chess Arm URDF
        with open(os.path.join(ROOT_DIR, "urdf-files", "chess_arm.urdf"), "r", encoding="utf-8") as f:
            urdf = f.read()
        res = call("/param_server/set_param", {"name": "robot_description", "value": urdf})
        assert res.get("result"), "Failed to set robot_description"
        print("Chess Arm URDF loaded.")
        time.sleep(0.3)

        # Load chess_pick_and_place FSM
        with open(os.path.join(ROOT_DIR, "fsm", "chess_pick_and_place.json"), "r", encoding="utf-8") as f:
            fsm_doc = json.load(f)
        res_fsm = call("/fsm/update_logic", fsm_doc)
        assert res_fsm.get("result"), "Failed to update FSM logic"
        print("chess_pick_and_place FSM loaded.")

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

        # States to interpolate across 9-state chess pick-and-place
        states = [s for s in fsm_doc["states"]]
        fb = FrameBuffer(WIDTH, HEIGHT)
        frame_idx = 0

        total_transitions = len(states)
        steps_per_trans = int(round(24 / speed))  # e.g. 48 steps per transition at 0.5x (~2.4s at 20 fps)

        # Hold durations at key milestones scaled for slow, legible pick-and-place
        base_holds = {
            "home": 16,
            "hover_e2": 12,
            "descend_e2": 14,
            "grasp_pawn": 18,
            "lift_pawn": 14,
            "carry_to_e4": 16,
            "place_on_e4": 18,
            "release_pawn": 18,
            "retract_ascent": 14
        }
        hold_counts = {k: int(round(v / speed)) for k, v in base_holds.items()}

        print(f"Rendering high-definition 3D world-state frames from /xform_world (transitions: {steps_per_trans} frames each)...")
        poster_frame_idx = 0

        for s_idx in range(total_transitions):
            s1 = states[s_idx]
            s2 = states[(s_idx + 1) % total_transitions]

            t1 = s1["target"]
            t2 = s2["target"]
            if isinstance(t1, dict) and "positions" in t1:
                joints = t1["joint_names"]
                q1 = t1["positions"]
                q2 = t2["positions"]
            else:
                joints = list(t1.keys())
                q1 = [t1[k] for k in joints]
                q2 = [t2.get(k, 0.0) for k in joints]

            # 1. State hold pause frames
            n_holds = hold_counts.get(s1["name"], 0)
            for h_step in range(n_holds):
                if s1["name"] == "lift_pawn" and h_step == n_holds // 2:
                    poster_frame_idx = frame_idx
                j_msg = {
                    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                    "name": joints,
                    "position": q1,
                    "velocity": [],
                    "effort": []
                }
                tf_event.clear()
                send_publish("/joint_states", j_msg)
                tf_event.wait(timeout=0.1)

                with lock:
                    transforms_map = dict(latest_transforms)

                time_in_state = (h_step * 0.05)
                render_robot_frame(fb, transforms_map, s1["name"], time_in_state, frame_idx)

                frame_filename = os.path.join(FRAMES_DIR, f"frame_{frame_idx:04d}.ppm")
                with open(frame_filename, "wb") as f_ppm:
                    f_ppm.write(fb.to_ppm_bytes())
                frame_idx += 1

            # 2. Smooth transition interpolation frames
            for step in range(steps_per_trans):
                alpha = step / float(steps_per_trans)
                # Smoothstep
                t = 0.5 - 0.5 * math.cos(alpha * math.pi)
                q_interp = [q1[i] + t * (q2[i] - q1[i]) for i in range(len(joints))]

                j_msg = {
                    "header": {"stamp": {"sec": 0, "nanosec": 0}, "frame_id": ""},
                    "name": joints,
                    "position": q_interp,
                    "velocity": [],
                    "effort": []
                }
                tf_event.clear()
                send_publish("/joint_states", j_msg)
                tf_event.wait(timeout=0.1)

                with lock:
                    transforms_map = dict(latest_transforms)

                time_in_state = (step * 0.05)
                state_label = s1["name"] if alpha < 0.5 else s2["name"]

                render_robot_frame(fb, transforms_map, state_label, time_in_state, frame_idx)

                frame_filename = os.path.join(FRAMES_DIR, f"frame_{frame_idx:04d}.ppm")
                with open(frame_filename, "wb") as f_ppm:
                    f_ppm.write(fb.to_ppm_bytes())

                frame_idx += 1

        print(f"Rendered {frame_idx} frames successfully.")

        # Compile video using ffmpeg
        print("Compiling video via ffmpeg...")
        mp4_path = os.path.join(PORTFOLIO_DIR, "chess_arm_choreography.mp4")
        webm_path = os.path.join(PORTFOLIO_DIR, "chess_arm_choreography.webm")
        poster_path = os.path.join(PORTFOLIO_DIR, "poster.png")

        cmd_mp4 = [
            "ffmpeg", "-y", "-framerate", "20",
            "-i", os.path.join(FRAMES_DIR, "frame_%04d.ppm"),
            "-c:v", "libx264", "-pix_fmt", "yuv420p",
            "-preset", "fast", "-crf", "22",
            mp4_path
        ]
        subprocess.run(cmd_mp4, check=True)

        cmd_webm = [
            "ffmpeg", "-y", "-i", mp4_path,
            "-c:v", "libvpx-vp9", "-b:v", "1200k",
            webm_path
        ]
        subprocess.run(cmd_webm, check=True)

        # Poster image (frame where piece is lifted in air)
        p_frame = poster_frame_idx if poster_frame_idx > 0 else min(240, frame_idx - 1)
        poster_ppm = os.path.join(FRAMES_DIR, f"frame_{p_frame:04d}.ppm")
        if os.path.exists(poster_ppm):
            subprocess.run(["ffmpeg", "-y", "-i", poster_ppm, poster_path], check=True)

        # Also copy/symlink to fetch_choreography.* so existing paths remain functional
        fetch_mp4 = os.path.join(PORTFOLIO_DIR, "fetch_choreography.mp4")
        fetch_webm = os.path.join(PORTFOLIO_DIR, "fetch_choreography.webm")
        shutil.copyfile(mp4_path, fetch_mp4)
        shutil.copyfile(webm_path, fetch_webm)

        print(f"Generated MP4: {mp4_path} ({os.path.getsize(mp4_path):,} bytes)")
        print(f"Generated WebM: {webm_path} ({os.path.getsize(webm_path):,} bytes)")
        print(f"Generated Poster: {poster_path} ({os.path.getsize(poster_path):,} bytes)")

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
    import argparse
    parser = argparse.ArgumentParser(description="Generate high-definition portfolio demonstration video")
    parser.add_argument("--speed", type=float, default=0.5, help="Animation speed factor (default 0.5 = 2x slower, ~35s duration)")
    args = parser.parse_args()
    generate_portfolio_demonstration_video(speed=args.speed)
