#!/usr/bin/env python3
"""Automated headless video capture tool for Fetch Choreography.
Launches the demo runtime, starts the 3D viewer server, runs Google Chrome headless
with --auto_dance=1 and --auto_record=8, and converts the resulting WebM into MP4
and a high-resolution poster image for the course portfolio.
"""

import os
import subprocess
import sys
import time

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PORT = 8094
CHROME_PATH = "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"


def record_choreography(duration_sec=8):
    print(f"=== Recording {duration_sec}s Fetch Choreography Video ===")
    os.makedirs(os.path.join(ROOT_DIR, "portfolio"), exist_ok=True)

    # 1. Start demo runtime
    print("Launching ./main --demo...")
    main_proc = subprocess.Popen(["./main", "--demo"], cwd=ROOT_DIR)
    time.sleep(0.3)

    # 2. Start viewer server
    print(f"Launching viewer server on port {PORT}...")
    viewer_proc = subprocess.Popen(
        [sys.executable, "scripts/viewer.py", "--no-open", "--port", str(PORT)],
        cwd=ROOT_DIR
    )
    time.sleep(1.0)

    try:
        url = f"http://127.0.0.1:{PORT}/index.html?auto_dance=1&auto_record={duration_sec}&capture_frames=1"
        chrome_cmd = [
            CHROME_PATH,
            "--headless=new",
            "--disable-gpu",
            "--window-size=1280,720",
            "--autoplay-policy=no-user-gesture-required",
            url
        ]
        print(f"Running Chrome headless to capture canvas stream: {url}")
        # Run Chrome for duration + 4 seconds margin to complete frame capture & ffmpeg compile
        chrome_proc = subprocess.Popen(chrome_cmd)
        time.sleep(duration_sec + 4.0)
        try:
            chrome_proc.terminate()
            chrome_proc.wait(timeout=1.0)
        except Exception:
            chrome_proc.kill()

        webm_path = os.path.join(ROOT_DIR, "portfolio", "fetch_choreography.webm")
        mp4_path = os.path.join(ROOT_DIR, "portfolio", "fetch_choreography.mp4")
        poster_path = os.path.join(ROOT_DIR, "portfolio", "poster.png")

        # Check if video was generated (either mp4 or webm)
        if (os.path.exists(mp4_path) and os.path.getsize(mp4_path) > 1000) or (os.path.exists(webm_path) and os.path.getsize(webm_path) > 1000):
            print(f"Successfully generated video deliverable: {mp4_path} ({os.path.getsize(mp4_path)} bytes)")
            print(f"Generated WebM: {webm_path} ({os.path.getsize(webm_path)} bytes)")
            if os.path.exists(poster_path):
                print(f"Generated poster: {poster_path} ({os.path.getsize(poster_path)} bytes)")
            return True
        else:
            print("Video was not generated.")
            return False

    finally:
        viewer_proc.terminate()
        main_proc.terminate()
        try:
            viewer_proc.wait(timeout=1.0)
        except Exception:
            viewer_proc.kill()
        try:
            main_proc.wait(timeout=1.0)
        except Exception:
            main_proc.kill()


if __name__ == "__main__":
    record_choreography()
