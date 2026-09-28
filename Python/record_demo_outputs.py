#!/usr/bin/env python3
"""
OPTINAV — High-Definition Demo Output Recorder
Connects to Unity live stream (port 9001), runs real-time computer vision detection,
streams results to Unity Detection Bridge (port 9002) for real closed-loop gimbal actuation,
records a high-framerate MP4 video of the tracking stream with HUD telemetry & PiP mask,
and captures high-resolution screenshots for documentation.
"""

import sys
import os
import time
import socket
import struct
import json
import math
from typing import Optional, Tuple, Dict, Any, List
import numpy as np
import cv2

# Import core detector components from optinav_beacon_detector
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from optinav_beacon_detector import (
    OptinavBeaconDetector,
    ResultSender,
    recv_all,
    HEADER_SIZE,
    HEADER_FORMAT,
    EXPECTED_MAGIC,
)


class KalmanFilter2DTracker:
    """Lightweight 2D Kalman filter for visualization matching M4 contracts."""
    def __init__(self):
        self.state = np.zeros((4, 1), dtype=np.float64) # x, y, vx, vy
        self.P = np.eye(4, dtype=np.float64) * 100.0
        self.Q = np.eye(4, dtype=np.float64) * 4.0
        self.R = np.eye(2, dtype=np.float64) * 2.0
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)
        self.initialized = False
        self.consecutive_misses = 0
        self.consecutive_hits = 0
        self.status = "SEARCH"

    def update(self, detected: bool, meas: Optional[Tuple[float, float]], dt: float = 0.04):
        # State transition matrix F
        F = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float64)

        if not self.initialized:
            if detected and meas is not None:
                self.state = np.array([[meas[0]], [meas[1]], [0.0], [0.0]], dtype=np.float64)
                self.initialized = True
                self.consecutive_hits = 1
                self.status = "ACQUISITION"
            return (0.0, 0.0), self.status

        # Predict step
        self.state = F @ self.state
        self.P = F @ self.P @ F.T + self.Q
        pred_x, pred_y = float(self.state[0, 0]), float(self.state[1, 0])

        if detected and meas is not None:
            self.consecutive_hits += 1
            self.consecutive_misses = 0
            if self.consecutive_hits >= 2:
                self.status = "TRACKING"
            else:
                self.status = "ACQUISITION"

            # Measurement update
            z = np.array([[meas[0]], [meas[1]]], dtype=np.float64)
            y = z - self.H @ self.state
            S = self.H @ self.P @ self.H.T + self.R
            K = self.P @ self.H.T @ np.linalg.inv(S)
            self.state = self.state + K @ y
            self.P = (np.eye(4) - K @ self.H) @ self.P
            return (float(self.state[0, 0]), float(self.state[1, 0])), self.status
        else:
            self.consecutive_misses += 1
            self.consecutive_hits = 0
            if self.consecutive_misses <= 8:
                self.status = "COASTING"
            else:
                self.status = "SEARCH"
                self.initialized = False
            return (pred_x, pred_y), self.status


def draw_hud(
    frame: np.ndarray,
    mask: np.ndarray,
    result: Dict[str, Any],
    tracked_pt: Tuple[float, float],
    tracking_state: str,
    fps: float,
    proc_ms: float,
    frame_id: int,
) -> np.ndarray:
    """Renders a sleek, modern, professional telemetry HUD overlay and PiP mask."""
    h, w = frame.shape[:2]
    vis = frame.copy()

    cx_center = w // 2
    cy_center = h // 2

    # 1. Optical Center Reticle (Target Boresight)
    reticle_color = (220, 220, 220)
    # Crosshair ticks
    cv2.line(vis, (cx_center - 18, cy_center), (cx_center + 18, cy_center), reticle_color, 1)
    cv2.line(vis, (cx_center, cy_center - 18), (cx_center, cy_center + 18), reticle_color, 1)
    cv2.circle(vis, (cx_center, cy_center), 12, reticle_color, 1)
    cv2.circle(vis, (cx_center, cy_center), 24, (120, 120, 120), 1)

    # 2. Target Annotation & Error Vector
    detected = result["detected"]
    pixel_err = 0.0

    if detected:
        bx = int(result["boundingBoxX"])
        by = int(result["boundingBoxY"])
        bw = int(result["boundingBoxWidth"])
        bh = int(result["boundingBoxHeight"])
        tx = int(round(result["pixelCenterX"]))
        ty = int(round(result["pixelCenterY"]))
        conf = result["confidence"]

        # Bounding box with corner accents
        box_color = (255, 230, 0) # Cyan in BGR
        cv2.rectangle(vis, (bx, by), (bx + bw, by + bh), box_color, 2)

        # Centroid crosshair
        cv2.drawMarker(vis, (tx, ty), (0, 255, 100), cv2.MARKER_CROSS, 16, 2)

        # Error vector line to optical center
        dx = tx - cx_center
        dy = ty - cy_center
        pixel_err = math.sqrt(dx * dx + dy * dy)
        err_color = (0, 255, 255) if pixel_err > 6.0 else (0, 255, 100) # Yellow if off, Green if aligned
        cv2.line(vis, (cx_center, cy_center), (tx, ty), err_color, 1, cv2.LINE_AA)

        # Label above target
        tag = f"BEACON #{frame_id} [Conf: {conf:.2f}]"
        cv2.putText(vis, tag, (bx, max(18, by - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
    elif tracking_state == "COASTING":
        # Draw predicted position marker
        px = int(round(tracked_pt[0]))
        py = int(round(tracked_pt[1]))
        if 0 <= px < w and 0 <= py < h:
            cv2.circle(vis, (px, py), 10, (0, 140, 255), 2) # Orange circle
            cv2.drawMarker(vis, (px, py), (0, 140, 255), cv2.MARKER_STAR, 12, 1)
            cv2.putText(vis, "KALMAN PREDICTED (COASTING)", (px + 12, py), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 165, 255), 1, cv2.LINE_AA)

    # 3. Top Telemetry Bar (Translucent Dark Background)
    bar_h = 42
    overlay = vis.copy()
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (12, 16, 24), -1)
    cv2.addWeighted(overlay, 0.78, vis, 0.22, 0, vis)
    cv2.line(vis, (0, bar_h), (w, bar_h), (0, 200, 255), 1)

    # State Badge Colors
    state_colors = {
        "TRACKING": (0, 255, 120),    # Emerald green
        "ACQUISITION": (0, 220, 255), # Yellow
        "COASTING": (0, 140, 255),    # Orange
        "SEARCH": (0, 80, 255),       # Red
    }
    badge_color = state_colors.get(tracking_state, (200, 200, 200))

    # Badge text
    cv2.putText(vis, f"OPTINAV M1-M5", (12, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, f"STATE: {tracking_state}", (12, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.48, badge_color, 2, cv2.LINE_AA)

    # Telemetry metrics
    aligned_text = "ALIGNED (LOCKED)" if (detected and pixel_err <= 6.0) else ("TRACKING" if detected else "UNALIGNED")
    aligned_color = (0, 255, 120) if (detected and pixel_err <= 6.0) else (180, 180, 180)
    cv2.putText(vis, f"STATUS: {aligned_text}", (180, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, aligned_color, 1, cv2.LINE_AA)
    cv2.putText(vis, f"ERR: {pixel_err:4.1f} px | FOV: 60.0 deg", (180, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

    cv2.putText(vis, f"FPS: {fps:4.1f} | PROC: {proc_ms:3.1f} ms", (420, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(vis, f"LATENCY: {result.get('latencyMs', 0):3.1f} ms", (420, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.44, (200, 200, 200), 1, cv2.LINE_AA)

    # 4. Picture-in-Picture (PiP) Binary Mask Inset (Lower-Right)
    pip_w, pip_h = 160, 90
    pip_x = w - pip_w - 10
    pip_y = h - pip_h - 10

    small_mask = cv2.resize(mask, (pip_w, pip_h))
    mask_bgr = cv2.cvtColor(small_mask, cv2.COLOR_GRAY2BGR)
    # Tint mask cyan
    mask_bgr[:, :, 0] = np.where(small_mask > 0, 255, 0)
    mask_bgr[:, :, 1] = np.where(small_mask > 0, 200, 0)
    mask_bgr[:, :, 2] = np.where(small_mask > 0, 0, 0)

    vis[pip_y:pip_y+pip_h, pip_x:pip_x+pip_w] = mask_bgr
    cv2.rectangle(vis, (pip_x, pip_y), (pip_x + pip_w, pip_y + pip_h), (255, 255, 255), 1)
    cv2.putText(vis, "M3 BINARY MASK", (pip_x + 4, pip_y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 255), 1, cv2.LINE_AA)

    return vis


def record_live_demo(
    output_dir: str = "Outputs",
    duration_sec: float = 18.0,
    host: str = "127.0.0.1",
    frame_port: int = 9001,
    result_port: int = 9002,
):
    os.makedirs(os.path.join(output_dir, "images"), exist_ok=True)
    video_path = os.path.join(output_dir, "optinav_tracking_stream.mp4")

    detector = OptinavBeaconDetector()
    sender = ResultSender(host=host, port=result_port)
    tracker = KalmanFilter2DTracker()

    print(f"[*] Connecting to Unity frame stream at {host}:{frame_port}...")
    frame_sock = None
    for attempt in range(15):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(2.0)
            s.connect((host, frame_port))
            frame_sock = s
            print(f"[+] Connected to Unity frame stream on port {frame_port}!")
            break
        except (ConnectionRefusedError, socket.timeout):
            time.sleep(0.5)

    if frame_sock is None:
        print("[!] ERROR: Could not connect to Unity stream. Is Play Mode running?")
        return False

    frame_sock.settimeout(4.0)

    # Initialize VideoWriter with avc1 or mp4v
    fourcc = cv2.VideoWriter_fourcc(*'avc1')
    writer = cv2.VideoWriter(video_path, fourcc, 25.0, (640, 360))
    if not writer.isOpened():
        print("[*] Falling back to mp4v codec...")
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        writer = cv2.VideoWriter(video_path, fourcc, 25.0, (640, 360))

    start_time = time.time()
    frame_count = 0
    fps_timer = time.time()
    current_fps = 25.0
    recent_proc_ms = 2.0

    captured_stationary = False
    captured_moving = False
    captured_disturbed = False
    captured_coasting = False
    captured_reacquired = False

    print(f"[*] Recording live tracking stream for ~{duration_sec}s into {video_path}...")

    try:
        while time.time() - start_time < duration_sec:
            # 1. Read header
            header_bytes = recv_all(frame_sock, HEADER_SIZE)
            if header_bytes is None:
                break
            magic, frame_id, capture_timestamp, width, height, payload_length = struct.unpack(
                HEADER_FORMAT, header_bytes
            )
            if magic != EXPECTED_MAGIC:
                break

            # 2. Read JPEG
            jpeg_bytes = recv_all(frame_sock, payload_length)
            if jpeg_bytes is None:
                break

            np_buf = np.frombuffer(jpeg_bytes, dtype=np.uint8)
            raw_frame = cv2.imdecode(np_buf, cv2.IMREAD_COLOR)
            if raw_frame is None:
                continue

            # 3. Detect
            t0 = time.perf_counter()
            result, candidates, mask = detector.process_frame(raw_frame, frame_id, capture_timestamp)
            recent_proc_ms = (time.perf_counter() - t0) * 1000.0

            # 4. Stream back to Unity M4 & M5
            sender.send_result(result)

            # 5. Tracker state
            meas = (result["pixelCenterX"], result["pixelCenterY"]) if result["detected"] else None
            tracked_pos, track_state = tracker.update(result["detected"], meas, dt=1.0/25.0)

            # 6. FPS
            frame_count += 1
            if time.time() - fps_timer >= 1.0:
                current_fps = frame_count / (time.time() - fps_timer)
                fps_timer = time.time()
                frame_count = 0

            # 7. Render HUD
            annotated = draw_hud(
                raw_frame, mask, result, tracked_pos, track_state,
                current_fps, recent_proc_ms, frame_id
            )

            # 8. Write to Video
            writer.write(annotated)

            # 9. Smart Screenshot Capture
            elapsed = time.time() - start_time
            dx = result["pixelCenterX"] - 320.0
            dy = result["pixelCenterY"] - 180.0
            px_err = math.sqrt(dx*dx + dy*dy) if result["detected"] else 999.0

            # Screenshot 1: Stationary locked (< 2 px error)
            if not captured_stationary and result["detected"] and px_err < 2.0 and elapsed > 2.0:
                img_path = os.path.join(output_dir, "images", "01_stationary_beacon_aligned.png")
                cv2.imwrite(img_path, annotated)
                captured_stationary = True
                print(f"[+] Saved screenshot: {img_path} (Err={px_err:.2f}px)")

            # Screenshot 2: Dynamic tracking (during moving beacon test, elapsed ~ 8-16s)
            if not captured_moving and result["detected"] and 10.0 < elapsed < 16.0:
                img_path = os.path.join(output_dir, "images", "02_moving_beacon_tracking.png")
                cv2.imwrite(img_path, annotated)
                captured_moving = True
                print(f"[+] Saved screenshot: {img_path} (Elapsed={elapsed:.1f}s)")

            # Screenshot 3: Disturbance rejection (during disturbance test, elapsed ~ 19-25s)
            if not captured_disturbed and result["detected"] and 19.0 < elapsed < 25.0:
                img_path = os.path.join(output_dir, "images", "03_disturbance_rejection.png")
                cv2.imwrite(img_path, annotated)
                captured_disturbed = True
                print(f"[+] Saved screenshot: {img_path} (Elapsed={elapsed:.1f}s)")

            # Screenshot 4: Coasting (during dropout test, elapsed ~ 26-31s)
            if not captured_coasting and track_state == "COASTING" and elapsed > 25.0:
                img_path = os.path.join(output_dir, "images", "04_dropout_and_coasting.png")
                cv2.imwrite(img_path, annotated)
                captured_coasting = True
                print(f"[+] Saved screenshot: {img_path} (Elapsed={elapsed:.1f}s)")

            # Screenshot 5: Reacquisition (elapsed ~ 31-36s)
            if captured_coasting and not captured_reacquired and result["detected"] and elapsed > 31.0:
                img_path = os.path.join(output_dir, "images", "05_reacquisition_lock.png")
                cv2.imwrite(img_path, annotated)
                captured_reacquired = True
                print(f"[+] Saved screenshot: {img_path} (Elapsed={elapsed:.1f}s)")

    finally:
        writer.release()
        frame_sock.close()
        print(f"[+] Video recording saved: {video_path} ({os.path.getsize(video_path)} bytes)")

    return True


if __name__ == "__main__":
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 15.0
    record_live_demo(duration_sec=dur)
