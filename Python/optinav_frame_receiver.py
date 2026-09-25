#!/usr/bin/env python3
"""
OPTINAV - Member 3 Optical Frame Receiver (Verification Testbed)
Receives post-processed camera frames from Unity over localhost TCP.
Decodes JPEG frames with cv2.imdecode and displays them with live telemetry.

DO NOT implement beacon detection, Kalman filtering, or PID control in this script.
This script is solely for proving that the Unity camera frame with disturbances
reaches Python cleanly, correctly, and efficiently.
"""

import sys
import os
import time
import socket
import struct
import argparse
import numpy as np
import cv2

HEADER_FORMAT = "!4sIdHHI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
EXPECTED_MAGIC = b"OPT1"


def recv_all(sock: socket.socket, num_bytes: int) -> bytes | None:
    """Reads exactly num_bytes from a socket, or returns None on EOF/disconnect."""
    buffer = bytearray(num_bytes)
    view = memoryview(buffer)
    received = 0
    while received < num_bytes:
        try:
            n = sock.recv_into(view[received:], num_bytes - received)
        except (ConnectionResetError, ConnectionAbortedError, TimeoutError):
            return None
        if n == 0:
            return None
        received += n
    return bytes(buffer)


def analyze_image_disturbances(image: np.ndarray) -> dict:
    """Computes quantitative metrics to verify optical blur, noise, and visibility."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    mean_val = float(np.mean(gray))
    std_val = float(np.std(gray))
    max_val = int(np.max(gray))
    # Laplacian variance is a standard measure of optical sharpness / blur
    laplacian_var = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    return {
        "mean_luminance": mean_val,
        "std_dev": std_val,
        "max_intensity": max_val,
        "sharpness_laplacian": laplacian_var,
    }


def run_receiver(
    host: str = "127.0.0.1",
    port: int = 9001,
    show_gui: bool = True,
    save_snapshots: bool = True,
    snapshot_dir: str = "Python/test_snapshots",
    max_frames: int = 0,
):
    print("=" * 65)
    print(" OPTINAV M3 — LOCALHOST FRAME RECEIVER ")
    print(f" Target: tcp://{host}:{port}")
    print(f" Header format: {HEADER_FORMAT} ({HEADER_SIZE} bytes)")
    print("=" * 65)

    if save_snapshots:
        os.makedirs(snapshot_dir, exist_ok=True)

    window_name = "OPTINAV M3 Optical Stream"
    if show_gui:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 540)

    running = True

    # Performance telemetry state
    total_received_frames = 0
    fps_frame_count = 0
    fps_start_time = time.time()
    current_fps = 0.0
    last_log_time = 0.0

    # Latency tracking
    recent_latencies = []

    # Automatic test snapshot checkpoints (frame_id -> test_label)
    snapshot_targets = {
        120: "test_a_clean",
        240: "test_b_blur",
        360: "test_c_noise",
        480: "test_d_haze",
        600: "test_e_visibility_loss",
        720: "test_f_combined",
        840: "test_g_motion",
    }
    saved_snapshots = set()

    while running:
        sock = None
        print(f"[*] Attempting to connect to Unity at {host}:{port}...")
        while running and sock is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((host, port))
                sock = s
                print(f"[+] Connected to Unity frame streamer on port {port}!")
            except (ConnectionRefusedError, socket.timeout):
                time.sleep(0.5)
            except KeyboardInterrupt:
                running = False
                break

        if not running or sock is None:
            break

        sock.settimeout(5.0)

        try:
            while running:
                # 1. Read the 24-byte frame header
                header_bytes = recv_all(sock, HEADER_SIZE)
                if header_bytes is None:
                    print("[!] Unity streamer disconnected or socket closed.")
                    break

                recv_timestamp = time.time()
                magic, frame_id, capture_timestamp, width, height, payload_length = struct.unpack(
                    HEADER_FORMAT, header_bytes
                )

                if magic != EXPECTED_MAGIC:
                    print(f"[!] Warning: Invalid magic header received: {magic!r}. Dropping sync.")
                    break

                if payload_length == 0 or payload_length > 20_000_000:
                    print(f"[!] Warning: Abnormal payload length: {payload_length} bytes. Dropping connection.")
                    break

                # 2. Read exactly the advertised JPEG payload
                jpeg_bytes = recv_all(sock, payload_length)
                if jpeg_bytes is None:
                    print("[!] Incomplete frame payload received.")
                    break

                # 3. Decode JPEG using cv2.imdecode
                np_buffer = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_buffer, cv2.IMREAD_COLOR)

                if frame is None or frame.size == 0:
                    print(f"[!] Failed to decode JPEG for frame {frame_id}")
                    continue

                actual_h, actual_w = frame.shape[:2]

                # 4. Telemetry calculations
                total_received_frames += 1
                fps_frame_count += 1
                now = time.time()
                if now - fps_start_time >= 1.0:
                    current_fps = fps_frame_count / (now - fps_start_time)
                    fps_frame_count = 0
                    fps_start_time = now

                transport_latency_ms = (recv_timestamp - capture_timestamp) * 1000.0
                if 0.0 <= transport_latency_ms <= 5000.0:
                    recent_latencies.append(transport_latency_ms)
                    if len(recent_latencies) > 60:
                        recent_latencies.pop(0)
                else:
                    transport_latency_ms = 0.0

                avg_latency = np.mean(recent_latencies) if recent_latencies else 0.0

                # Print telemetry once per second
                if now - last_log_time >= 1.0:
                    print(
                        f"[STREAM] Frame: {frame_id:6d} | Res: {actual_w}x{actual_h} | "
                        f"JPEG: {payload_length / 1024:5.1f} KB | "
                        f"Receiver FPS: {current_fps:5.1f} | Latency: {transport_latency_ms:5.1f} ms (avg {avg_latency:4.1f} ms)"
                    )
                    last_log_time = now

                # 5. Automated snapshot verification for Task 7
                if save_snapshots:
                    for target_fid, label in snapshot_targets.items():
                        if frame_id >= target_fid and label not in saved_snapshots:
                            saved_snapshots.add(label)
                            snap_path = os.path.join(snapshot_dir, f"{label}.jpg")
                            cv2.imwrite(snap_path, frame)
                            metrics = analyze_image_disturbances(frame)
                            print(
                                f"\n>>> [SNAPSHOT SAVED] {snap_path} (Frame {frame_id}) <<<\n"
                                f"    Mean Luminance: {metrics['mean_luminance']:.2f} | "
                                f"Std Dev (Noise): {metrics['std_dev']:.2f} | "
                                f"Sharpness (Laplacian): {metrics['sharpness_laplacian']:.1f} | "
                                f"Max Intensity: {metrics['max_intensity']}\n"
                            )

                # 6. Display frame with telemetry overlay
                if show_gui:
                    display_frame = frame.copy()
                    overlay_text = (
                        f"OPTINAV Frame #{frame_id} | {actual_w}x{actual_h} | "
                        f"FPS: {current_fps:.1f} | Latency: {transport_latency_ms:.1f}ms"
                    )
                    cv2.putText(
                        display_frame,
                        overlay_text,
                        (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.55,
                        (0, 255, 0),
                        1,
                        cv2.LINE_AA,
                    )

                    cv2.imshow(window_name, display_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27 or key == ord("q"):  # ESC or q
                        print("[*] User quit requested via OpenCV window.")
                        running = False
                        break
                    elif key == ord("s"):
                        manual_snap = os.path.join(snapshot_dir, f"manual_frame_{frame_id}.jpg")
                        cv2.imwrite(manual_snap, frame)
                        print(f"[*] Manual snapshot saved: {manual_snap}")

                if max_frames > 0 and total_received_frames >= max_frames:
                    print(f"[*] Reached target frame count: {max_frames}. Finishing test.")
                    running = False
                    break

        except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
            print(f"[!] Connection error: {e}")
        except KeyboardInterrupt:
            print("\n[*] Ctrl+C received, shutting down cleanly...")
            running = False
        finally:
            try:
                sock.close()
            except Exception:
                pass

    if show_gui:
        cv2.destroyAllWindows()

    print("=" * 65)
    print(f"[*] Receiver shut down cleanly. Total frames processed: {total_received_frames}")
    if recent_latencies:
        print(f"[*] Average Transport Latency: {np.mean(recent_latencies):.2f} ms (min: {np.min(recent_latencies):.2f} ms, max: {np.max(recent_latencies):.2f} ms)")
    print("=" * 65)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OPTINAV Optical Frame Receiver (Member 3)")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host to connect to (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=9001, help="Port to connect to (default: 9001)")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless telemetry-only mode")
    parser.add_argument("--no-snapshots", action="store_true", help="Disable automatic test snapshots")
    parser.add_argument("--max-frames", type=int, default=0, help="Exit after receiving N frames (0 = run indefinitely)")
    args = parser.parse_args()

    try:
        run_receiver(
            host=args.host,
            port=args.port,
            show_gui=not args.no_gui,
            save_snapshots=not args.no_snapshots,
            max_frames=args.max_frames,
        )
    except KeyboardInterrupt:
        print("\n[*] Exiting.")
        sys.exit(0)
