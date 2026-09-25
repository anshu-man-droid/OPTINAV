#!/usr/bin/env python3
"""
OPTINAV — Member 3 Optical Beacon Detector
Connects to the Unity camera frame stream over TCP (127.0.0.1:9001),
executes deterministic classical computer-vision segmentation (HSV + morphology + contours),
filters and scores candidate optical beacons, and streams JSON detection results back
to the Unity Detection Bridge (127.0.0.1:9002).

STRICT SCOPE BOUNDARY (M3):
- No Kalman filtering
- No prediction / extrapolation
- No PID or gimbal control
- No neural networks (YOLO, ONNX, PyTorch)
- When beacon is absent/dropped out, reports detected=False, pixelCenter=(-1, -1), confidence=0.0
"""

import sys
import os
import time
import socket
import struct
import json
import math
import argparse
from typing import Optional, Tuple, Dict, Any, List
import numpy as np
import cv2

# ==============================================================================
# CONFIGURABLE THRESHOLDS & OPTICAL PARAMETERS
# ==============================================================================
# In OpenCV HSV:
#   Hue is in [0, 179] (Cyan / teal optical emission is centered around H ~ 88-92)
#   Saturation is in [0, 255] (Emissive beacon core has high saturation)
#   Value is in [0, 255] (Emissive beacon has high brightness)
# Orange distractor has Hue ~ 13-15 and is completely rejected by the Hue band.
# Sky background has Hue ~ 106-109 and is excluded by Hue Upper <= 96.
# Background mountain has Hue ~ 97-105 and Sat < 65.
BEACON_HUE_LOWER = 82       # Minimum hue for cyan/teal beacon
BEACON_HUE_UPPER = 96       # Maximum hue for cyan/teal beacon (excludes mountain & sky)
SATURATION_MIN = 60         # Minimum saturation (handles sensor noise & slight blur)
SATURATION_MAX = 255        # Maximum saturation
VALUE_MIN = 75              # Minimum optical value / brightness
VALUE_MAX = 255             # Maximum optical value / brightness

BEACON_HSV_LOWER = np.array([BEACON_HUE_LOWER, SATURATION_MIN, VALUE_MIN], dtype=np.uint8)
BEACON_HSV_UPPER = np.array([BEACON_HUE_UPPER, SATURATION_MAX, VALUE_MAX], dtype=np.uint8)

# Geometric & Contour Filtering Limits
MIN_CONTOUR_AREA = 4.0      # Minimum area in pixels to reject sub-pixel noise spikes
MAX_CONTOUR_AREA = 2500.0   # Maximum area in pixels to reject large background structures
MIN_WIDTH = 2               # Minimum candidate bounding-box width
MAX_WIDTH = 120             # Maximum candidate bounding-box width
MIN_HEIGHT = 2              # Minimum candidate bounding-box height
MAX_HEIGHT = 120            # Maximum candidate bounding-box height
MIN_CIRCULARITY = 0.20      # Minimum 4*pi*A / P^2 circularity (beacon sphere is ~0.8-1.0)
MIN_ASPECT_RATIO = 0.35     # Minimum width/height ratio
MAX_ASPECT_RATIO = 2.8      # Maximum width/height ratio

# Morphological Filtering
MORPH_KERNEL_SIZE = 3       # Structuring element size (3x3 ellipse)
MORPH_OPEN_ITERS = 1        # Opening iterations to eliminate isolated noise spikes
MORPH_CLOSE_ITERS = 1       # Closing iterations to bridge optical blur gaps

# Candidate Scoring & Confidence
MIN_CONFIDENCE_THRESHOLD = 0.30  # Minimum confidence required to accept a detection
NOMINAL_BEACON_HUE = 90.0        # Expected center hue for primary FSOC beacon

# Sensor Frame Geometry
FRAME_WIDTH = 640
FRAME_HEIGHT = 360

# Binary Network Header Protocol (from OPTINAVFrameCapture.cs)
# Format: 4s (magic "OPT1"), I (frameId), d (timestamp), H (width), H (height), I (payloadLen)
HEADER_FORMAT = "!4sIdHHI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
EXPECTED_MAGIC = b"OPT1"


# ==============================================================================
# NETWORK HELPERS
# ==============================================================================
def recv_all(sock: socket.socket, num_bytes: int) -> Optional[bytes]:
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


class ResultSender:
    """Thread-safe / non-blocking TCP client to send detection results back to Unity (port 9002)."""

    def __init__(self, host: str = "127.0.0.1", port: int = 9002):
        self.host = host
        self.port = port
        self.sock: Optional[socket.socket] = None
        self.connected = False
        self.last_attempt_time = 0.0

    def connect(self) -> bool:
        if self.connected and self.sock is not None:
            return True

        now = time.time()
        if now - self.last_attempt_time < 1.0:
            return False
        self.last_attempt_time = now

        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(0.5)
            s.connect((self.host, self.port))
            s.settimeout(None)
            self.sock = s
            self.connected = True
            print(f"[ResultSender] Connected to Unity Detection Bridge at {self.host}:{self.port}")
            return True
        except (ConnectionRefusedError, socket.timeout, OSError):
            self.connected = False
            self.sock = None
            return False

    def send_result(self, result_dict: Dict[str, Any]) -> bool:
        """Encodes result to compact JSON and sends with a newline delimiter."""
        if not self.connect():
            return False

        try:
            json_str = json.dumps(result_dict, separators=(",", ":")) + "\n"
            data = json_str.encode("utf-8")
            self.sock.sendall(data)
            return True
        except (BrokenPipeError, ConnectionResetError, OSError):
            self.connected = False
            if self.sock:
                try:
                    self.sock.close()
                except Exception:
                    pass
            self.sock = None
            return False

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None
        self.connected = False


# ==============================================================================
# COMPUTER VISION DETECTION ENGINE
# ==============================================================================
class OptinavBeaconDetector:
    """
    Classical deterministic computer vision detector for FSOC optical beacon.
    Extracts candidates via HSV segmentation, morphology, and contour analysis,
    and computes confidence based on color match, circularity, aspect ratio, and intensity.
    """

    def __init__(self):
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE)
        )

    def process_frame(
        self,
        bgr_image: np.ndarray,
        frame_id: int,
        capture_timestamp: float,
    ) -> Tuple[Dict[str, Any], List[Dict[str, Any]], np.ndarray]:
        """
        Executes the beacon detection pipeline on a single BGR frame.
        Returns:
            result_dict: DetectionResult matching project specification
            all_candidates: List of evaluated candidates
            debug_mask: Final binary mask for inspection/visualization
        """
        det_start_time = time.time()
        actual_h, actual_w = bgr_image.shape[:2]

        # 1. Convert BGR to HSV color space
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 2. Threshold image using beacon HSV range (Cyan/Teal)
        color_mask = cv2.inRange(hsv, BEACON_HSV_LOWER, BEACON_HSV_UPPER)

        # 3. Morphological filtering (Opening removes noise, Closing fills blur holes)
        cleaned_mask = cv2.morphologyEx(
            color_mask, cv2.MORPH_OPEN, self.morph_kernel, iterations=MORPH_OPEN_ITERS
        )
        cleaned_mask = cv2.morphologyEx(
            cleaned_mask, cv2.MORPH_CLOSE, self.morph_kernel, iterations=MORPH_CLOSE_ITERS
        )

        # 4. Contour extraction
        contours, _ = cv2.findContours(
            cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        # 5. Candidate filtering & scoring
        valid_candidates: List[Dict[str, Any]] = []

        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not (MIN_CONTOUR_AREA <= area <= MAX_CONTOUR_AREA):
                continue

            perim = float(cv2.arcLength(contour, True))
            if perim <= 0.0:
                continue

            bx, by, bw, bh = cv2.boundingRect(contour)
            if not (MIN_WIDTH <= bw <= MAX_WIDTH and MIN_HEIGHT <= bh <= MAX_HEIGHT):
                continue

            aspect_ratio = float(bw) / float(bh)
            if not (MIN_ASPECT_RATIO <= aspect_ratio <= MAX_ASPECT_RATIO):
                continue

            # Circularity: 4 * pi * Area / Perimeter^2
            circularity = (4.0 * math.pi * area) / (perim * perim)
            circularity = min(1.0, max(0.0, circularity))
            if circularity < MIN_CIRCULARITY:
                continue

            # Candidate mask to sample pixel statistics inside the contour
            cand_mask = np.zeros(cleaned_mask.shape, dtype=np.uint8)
            cv2.drawContours(cand_mask, [contour], -1, 255, -1)
            pts_hsv = hsv[cand_mask > 0]
            if len(pts_hsv) == 0:
                continue

            mean_h = float(np.mean(pts_hsv[:, 0]))
            mean_s = float(np.mean(pts_hsv[:, 1]))
            mean_v = float(np.mean(pts_hsv[:, 2]))
            max_v = int(np.max(pts_hsv[:, 2]))

            # Centroid via spatial moments
            moments = cv2.moments(contour)
            if moments["m00"] > 0:
                cx = float(moments["m10"] / moments["m00"])
                cy = float(moments["m01"] / moments["m00"])
            else:
                cx = float(bx + bw / 2.0)
                cy = float(by + bh / 2.0)

            # Spot radius approximation
            spot_radius = float(math.sqrt(area / math.pi)) if area > 0 else float(bw + bh) / 4.0

            # 6. Candidate Confidence Scoring
            # - Hue centering score: optimal cyan hue is 90
            hue_dev = abs(mean_h - NOMINAL_BEACON_HUE) / 10.0
            s_hue = max(0.0, 1.0 - hue_dev)

            # - Saturation score: higher saturation indicates pure optical emission
            s_sat = min(1.0, mean_s / 180.0)

            # - Value score: emissive beacon has high brightness
            s_val = min(1.0, mean_v / 200.0)

            # - Circularity score: spherical beacon approaches 0.85-1.0
            s_circ = min(1.0, circularity / 0.85)

            # - Aspect ratio score: nominal circle has aspect ratio 1.0
            s_ar = max(0.0, 1.0 - abs(aspect_ratio - 1.0) / 0.8)

            # - Size suitability score: nominal target range is 15-500 px
            s_size = 1.0 if (15.0 <= area <= 500.0) else (0.7 if area >= MIN_CONTOUR_AREA else 0.4)

            # Composite weighted confidence
            confidence = (
                0.25 * s_hue
                + 0.20 * s_sat
                + 0.20 * s_val
                + 0.15 * s_circ
                + 0.10 * s_ar
                + 0.10 * s_size
            )
            confidence = float(np.clip(confidence, 0.0, 1.0))

            if confidence >= MIN_CONFIDENCE_THRESHOLD:
                cand_info = {
                    "contour": contour,
                    "area": area,
                    "perimeter": perim,
                    "circularity": circularity,
                    "aspectRatio": aspect_ratio,
                    "bbox": (bx, by, bw, bh),
                    "centroid": (cx, cy),
                    "spotRadius": spot_radius,
                    "meanHue": mean_h,
                    "meanSat": mean_s,
                    "meanVal": mean_v,
                    "maxVal": max_v,
                    "confidence": confidence,
                }
                valid_candidates.append(cand_info)

        # 7. Beacon Candidate Selection (Highest scoring valid candidate)
        valid_candidates.sort(key=lambda k: k["confidence"], reverse=True)

        det_end_time = time.time()
        latency_ms = (det_end_time - capture_timestamp) * 1000.0 if capture_timestamp > 0 else 0.0

        if valid_candidates:
            best = valid_candidates[0]
            cx, cy = best["centroid"]
            bx, by, bw, bh = best["bbox"]
            conf = best["confidence"]
            spot_r = best["spotRadius"]

            norm_x = cx / float(actual_w)
            norm_y = cy / float(actual_h)

            result = {
                "frameId": frame_id,
                "detected": True,
                "pixelCenterX": round(cx, 2),
                "pixelCenterY": round(cy, 2),
                "normalizedCenterX": round(norm_x, 4),
                "normalizedCenterY": round(norm_y, 4),
                "boundingBoxX": float(bx),
                "boundingBoxY": float(by),
                "boundingBoxWidth": float(bw),
                "boundingBoxHeight": float(bh),
                "confidence": round(conf, 3),
                "latencyMs": round(latency_ms, 2),
                "captureTimestamp": capture_timestamp,
                "detectionTimestamp": det_end_time,
                "spotRadius": round(spot_r, 2),
            }
        else:
            # DROPOUT / NOT DETECTED: Return explicit non-detected state with NO stale coordinates
            result = {
                "frameId": frame_id,
                "detected": False,
                "pixelCenterX": -1.0,
                "pixelCenterY": -1.0,
                "normalizedCenterX": -1.0,
                "normalizedCenterY": -1.0,
                "boundingBoxX": 0.0,
                "boundingBoxY": 0.0,
                "boundingBoxWidth": 0.0,
                "boundingBoxHeight": 0.0,
                "confidence": 0.0,
                "latencyMs": round(latency_ms, 2),
                "captureTimestamp": capture_timestamp,
                "detectionTimestamp": det_end_time,
                "spotRadius": 0.0,
            }

        return result, valid_candidates, cleaned_mask


# ==============================================================================
# MAIN RECEIVER & DETECTOR RUNNER
# ==============================================================================
def run_detector(
    host: str = "127.0.0.1",
    frame_port: int = 9001,
    result_port: int = 9002,
    show_gui: bool = True,
    max_frames: int = 0,
):
    print("=" * 70)
    print(" OPTINAV M3 — COMPUTER VISION BEACON DETECTOR ")
    print(f" Frame Stream:     tcp://{host}:{frame_port} (Input)")
    print(f" Detection Bridge: tcp://{host}:{result_port} (Output JSON)")
    print(f" HSV Filter:       H:[{BEACON_HUE_LOWER}, {BEACON_HUE_UPPER}], S:[{SATURATION_MIN}, {SATURATION_MAX}], V:[{VALUE_MIN}, {VALUE_MAX}]")
    print(f" Confidence Min:   {MIN_CONFIDENCE_THRESHOLD}")
    print("=" * 70)

    detector = OptinavBeaconDetector()
    sender = ResultSender(host=host, port=result_port)

    window_name = "OPTINAV M3 — Beacon Detector Output"
    if show_gui:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 960, 540)

    running = True
    total_processed_frames = 0
    total_detections = 0
    fps_frame_count = 0
    fps_start_time = time.time()
    current_fps = 0.0
    last_log_time = 0.0

    recent_proc_times = []
    recent_latencies = []
    recent_confidences = []

    while running:
        frame_sock = None
        print(f"[*] Connecting to Unity frame stream at {host}:{frame_port}...")
        while running and frame_sock is None:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((host, frame_port))
                frame_sock = s
                print(f"[+] Connected to Unity frame stream on port {frame_port}!")
            except (ConnectionRefusedError, socket.timeout):
                time.sleep(0.5)
            except KeyboardInterrupt:
                running = False
                break

        if not running or frame_sock is None:
            break

        frame_sock.settimeout(5.0)

        try:
            while running:
                # 1. Read header
                header_bytes = recv_all(frame_sock, HEADER_SIZE)
                if header_bytes is None:
                    print("[!] Unity frame stream closed.")
                    break

                magic, frame_id, capture_timestamp, width, height, payload_length = struct.unpack(
                    HEADER_FORMAT, header_bytes
                )

                if magic != EXPECTED_MAGIC or payload_length == 0 or payload_length > 20_000_000:
                    print(f"[!] Invalid frame header or payload ({payload_length} bytes). Dropping.")
                    break

                # 2. Read JPEG payload
                jpeg_bytes = recv_all(frame_sock, payload_length)
                if jpeg_bytes is None:
                    print("[!] Incomplete frame payload.")
                    break

                # 3. Decode JPEG
                np_buffer = np.frombuffer(jpeg_bytes, dtype=np.uint8)
                frame = cv2.imdecode(np_buffer, cv2.IMREAD_COLOR)
                if frame is None or frame.size == 0:
                    continue

                # 4. Execute Beacon Detection Pipeline
                t_proc_start = time.perf_counter()
                result, candidates, mask = detector.process_frame(
                    frame, frame_id, capture_timestamp
                )
                t_proc_ms = (time.perf_counter() - t_proc_start) * 1000.0

                total_processed_frames += 1
                fps_frame_count += 1
                recent_proc_times.append(t_proc_ms)
                if len(recent_proc_times) > 100:
                    recent_proc_times.pop(0)

                if result["detected"]:
                    total_detections += 1
                    recent_confidences.append(result["confidence"])
                    if len(recent_confidences) > 100:
                        recent_confidences.pop(0)

                if result["latencyMs"] > 0:
                    recent_latencies.append(result["latencyMs"])
                    if len(recent_latencies) > 100:
                        recent_latencies.pop(0)

                # 5. Send detection result back to Unity
                sender.send_result(result)

                # 6. FPS and periodic logging
                now = time.time()
                if now - fps_start_time >= 1.0:
                    current_fps = fps_frame_count / (now - fps_start_time)
                    fps_frame_count = 0
                    fps_start_time = now

                if now - last_log_time >= 1.0:
                    avg_proc = np.mean(recent_proc_times) if recent_proc_times else 0.0
                    avg_lat = np.mean(recent_latencies) if recent_latencies else 0.0
                    det_rate = (total_detections / total_processed_frames * 100.0) if total_processed_frames > 0 else 0.0
                    status_tag = f"DETECTED (conf={result['confidence']:.2f}, pt=({result['pixelCenterX']:.1f}, {result['pixelCenterY']:.1f}))" if result["detected"] else "NOT DETECTED"

                    print(
                        f"[DETECTOR] Frame #{frame_id:5d} | {status_tag} | "
                        f"Proc: {avg_proc:4.1f} ms | Det FPS: {current_fps:4.1f} | Latency: {avg_lat:4.1f} ms | "
                        f"Det Rate: {det_rate:4.1f}%"
                    )
                    last_log_time = now

                # 7. Visualization
                if show_gui:
                    vis_frame = frame.copy()
                    h, w = vis_frame.shape[:2]

                    if result["detected"]:
                        bx = int(result["boundingBoxX"])
                        by = int(result["boundingBoxY"])
                        bw = int(result["boundingBoxWidth"])
                        bh = int(result["boundingBoxHeight"])
                        cx = int(round(result["pixelCenterX"]))
                        cy = int(round(result["pixelCenterY"]))

                        # Draw bounding box (Cyan)
                        cv2.rectangle(vis_frame, (bx, by), (bx + bw, by + bh), (255, 255, 0), 2)
                        # Draw crosshair at centroid
                        cv2.drawMarker(vis_frame, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)
                        # Label
                        label = f"BEACON [conf: {result['confidence']:.2f}]"
                        cv2.putText(
                            vis_frame,
                            label,
                            (bx, max(18, by - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (255, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )
                    else:
                        # Draw NOT DETECTED banner
                        cv2.putText(
                            vis_frame,
                            "TARGET NOT DETECTED",
                            (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.65,
                            (0, 0, 255),
                            2,
                            cv2.LINE_AA,
                        )

                    # Top HUD overlay
                    hud_text = (
                        f"OPTINAV Frame #{frame_id} | Det: {'YES' if result['detected'] else 'NO'} | "
                        f"FPS: {current_fps:.1f} | Proc: {t_proc_ms:.1f}ms | Latency: {result['latencyMs']:.1f}ms"
                    )
                    cv2.putText(
                        vis_frame,
                        hud_text,
                        (10, 22),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.52,
                        (0, 255, 0) if result["detected"] else (0, 165, 255),
                        1,
                        cv2.LINE_AA,
                    )

                    cv2.imshow(window_name, vis_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == 27 or key == ord("q"):
                        print("[*] Quit requested via OpenCV window.")
                        running = False
                        break

                if max_frames > 0 and total_processed_frames >= max_frames:
                    print(f"[*] Reached target frame count: {max_frames}. Exiting.")
                    running = False
                    break

        except (socket.timeout, ConnectionResetError, BrokenPipeError) as e:
            print(f"[!] Frame stream error: {e}")
        except KeyboardInterrupt:
            print("\n[*] Ctrl+C received, shutting down cleanly...")
            running = False
        finally:
            if frame_sock:
                try:
                    frame_sock.close()
                except Exception:
                    pass

    if show_gui:
        cv2.destroyAllWindows()

    sender.close()

    print("=" * 70)
    print(f"[*] Detector shutdown. Total Frames: {total_processed_frames} | Total Detections: {total_detections}")
    if total_processed_frames > 0:
        det_rate = (total_detections / total_processed_frames) * 100.0
        print(f"[*] Detection Rate: {det_rate:.2f}%")
    if recent_proc_times:
        print(f"[*] Processing Time: Avg {np.mean(recent_proc_times):.2f} ms | Max {np.max(recent_proc_times):.2f} ms")
    if recent_latencies:
        print(f"[*] Latency: Avg {np.mean(recent_latencies):.2f} ms")
    if recent_confidences:
        print(f"[*] Avg Confidence: {np.mean(recent_confidences):.3f}")
    print("=" * 70)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="OPTINAV Member 3 Optical Beacon Detector")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="Host IP (default: 127.0.0.1)")
    parser.add_argument("--frame-port", type=int, default=9001, help="Port to receive frames from Unity (default: 9001)")
    parser.add_argument("--result-port", type=int, default=9002, help="Port to send results to Unity (default: 9002)")
    parser.add_argument("--no-gui", action="store_true", help="Run in headless mode without cv2 window")
    parser.add_argument("--max-frames", type=int, default=0, help="Exit after N frames (0 = run indefinitely)")
    args = parser.parse_args()

    try:
        run_detector(
            host=args.host,
            frame_port=args.frame_port,
            result_port=args.result_port,
            show_gui=not args.no_gui,
            max_frames=args.max_frames,
        )
    except KeyboardInterrupt:
        print("\n[*] Exiting.")
        sys.exit(0)
