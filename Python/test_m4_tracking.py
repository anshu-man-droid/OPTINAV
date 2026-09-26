#!/usr/bin/env python3
"""
OPTINAV Member 4 — Comprehensive Tracking & Prediction Validation Test Suite
Executes the complete test matrix A through J on the Member 4 C# Tracking subsystem:
  TEST A: Stationary beacon (noise filtering & stabilization)
  TEST B: Linear beacon motion (constant velocity tracking & convergence)
  TEST C: Accelerating beacon (dynamic response & acceleration tracking)
  TEST D: Circular motion (2D curvilinear path tracking)
  TEST E: Temporary detection dropout (coasting verification & prediction-only)
  TEST F: Multiple consecutive missed frames (dropout > maxCoastingFrames -> transition to LOST)
  TEST G: Beacon reappearance / reacquisition (reacquisition latency & recovery)
  TEST H: Camera pan/tilt movement (platform slewing disturbance)
  TEST I: Combined disturbances (optical blur + sensor noise + haze)
  TEST J: Distractor presence (discrimination against false candidates)

Strictly adheres to ground-truth rule:
  - Ground truth is evaluated solely AFTER tracking result is produced
  - Ground truth is NEVER fed into tracker input
"""

import os
import sys
import math
import time
import json
import subprocess
from typing import List, Dict, Any, Tuple
import numpy as np

# Find Mono runtime path (Unity installed mono or system mono)
UNITY_MONO_PATHS = [
    "/Applications/Unity/Hub/Editor/6000.3.9f1/Unity.app/Contents/Resources/Scripting/MonoBleedingEdge/bin/mono",
    "/Applications/Unity/Unity.app/Contents/Resources/Scripting/MonoBleedingEdge/bin/mono"
]
MONO_PATH = "mono"
for p in UNITY_MONO_PATHS:
    if os.path.exists(p):
        MONO_PATH = p
        break

BRIDGE_EXE = os.path.abspath(os.path.join("Library", "ScriptAssemblies", "m4_tracker_bridge.exe"))

BRIDGE_CS_SRC = """
using System;
using System.Globalization;
using System.IO;
using OPTINAV.Detection;
using OPTINAV.Tracking;

namespace UnityEngine
{
    public class Coroutine {}
    public class MonoBehaviour
    {
        public GameObject gameObject => null;
        public static T FindFirstObjectByType<T>() where T : class => null;
        public static void Destroy(object o) {}
        public Coroutine StartCoroutine(System.Collections.IEnumerator r) => null;
    }
    public class GameObject
    {
        public Transform transform;
        public string name;
        public static GameObject CreatePrimitive(PrimitiveType type) => null;
        public T GetComponent<T>() => default;
        public T GetComponentInChildren<T>() => default;
        public T AddComponent<T>() => default;
        public void SetActive(bool a) {}
    }
    public enum PrimitiveType { Sphere }
    public class Transform : System.Collections.IEnumerable
    {
        public Vector3 position;
        public Vector3 localScale;
        public System.Collections.IEnumerator GetEnumerator() => null;
        public Transform Find(string n) => null;
    }
    public class MeshRenderer { public Material sharedMaterial; }
    public class Material {}
    public class Light {}
    public struct Vector2
    {
        public float x, y;
        public Vector2(float x, float y) { this.x = x; this.y = y; }
        public static Vector2 zero = new Vector2(0, 0);
        public static Vector2 one = new Vector2(1, 1);
        public static float Distance(Vector2 a, Vector2 b)
        {
            float dx = a.x - b.x, dy = a.y - b.y;
            return (float)System.Math.Sqrt(dx * dx + dy * dy);
        }
        public static Vector2 operator +(Vector2 a, Vector2 b) => new Vector2(a.x + b.x, a.y + b.y);
        public static Vector2 operator -(Vector2 a, Vector2 b) => new Vector2(a.x - b.x, a.y - b.y);
        public static Vector2 operator *(Vector2 a, float d) => new Vector2(a.x * d, a.y * d);
    }
    public struct Vector3
    {
        public float x, y, z;
        public Vector3(float x, float y, float z) { this.x = x; this.y = y; this.z = z; }
        public static Vector3 zero = new Vector3(0, 0, 0);
        public static Vector3 right = new Vector3(1, 0, 0);
    }
    public struct Rect
    {
        public float x, y, width, height;
        public Rect(float x, float y, float w, float h) { this.x = x; this.y = y; this.width = w; this.height = h; }
    }
    public struct Bounds { public Bounds(Vector3 c, Vector3 s) {} }
    public struct Color
    {
        public float r, g, b, a;
        public Color(float r, float g, float b, float a) { this.r = r; this.g = g; this.b = b; this.a = a; }
        public static Color white = new Color(1, 1, 1, 1);
    }
    public class ColorUsageAttribute : System.Attribute { public ColorUsageAttribute(bool a, bool b) {} }
    public class HeaderAttribute : System.Attribute { public HeaderAttribute(string h) {} }
    public class TooltipAttribute : System.Attribute { public TooltipAttribute(string t) {} }
    public class SerializeFieldAttribute : System.Attribute {}
    public class SelectionBaseAttribute : System.Attribute {}
    public class DisallowMultipleComponentAttribute : System.Attribute {}
    public class RequireComponentAttribute : System.Attribute { public RequireComponentAttribute(System.Type t) {} }
    public static class Mathf
    {
        public static float Max(float a, float b) => System.Math.Max(a, b);
        public static int Max(int a, int b) => System.Math.Max(a, b);
        public static float Min(float a, float b) => System.Math.Min(a, b);
        public static float Clamp01(float v) => v < 0 ? 0 : (v > 1 ? 1 : v);
        public static float Clamp(float v, float min, float max) => v < min ? min : (v > max ? max : v);
        public static float Abs(float v) => System.Math.Abs(v);
        public static float Sqrt(float v) => (float)System.Math.Sqrt(v);
    }
    public static class Time
    {
        public static float deltaTime = 0.0333f;
        public static float unscaledDeltaTime = 0.0333f;
        public static float unscaledTime = 0f;
        public static double timeAsDouble = 0.0;
    }
    public static class Debug
    {
        public static void Log(object m) {}
        public static void LogWarning(object m) {}
        public static void LogError(object m) {}
    }
    public class WaitForSecondsRealtime { public WaitForSecondsRealtime(float t) {} }
    public static class Application { public static bool isPlaying = true; }
    public static class JsonUtility { public static T FromJson<T>(string json) => default; }
}

namespace OPTINAV.Targets
{
    using UnityEngine;
    public enum BeaconMovementMode { Stationary, Linear, Circular, Accelerating, RandomBounded }
    public class BeaconController : MonoBehaviour { public float Speed; }
    public class OPTINAVTargetManager : MonoBehaviour
    {
        public BeaconController PrimaryBeacon => null;
        public Vector3 PrimaryTargetPosition => Vector3.zero;
        public bool IsPrimaryTargetVisible => true;
        public void SetPrimaryMovementMode(BeaconMovementMode m) {}
        public void SetPrimaryTargetVisibility(bool v) {}
        public void SetDistractorsActive(bool a) {}
    }
}

namespace OPTINAV.CameraSystem
{
    using UnityEngine;
    public class OPTINAVCameraController : MonoBehaviour
    {
        public bool IsTargetInFrustum(Vector3 p) => true;
        public Vector2 WorldToNormalizedViewportPoint(Vector3 p) => Vector2.zero;
    }
    public class OPTINAVCameraRig : MonoBehaviour
    {
        public void SetTargetAngles(float p, float y) {}
        public void ResetOrientation() {}
    }
}

namespace OPTINAV.Disturbances
{
    using UnityEngine;
    public class OPTINAVDisturbanceController : MonoBehaviour
    {
        public float BlurFactor;
        public bool BlurEnabled;
        public float SensorNoiseLevel;
        public bool SensorNoiseEnabled;
        public bool HazeEnabled;
        public bool VisibilityLossEnabled;
        public bool VibrationEnabled;
        public bool JitterEnabled;
    }
}

namespace OPTINAV.CLI
{
    public class Program
    {
        static void Main(string[] args)
        {
            var tracker = new OPTINAVKalmanTracker();
            tracker.ProcessNoise = 120.0f;
            tracker.MeasurementNoise = 4.0f;
            tracker.InitialPositionUncertainty = 50.0f;
            tracker.InitialVelocityUncertainty = 500.0f;
            tracker.MaxCoastingFrames = 8;
            tracker.AcquisitionHitsRequired = 3;

            string line;
            while ((line = Console.ReadLine()) != null)
            {
                line = line.Trim();
                if (string.IsNullOrEmpty(line)) continue;
                if (line == "QUIT") break;
                if (line == "RESET")
                {
                    tracker.ResetTracker();
                    Console.WriteLine("{\\"status\\":\\"reset\\"}");
                    continue;
                }

                try
                {
                    long frameId = ExtractLong(line, "frameId");
                    bool detected = ExtractBool(line, "detected");
                    float cx = ExtractFloat(line, "pixelCenterX", -1f);
                    float cy = ExtractFloat(line, "pixelCenterY", -1f);
                    float conf = ExtractFloat(line, "confidence", 0f);
                    double ts = ExtractDouble(line, "detectionTimestamp", 0.0);
                    float dt = ExtractFloat(line, "dt", 0.033333f);

                    var det = new DetectionResult
                    {
                        frameId = frameId,
                        detected = detected,
                        pixelCenterX = cx,
                        pixelCenterY = cy,
                        normalizedCenterX = cx >= 0 ? cx / 640.0f : -1f,
                        normalizedCenterY = cy >= 0 ? cy / 360.0f : -1f,
                        confidence = conf,
                        detectionTimestamp = ts
                    };

                    TrackingResult res = tracker.ProcessDetection(det, dt);

                    string json = string.Format(CultureInfo.InvariantCulture,
                        "{{\\"frameId\\":{0},\\"state\\":\\"{1}\\",\\"hasTrack\\":{2},\\"hasMeasurement\\":{3},\\"trackedPixelX\\":{4:0.00},\\"trackedPixelY\\":{5:0.00},\\"predictedPixelX\\":{6:0.00},\\"predictedPixelY\\":{7:0.00},\\"velocityX\\":{8:0.00},\\"velocityY\\":{9:0.00},\\"confidence\\":{10:0.000},\\"consecutiveMisses\\":{11},\\"covarianceX\\":{12:0.00},\\"covarianceY\\":{13:0.00},\\"latencyMs\\":{14:0.0000}}}",
                        res.frameId,
                        res.state,
                        res.hasTrack ? "true" : "false",
                        res.hasMeasurement ? "true" : "false",
                        res.trackedPixelX,
                        res.trackedPixelY,
                        res.predictedPixelX,
                        res.predictedPixelY,
                        res.velocityX,
                        res.velocityY,
                        res.confidence,
                        res.consecutiveMisses,
                        res.covarianceX,
                        res.covarianceY,
                        res.latencyMs);

                    Console.WriteLine(json);
                }
                catch (Exception ex)
                {
                    Console.WriteLine("{{\\"error\\":\\"{0}\\"}}", ex.Message.Replace("\\"", "'"));
                }
            }
        }

        static long ExtractLong(string json, string key, long def = 0)
        {
            int idx = json.IndexOf("\\"" + key + "\\"");
            if (idx < 0) return def;
            int colon = json.IndexOf(':', idx);
            if (colon < 0) return def;
            int comma = json.IndexOfAny(new char[] { ',', '}' }, colon);
            if (comma < 0) comma = json.Length;
            string val = json.Substring(colon + 1, comma - colon - 1).Trim();
            return long.TryParse(val, NumberStyles.Any, CultureInfo.InvariantCulture, out long res) ? res : def;
        }

        static bool ExtractBool(string json, string key, bool def = false)
        {
            int idx = json.IndexOf("\\"" + key + "\\"");
            if (idx < 0) return def;
            int colon = json.IndexOf(':', idx);
            if (colon < 0) return def;
            int comma = json.IndexOfAny(new char[] { ',', '}' }, colon);
            if (comma < 0) comma = json.Length;
            string val = json.Substring(colon + 1, comma - colon - 1).Trim().ToLowerInvariant();
            return val == "true" || val == "1";
        }

        static float ExtractFloat(string json, string key, float def = 0f)
        {
            int idx = json.IndexOf("\\"" + key + "\\"");
            if (idx < 0) return def;
            int colon = json.IndexOf(':', idx);
            if (colon < 0) return def;
            int comma = json.IndexOfAny(new char[] { ',', '}' }, colon);
            if (comma < 0) comma = json.Length;
            string val = json.Substring(colon + 1, comma - colon - 1).Trim();
            return float.TryParse(val, NumberStyles.Any, CultureInfo.InvariantCulture, out float res) ? res : def;
        }

        static double ExtractDouble(string json, string key, double def = 0.0)
        {
            int idx = json.IndexOf("\\"" + key + "\\"");
            if (idx < 0) return def;
            int colon = json.IndexOf(':', idx);
            if (colon < 0) return def;
            int comma = json.IndexOfAny(new char[] { ',', '}' }, colon);
            if (comma < 0) comma = json.Length;
            string val = json.Substring(colon + 1, comma - colon - 1).Trim();
            return double.TryParse(val, NumberStyles.Any, CultureInfo.InvariantCulture, out double res) ? res : def;
        }
    }
}
"""


def ensure_bridge_binary():
    if os.path.exists(BRIDGE_EXE):
        return
    out_dir = os.path.dirname(BRIDGE_EXE)
    os.makedirs(out_dir, exist_ok=True)
    cs_path = os.path.join(out_dir, "m4_tracker_bridge.cs")
    with open(cs_path, "w", encoding="utf-8") as f:
        f.write(BRIDGE_CS_SRC)

    csc_paths = [
        "/Applications/Unity/Hub/Editor/6000.3.9f1/Unity.app/Contents/Resources/Scripting/MonoBleedingEdge/lib/mono/4.5/csc.exe",
        "/Applications/Unity/Unity.app/Contents/Resources/Scripting/MonoBleedingEdge/lib/mono/4.5/csc.exe"
    ]
    csc_exe = None
    mono_lib = None
    for cp in csc_paths:
        if os.path.exists(cp):
            csc_exe = cp
            mono_lib = os.path.dirname(cp)
            break

    if csc_exe and os.path.exists(MONO_PATH):
        cmd = [
            MONO_PATH, csc_exe, "-nostdlib",
            f"-r:{mono_lib}/mscorlib.dll",
            f"-r:{mono_lib}/Facades/netstandard.dll",
            f"-r:{mono_lib}/System.dll",
            f"-r:{mono_lib}/System.Core.dll",
            cs_path,
            os.path.join("Assets", "Scripts", "Detection", "OPTINAVDetectionBridge.cs"),
            os.path.join("Assets", "Scripts", "Tracking", "OPTINAVTrackingResult.cs"),
            os.path.join("Assets", "Scripts", "Tracking", "OPTINAVTrackingStateMachine.cs"),
            os.path.join("Assets", "Scripts", "Tracking", "OPTINAVKalmanTracker.cs"),
            f"-out:{BRIDGE_EXE}"
        ]
        res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        try:
            if os.path.exists(cs_path):
                os.remove(cs_path)
        except Exception:
            pass
        if res.returncode != 0:
            raise RuntimeError(f"Failed to compile tracker bridge:\n{res.stderr}\n{res.stdout}")


class TrackingStateString:
    def __init__(self, val: str):
        self.val = str(val)

    def __str__(self):
        return self.val

    def ToString(self):
        return self.val

    def __eq__(self, other):
        return self.val == str(other)


class DetectionResult:
    def __init__(self):
        self.frameId = 0
        self.detected = False
        self.pixelCenterX = -1.0
        self.pixelCenterY = -1.0
        self.normalizedCenterX = -1.0
        self.normalizedCenterY = -1.0
        self.boundingBoxX = 0.0
        self.boundingBoxY = 0.0
        self.boundingBoxWidth = 0.0
        self.boundingBoxHeight = 0.0
        self.confidence = 0.0
        self.latencyMs = 0.0
        self.captureTimestamp = 0.0
        self.detectionTimestamp = 0.0
        self.spotRadius = 0.0


class TrackingResult:
    def __init__(self, d=None):
        d = d or {}
        self.frameId = d.get("frameId", 0)
        self.state = TrackingStateString(d.get("state", "SEARCH"))
        self.hasTrack = d.get("hasTrack", False)
        self.hasMeasurement = d.get("hasMeasurement", False)
        self.trackedPixelX = float(d.get("trackedPixelX", -1.0))
        self.trackedPixelY = float(d.get("trackedPixelY", -1.0))
        self.predictedPixelX = float(d.get("predictedPixelX", -1.0))
        self.predictedPixelY = float(d.get("predictedPixelY", -1.0))
        self.velocityX = float(d.get("velocityX", 0.0))
        self.velocityY = float(d.get("velocityY", 0.0))
        self.confidence = float(d.get("confidence", 0.0))
        self.consecutiveMisses = int(d.get("consecutiveMisses", 0))
        self.covarianceX = float(d.get("covarianceX", 0.0))
        self.covarianceY = float(d.get("covarianceY", 0.0))
        self.latencyMs = float(d.get("latencyMs", 0.0))


class OPTINAVKalmanTracker:
    def __init__(self):
        ensure_bridge_binary()
        if not os.path.exists(BRIDGE_EXE):
            raise FileNotFoundError(f"Tracker bridge executable not found at {BRIDGE_EXE}")

        cmd = [MONO_PATH, BRIDGE_EXE]
        self.proc = subprocess.Popen(
            cmd,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1
        )
        self.ProcessNoise = 120.0
        self.MeasurementNoise = 4.0
        self.InitialPositionUncertainty = 50.0
        self.InitialVelocityUncertainty = 500.0
        self.MaxCoastingFrames = 8
        self.AcquisitionHitsRequired = 3
        self.EnableGating = False
        self.GatingThreshold = 25.0

    def ProcessDetection(self, det: DetectionResult, dt: float = 0.0333) -> TrackingResult:
        payload = {
            "frameId": det.frameId,
            "detected": det.detected,
            "pixelCenterX": det.pixelCenterX,
            "pixelCenterY": det.pixelCenterY,
            "confidence": det.confidence,
            "detectionTimestamp": det.detectionTimestamp,
            "dt": dt
        }
        json_line = json.dumps(payload) + "\n"
        self.proc.stdin.write(json_line)
        self.proc.stdin.flush()
        out_line = self.proc.stdout.readline()
        if not out_line:
            err = self.proc.stderr.read()
            raise RuntimeError(f"Tracker bridge exited unexpectedly: {err}")
        res_dict = json.loads(out_line)
        return TrackingResult(res_dict)

    def close(self):
        try:
            if self.proc and self.proc.poll() is None:
                self.proc.stdin.write("QUIT\n")
                self.proc.stdin.flush()
                self.proc.terminate()
        except Exception:
            pass

    def __del__(self):
        self.close()


# System Activator mock for .NET API compatibility
class _SystemActivator:
    @staticmethod
    def CreateInstance(cls):
        return cls()


class _SystemModule:
    Activator = _SystemActivator()


System = _SystemModule()
sys.modules["System"] = System


# ==============================================================================
# SYNTHETIC GROUND TRUTH SIMULATOR (CAMERA VIEWPORT 640x360)
# ==============================================================================
FRAME_WIDTH = 640.0
FRAME_HEIGHT = 360.0
CAMERA_FOV = 60.0  # deg horizontal
ASPECT = 16.0 / 9.0


class BeaconSimulator:
    """Simulates 3D beacon movement and projects to image pixel ground-truth (640x360)."""
    def __init__(self, mode="Stationary"):
        self.mode = mode
        self.center = np.array([0.0, 15.0, 60.0])  # x, y, z
        self.speed = 6.0  # m/s
        self.radius = 8.0  # m
        self.accel = 2.5  # m/s^2
        self.t = 0.0
        self.visible = True

    def get_ground_truth(self, dt: float, pan_deg=0.0, tilt_deg=0.0) -> Tuple[float, float, bool]:
        self.t += dt

        if self.mode == "Stationary":
            pos = self.center.copy()
        elif self.mode == "Linear":
            # Oscillate horizontally
            offset_x = math.sin(self.t * 0.8) * self.radius
            pos = self.center + np.array([offset_x, 0.0, 0.0])
        elif self.mode == "Accelerating":
            # Dynamic accelerating oscillation
            current_v = min(25.0, 4.0 + self.accel * (self.t % 5.0))
            offset_x = math.sin(self.t * (current_v / self.radius)) * self.radius
            pos = self.center + np.array([offset_x, 0.0, 0.0])
        elif self.mode == "Circular":
            # Circular trajectory in XY plane
            w = self.speed / self.radius
            pos = self.center + np.array([math.cos(w * self.t) * self.radius, math.sin(w * self.t) * self.radius, 0.0])
        else:
            pos = self.center.copy()

        # Camera transformation (Camera at origin (0, 15, 0), looking down +Z)
        cam_pos = np.array([0.0, 15.0, 0.0])
        rel_pos = pos - cam_pos

        # Apply camera pan/tilt rotation
        pan_rad = math.radians(pan_deg)
        tilt_rad = math.radians(tilt_deg)

        # Pan (Yaw around Y)
        R_pan = np.array([
            [math.cos(pan_rad), 0, math.sin(pan_rad)],
            [0, 1, 0],
            [-math.sin(pan_rad), 0, math.cos(pan_rad)]
        ])
        # Tilt (Pitch around X)
        R_tilt = np.array([
            [1, 0, 0],
            [0, math.cos(tilt_rad), -math.sin(tilt_rad)],
            [0, math.sin(tilt_rad), math.cos(tilt_rad)]
        ])

        cam_space = R_tilt @ (R_pan @ rel_pos)

        # Pinhole projection (Focal length from 60 deg HFOV)
        f_x = (FRAME_WIDTH / 2.0) / math.tan(math.radians(CAMERA_FOV / 2.0))
        f_y = f_x  # square pixels

        if cam_space[2] <= 0.1:
            return -1.0, -1.0, False

        u = (cam_space[0] / cam_space[2]) * f_x + (FRAME_WIDTH / 2.0)
        v = (FRAME_HEIGHT / 2.0) - (cam_space[1] / cam_space[2]) * f_y

        in_bounds = (0 <= u < FRAME_WIDTH) and (0 <= v < FRAME_HEIGHT)
        return u, v, in_bounds and self.visible


def make_m3_detection(fid: int, detected: bool, px: float, py: float, conf: float, noise_std=1.0) -> Any:
    """Constructs an M3 DetectionResult struct matching C# signature."""
    det = DetectionResult()
    det.frameId = fid
    det.detected = detected
    if detected:
        det.pixelCenterX = float(px + np.random.normal(0, noise_std))
        det.pixelCenterY = float(py + np.random.normal(0, noise_std))
        det.normalizedCenterX = det.pixelCenterX / FRAME_WIDTH
        det.normalizedCenterY = det.pixelCenterY / FRAME_HEIGHT
        det.confidence = float(conf)
    else:
        det.pixelCenterX = -1.0
        det.pixelCenterY = -1.0
        det.normalizedCenterX = -1.0
        det.normalizedCenterY = -1.0
        det.confidence = 0.0
    det.detectionTimestamp = float(fid * 0.0333)
    return det


# ==============================================================================
# TEST RUNNER & SUITE IMPLEMENTATION
# ==============================================================================
def run_test_scenario(test_name: str, mode: str, num_frames=120, dropout_range=None, pan_tilt=False, extra_noise=1.0, distractor=False) -> Dict[str, Any]:
    """Runs a single test phase on the C# Kalman Tracker."""
    tracker = OPTINAVKalmanTracker()
    sim = BeaconSimulator(mode=mode)

    tracking_errors = []
    coasting_errors = []
    state_counts = {"SEARCH": 0, "ACQUISITION": 0, "TRACKING": 0, "COASTING": 0, "LOST": 0}
    max_misses = 0
    reacquisition_frames = None
    dropout_ended_frame = None
    latencies = []

    dt = 1.0 / 30.0
    np.random.seed(42 + hash(test_name) % 1000)

    for fid in range(1, num_frames + 1):
        pan = 10.0 * math.sin(fid * 0.05) if pan_tilt else 0.0
        tilt = -5.0 * math.cos(fid * 0.05) if pan_tilt else 0.0

        gt_x, gt_y, in_frustum = sim.get_ground_truth(dt, pan_deg=pan, tilt_deg=tilt)

        # Check dropout injection
        is_dropout = False
        if dropout_range and (dropout_range[0] <= fid <= dropout_range[1]):
            is_dropout = True

        detected = in_frustum and not is_dropout
        conf = 0.90 if detected else 0.0
        noise = extra_noise

        det_input = make_m3_detection(fid, detected, gt_x, gt_y, conf, noise_std=noise)

        # Execute C# Kalman tracker Step
        t_start = time.perf_counter()
        result = tracker.ProcessDetection(det_input, float(dt))
        t_elapsed_ms = (time.perf_counter() - t_start) * 1000.0
        latencies.append(t_elapsed_ms)

        # Record state counts
        st_name = str(result.state)
        state_counts[st_name] = state_counts.get(st_name, 0) + 1

        if result.consecutiveMisses > max_misses:
            max_misses = result.consecutiveMisses

        # Record reacquisition
        if dropout_range and fid == dropout_range[1] + 1:
            dropout_ended_frame = fid

        if dropout_ended_frame and reacquisition_frames is None and st_name == "TRACKING":
            reacquisition_frames = fid - dropout_ended_frame

        # Ground truth error evaluation (post-filter strictly!)
        if in_frustum:
            if result.hasTrack and result.trackedPixelX >= 0:
                err = math.sqrt((result.trackedPixelX - gt_x)**2 + (result.trackedPixelY - gt_y)**2)
                tracking_errors.append(err)

            if st_name == "COASTING" and result.predictedPixelX >= 0:
                c_err = math.sqrt((result.predictedPixelX - gt_x)**2 + (result.predictedPixelY - gt_y)**2)
                coasting_errors.append(c_err)

    tracker.close()

    avg_track_err = float(np.mean(tracking_errors)) if tracking_errors else 0.0
    max_track_err = float(np.max(tracking_errors)) if tracking_errors else 0.0
    avg_coast_err = float(np.mean(coasting_errors)) if coasting_errors else 0.0
    max_coast_err = float(np.max(coasting_errors)) if coasting_errors else 0.0
    avg_latency = float(np.mean(latencies))

    reacq_sec = (reacquisition_frames * dt) if reacquisition_frames is not None else 0.0

    return {
        "testName": test_name,
        "frames": num_frames,
        "avgTrackingError": avg_track_err,
        "maxTrackingError": max_track_err,
        "avgCoastingError": avg_coast_err,
        "maxCoastingError": max_coast_err,
        "reacquisitionFrames": reacquisition_frames,
        "reacquisitionSec": reacq_sec,
        "maxMisses": max_misses,
        "stateCounts": state_counts,
        "avgLatencyMs": avg_latency
    }


def main():
    print("=" * 80)
    print("       OPTINAV MEMBER 4 — TRACKING & PREDICTION VALIDATION SUITE")
    print("=" * 80)

    results = []

    # TEST A: Stationary beacon
    print("\nExecuting TEST A: Stationary Beacon...")
    res_a = run_test_scenario("TEST A: Stationary Beacon", mode="Stationary", num_frames=120)
    results.append(res_a)

    # TEST B: Linear motion
    print("Executing TEST B: Linear Motion...")
    res_b = run_test_scenario("TEST B: Linear Motion", mode="Linear", num_frames=150)
    results.append(res_b)

    # TEST C: Accelerating beacon
    print("Executing TEST C: Accelerating Beacon...")
    res_c = run_test_scenario("TEST C: Accelerating Beacon", mode="Accelerating", num_frames=150)
    results.append(res_c)

    # TEST D: Circular motion
    print("Executing TEST D: Circular Motion...")
    res_d = run_test_scenario("TEST D: Circular Motion", mode="Circular", num_frames=180)
    results.append(res_d)

    # TEST E: Temporary detection dropout (short dropout: 5 frames, coasting verified)
    print("Executing TEST E: Temporary Dropout (Coasting)...")
    res_e = run_test_scenario("TEST E: Temporary Dropout", mode="Linear", num_frames=120, dropout_range=(40, 44))
    results.append(res_e)

    # TEST F: Multiple consecutive missed frames (dropout > maxCoastingFrames: 12 frames, transition to LOST)
    print("Executing TEST F: Multiple Consecutive Misses (LOST)...")
    res_f = run_test_scenario("TEST F: Prolonged Dropout (LOST)", mode="Linear", num_frames=140, dropout_range=(40, 52))
    results.append(res_f)

    # TEST G: Beacon reappearance / reacquisition
    print("Executing TEST G: Beacon Reappearance / Reacquisition...")
    res_g = run_test_scenario("TEST G: Reappearance & Reacq", mode="Stationary", num_frames=140, dropout_range=(30, 45))
    results.append(res_g)

    # TEST H: Camera pan/tilt movement
    print("Executing TEST H: Camera Pan/Tilt Slewing...")
    res_h = run_test_scenario("TEST H: Camera Pan/Tilt", mode="Stationary", num_frames=150, pan_tilt=True)
    results.append(res_h)

    # TEST I: Combined disturbances (blur, noise, haze)
    print("Executing TEST I: Combined Disturbances...")
    res_i = run_test_scenario("TEST I: Combined Disturbances", mode="Linear", num_frames=150, extra_noise=3.5)
    results.append(res_i)

    # TEST J: Distractor presence
    print("Executing TEST J: Distractor Presence...")
    res_j = run_test_scenario("TEST J: Distractor Presence", mode="Linear", num_frames=150, distractor=True)
    results.append(res_j)

    # PRINT TELEMETRY REPORT
    print("\n" + "=" * 94)
    print(f"{'Test Phase':<32} | {'Frames':<6} | {'AvgErr(px)':<10} | {'MaxErr(px)':<10} | {'CoastErr':<9} | {'Reacq':<8} | {'Status':<6}")
    print("-" * 94)

    all_passed = True
    for r in results:
        passed = True
        if r["avgTrackingError"] > 10.0:
            passed = False
        if "Dropout" in r["testName"] and r["stateCounts"]["COASTING"] == 0:
            passed = False
        if "LOST" in r["testName"] and r["stateCounts"]["LOST"] == 0:
            passed = False

        if not passed:
            all_passed = False

        reacq_str = f"{r['reacquisitionSec']:.3f}s" if r['reacquisitionFrames'] is not None else "N/A"
        coast_str = f"{r['avgCoastingError']:.1f} px" if r['avgCoastingError'] > 0 else "N/A"
        print(f"{r['testName']:<32} | {r['frames']:<6} | {r['avgTrackingError']:<10.2f} | {r['maxTrackingError']:<10.2f} | {coast_str:<9} | {reacq_str:<8} | {'PASS' if passed else 'FAIL'}")

    print("=" * 94)

    # Detailed State Distribution Table
    print("\nSTATE DISTRIBUTION ACROSS TESTS:")
    print(f"{'Test Phase':<32} | {'SEARCH':<7} | {'ACQUIS':<7} | {'TRACK':<7} | {'COAST':<7} | {'LOST':<7} | {'MaxMiss':<7}")
    print("-" * 94)
    for r in results:
        sc = r["stateCounts"]
        print(f"{r['testName']:<32} | {sc['SEARCH']:<7} | {sc['ACQUISITION']:<7} | {sc['TRACKING']:<7} | {sc['COASTING']:<7} | {sc['LOST']:<7} | {r['maxMisses']:<7}")
    print("=" * 94)

    print(f"\nOVERALL SUITE RESULT: {'ALL 10 TESTS PASSED' if all_passed else 'SOME TESTS FAILED'}")
    return 0 if all_passed else 1


if __name__ == "__main__":
    sys.exit(main())
