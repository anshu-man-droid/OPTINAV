using UnityEngine;
using OPTINAV.CameraSystem;
using OPTINAV.Targets;

namespace OPTINAV.Detection
{
    /// <summary>
    /// Visualizes Member 3 computer-vision detection results in the Unity Game view.
    /// Strict adherence to M3 rules:
    /// - Marker position is driven SOLELY by CV detection coordinates received from Python.
    /// - It is NEVER driven by Unity ground truth or tracking prediction.
    /// - During dropout / not detected, the detection marker is hidden and NOT DETECTED is displayed.
    /// - Benchmarking against ground-truth is optional, separate, and explicitly labeled.
    /// </summary>
    [RequireComponent(typeof(OPTINAVDetectionBridge))]
    [DisallowMultipleComponent]
    public class OPTINAVDetectionVisualizer : MonoBehaviour
    {
        [Header("Visualization Configuration")]
        [Tooltip("Toggle overlay rendering")]
        [SerializeField] private bool showVisualization = true;

        [Tooltip("Color for the CV detection bounding box and crosshair")]
        [SerializeField] private Color detectionColor = new Color(0.0f, 1.0f, 0.8f, 1.0f); // Bright cyan

        [Tooltip("Color when target is not detected")]
        [SerializeField] private Color lostColor = new Color(1.0f, 0.2f, 0.2f, 1.0f); // Bright red

        [Header("Benchmarking / Ground Truth Evaluation")]
        [Tooltip("Show ground-truth evaluation marker and pixel error (Evaluation only!)")]
        [SerializeField] private bool showGroundTruthEvaluation = true;

        [Tooltip("Color for ground truth marker (distinct from detector)")]
        [SerializeField] private Color groundTruthColor = new Color(1.0f, 0.8f, 0.0f, 0.85f); // Amber / Yellow

        [Header("Telemetry (Live)")]
        [SerializeField] private float lastPixelError = -1f;
        [SerializeField] private bool isGroundTruthValid = false;

        public float LastPixelError => lastPixelError;
        public bool IsGroundTruthValid => isGroundTruthValid;

        private OPTINAVDetectionBridge detectionBridge;
        private OPTINAVCameraController cameraController;
        private OPTINAVTargetManager targetManager;

        private Texture2D boxBorderTexture;
        private Texture2D backgroundTexture;
        private GUIStyle headerStyle;
        private GUIStyle labelStyle;
        private GUIStyle statusStyle;
        private GUIStyle gtStyle;

        private void Awake()
        {
            detectionBridge = GetComponent<OPTINAVDetectionBridge>();
            cameraController = FindFirstObjectByType<OPTINAVCameraController>();
            targetManager = FindFirstObjectByType<OPTINAVTargetManager>();

            boxBorderTexture = new Texture2D(1, 1);
            boxBorderTexture.SetPixel(0, 0, Color.white);
            boxBorderTexture.Apply();

            backgroundTexture = new Texture2D(1, 1);
            backgroundTexture.SetPixel(0, 0, new Color(0.05f, 0.07f, 0.1f, 0.85f));
            backgroundTexture.Apply();
        }

        private void OnDestroy()
        {
            if (boxBorderTexture != null) Destroy(boxBorderTexture);
            if (backgroundTexture != null) Destroy(backgroundTexture);
        }

        private void InitStyles()
        {
            if (headerStyle != null) return;

            headerStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 13,
                fontStyle = FontStyle.Bold,
                normal = { textColor = Color.white }
            };

            labelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                fontStyle = FontStyle.Normal,
                normal = { textColor = new Color(0.85f, 0.85f, 0.85f) }
            };

            statusStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 12,
                fontStyle = FontStyle.Bold
            };

            gtStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                fontStyle = FontStyle.Normal,
                normal = { textColor = groundTruthColor }
            };
        }

        private void OnGUI()
        {
            if (!showVisualization || detectionBridge == null) return;
            InitStyles();

            DetectionResult res = detectionBridge.LatestResult;
            float scaleX = (float)Screen.width / 640.0f;
            float scaleY = (float)Screen.height / 360.0f;

            // 1. Compute ground-truth pixel coordinates (FOR BENCHMARKING ONLY)
            Vector2 gtScreenPos = Vector2.zero;
            Vector2 gt640Pos = Vector2.zero;
            isGroundTruthValid = false;
            lastPixelError = -1f;

            if (targetManager != null && cameraController != null && targetManager.IsPrimaryTargetVisible)
            {
                Vector3 targetPos = targetManager.PrimaryTargetPosition;
                if (cameraController.IsTargetInFrustum(targetPos))
                {
                    Vector2 normVp = cameraController.WorldToNormalizedViewportPoint(targetPos);
                    // Viewport: (0,0) is bottom-left. Image/Screen: (0,0) is top-left.
                    gt640Pos = new Vector2(normVp.x * 640.0f, (1.0f - normVp.y) * 360.0f);
                    gtScreenPos = new Vector2(gt640Pos.x * scaleX, gt640Pos.y * scaleY);
                    isGroundTruthValid = true;

                    if (res.detected && res.pixelCenterX >= 0 && res.pixelCenterY >= 0)
                    {
                        // Calculate ground-truth error in 640x360 image coordinates
                        lastPixelError = Vector2.Distance(new Vector2(res.pixelCenterX, res.pixelCenterY), gt640Pos);
                    }
                }
            }

            // 2. Draw CV Detection Marker if target is DETECTED
            if (res.detected && res.pixelCenterX >= 0 && res.pixelCenterY >= 0)
            {
                float bx = res.boundingBoxX * scaleX;
                float by = res.boundingBoxY * scaleY;
                float bw = Mathf.Max(res.boundingBoxWidth * scaleX, 16f);
                float bh = Mathf.Max(res.boundingBoxHeight * scaleY, 16f);
                float cx = res.pixelCenterX * scaleX;
                float cy = res.pixelCenterY * scaleY;

                // Draw bounding box
                DrawRectOutline(new Rect(bx, by, bw, bh), detectionColor, 2);

                // Draw crosshair at detected centroid
                DrawCrosshair(new Vector2(cx, cy), detectionColor, 10);

                // Draw detection tag above bounding box
                GUI.color = detectionColor;
                GUI.Label(new Rect(bx, by - 22, 220, 20), $"BEACON [CONF: {res.confidence:F2}]", headerStyle);
                GUI.color = Color.white;
            }

            // 3. Draw Ground Truth marker if benchmark display is enabled (ONLY FOR EVALUATION)
            if (showGroundTruthEvaluation && isGroundTruthValid)
            {
                DrawCircleOutline(gtScreenPos, 6f, groundTruthColor, 1);
                GUI.color = groundTruthColor;
                GUI.Label(new Rect(gtScreenPos.x + 10, gtScreenPos.y - 10, 200, 18), "GT (Benchmark)", gtStyle);
                GUI.color = Color.white;

                // Draw error connection line if detected
                if (res.detected && lastPixelError >= 0)
                {
                    Vector2 detScreenPos = new Vector2(res.pixelCenterX * scaleX, res.pixelCenterY * scaleY);
                    DrawLine(detScreenPos, gtScreenPos, new Color(1f, 1f, 1f, 0.4f), 1);
                }
            }

            // 4. Draw Telemetry HUD Panel (Top-Right)
            DrawTelemetryHUD(res);
        }

        private void DrawTelemetryHUD(DetectionResult res)
        {
            float panelWidth = 260f;
            float panelHeight = 160f;
            float panelX = Screen.width - panelWidth - 15f;
            float panelY = 15f;

            Rect panelRect = new Rect(panelX, panelY, panelWidth, panelHeight);
            GUI.DrawTexture(panelRect, backgroundTexture);
            DrawRectOutline(panelRect, new Color(0.2f, 0.3f, 0.45f, 0.8f), 1);

            GUILayout.BeginArea(new Rect(panelX + 10, panelY + 8, panelWidth - 20, panelHeight - 16));

            GUILayout.Label("M3 OPTICAL BEACON DETECTOR", headerStyle);
            GUILayout.Space(2);

            // Status indicator
            statusStyle.normal.textColor = res.detected ? detectionColor : lostColor;
            string statusStr = res.detected ? "DETECTED" : "NOT DETECTED";
            GUILayout.Label($"STATUS: {statusStr}", statusStyle);

            // Metrics
            GUILayout.Label($"Confidence:      {(res.detected ? res.confidence.ToString("F3") : "0.000")}", labelStyle);
            GUILayout.Label($"Pixel Center:    {(res.detected ? $"({res.pixelCenterX:F1}, {res.pixelCenterY:F1})" : "(-1, -1)")}", labelStyle);
            GUILayout.Label($"Normalized:      {(res.detected ? $"({res.normalizedCenterX:F3}, {res.normalizedCenterY:F3})" : "(-1, -1)")}", labelStyle);
            GUILayout.Label($"CV Latency:      {res.latencyMs:F1} ms", labelStyle);
            GUILayout.Label($"Bridge FPS:      {detectionBridge.DetectionFPS:F1} FPS", labelStyle);

            if (isGroundTruthValid && lastPixelError >= 0)
            {
                gtStyle.normal.textColor = groundTruthColor;
                GUILayout.Label($"GT Pixel Error:  {lastPixelError:F1} px", gtStyle);
            }
            else
            {
                GUILayout.Label("GT Benchmark:    N/A (Dropout / Outside)", labelStyle);
            }

            GUILayout.EndArea();
        }

        private void DrawRectOutline(Rect rect, Color color, int thickness)
        {
            GUI.color = color;
            // Top, Bottom, Left, Right
            GUI.DrawTexture(new Rect(rect.x, rect.y, rect.width, thickness), boxBorderTexture);
            GUI.DrawTexture(new Rect(rect.x, rect.yMax - thickness, rect.width, thickness), boxBorderTexture);
            GUI.DrawTexture(new Rect(rect.x, rect.y, thickness, rect.height), boxBorderTexture);
            GUI.DrawTexture(new Rect(rect.xMax - thickness, rect.y, thickness, rect.height), boxBorderTexture);
            GUI.color = Color.white;
        }

        private void DrawCrosshair(Vector2 center, Color color, float size)
        {
            GUI.color = color;
            GUI.DrawTexture(new Rect(center.x - size, center.y - 1, size * 2, 2), boxBorderTexture);
            GUI.DrawTexture(new Rect(center.x - 1, center.y - size, 2, size * 2), boxBorderTexture);
            GUI.color = Color.white;
        }

        private void DrawCircleOutline(Vector2 center, float radius, Color color, int thickness)
        {
            int segments = 12;
            float angleStep = 360f / segments;
            for (int i = 0; i < segments; i++)
            {
                float a1 = i * angleStep * Mathf.Deg2Rad;
                float a2 = (i + 1) * angleStep * Mathf.Deg2Rad;
                Vector2 p1 = center + new Vector2(Mathf.Cos(a1), Mathf.Sin(a1)) * radius;
                Vector2 p2 = center + new Vector2(Mathf.Cos(a2), Mathf.Sin(a2)) * radius;
                DrawLine(p1, p2, color, thickness);
            }
        }

        private void DrawLine(Vector2 start, Vector2 end, Color color, int thickness)
        {
            GUI.color = color;
            Vector2 d = end - start;
            float length = d.magnitude;
            if (length < 0.001f) return;
            float angle = Mathf.Atan2(d.y, d.x) * Mathf.Rad2Deg;

            Matrix4x4 matrix = GUI.matrix;
            GUIUtility.RotateAroundPivot(angle, start);
            GUI.DrawTexture(new Rect(start.x, start.y - thickness / 2.0f, length, thickness), boxBorderTexture);
            GUI.matrix = matrix;
            GUI.color = Color.white;
        }
    }
}
