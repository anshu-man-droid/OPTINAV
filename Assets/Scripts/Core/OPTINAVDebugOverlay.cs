using UnityEngine;
using OPTINAV.Targets;
using OPTINAV.CameraSystem;
using OPTINAV.Disturbances;
#if ENABLE_INPUT_SYSTEM
using UnityEngine.InputSystem;
#endif

namespace OPTINAV.Core
{
    /// <summary>
    /// Minimal, lightweight development overlay for verifying the OPTINAV simulation testbed.
    /// Compatible with Unity's new Input System and legacy input.
    /// Uses IMGUI (OnGUI) with clickable GUI buttons and keyboard shortcuts.
    /// Displays beacon telemetry, camera orientation, disturbance flags, and interactive controls.
    /// </summary>
    public class OPTINAVDebugOverlay : MonoBehaviour
    {
        [Header("Overlay Settings")]
        [SerializeField] private bool showOverlay = true;

        [Header("Subsystem References")]
        [SerializeField] private OPTINAVTargetManager targetManager;
        [SerializeField] private OPTINAVCameraRig cameraRig;
        [SerializeField] private OPTINAVDisturbanceController disturbanceController;
        [SerializeField] private OPTINAVSimulationManager simulationManager;

        private GUIStyle headerStyle;
        private GUIStyle labelStyle;
        private GUIStyle boxStyle;
        private GUIStyle buttonStyle;
        private bool stylesInitialized = false;

        private void Awake()
        {
            FindReferences();
        }

        private void FindReferences()
        {
            if (targetManager == null) targetManager = FindFirstObjectByType<OPTINAVTargetManager>();
            if (cameraRig == null) cameraRig = FindFirstObjectByType<OPTINAVCameraRig>();
            if (disturbanceController == null) disturbanceController = FindFirstObjectByType<OPTINAVDisturbanceController>();
            if (simulationManager == null) simulationManager = FindFirstObjectByType<OPTINAVSimulationManager>();
        }

        private void Update()
        {
#if ENABLE_INPUT_SYSTEM
            var kb = Keyboard.current;
            if (kb != null)
            {
                if (kb.f1Key.wasPressedThisFrame) showOverlay = !showOverlay;
                if (kb.digit1Key.wasPressedThisFrame) SetMode(BeaconMovementMode.Stationary);
                if (kb.digit2Key.wasPressedThisFrame) SetMode(BeaconMovementMode.Linear);
                if (kb.digit3Key.wasPressedThisFrame) SetMode(BeaconMovementMode.Circular);
                if (kb.digit4Key.wasPressedThisFrame) SetMode(BeaconMovementMode.Accelerating);
                if (kb.digit5Key.wasPressedThisFrame) SetMode(BeaconMovementMode.RandomBounded);

                if (kb.vKey.wasPressedThisFrame && disturbanceController != null)
                    disturbanceController.VibrationEnabled = !disturbanceController.VibrationEnabled;
                if (kb.jKey.wasPressedThisFrame && disturbanceController != null)
                    disturbanceController.JitterEnabled = !disturbanceController.JitterEnabled;
                if (kb.hKey.wasPressedThisFrame && disturbanceController != null)
                    disturbanceController.HazeEnabled = !disturbanceController.HazeEnabled;
                if (kb.lKey.wasPressedThisFrame && disturbanceController != null)
                    disturbanceController.VisibilityLossEnabled = !disturbanceController.VisibilityLossEnabled;
                if (kb.bKey.wasPressedThisFrame && disturbanceController != null)
                    disturbanceController.BlurEnabled = !disturbanceController.BlurEnabled;
                if (kb.nKey.wasPressedThisFrame && disturbanceController != null)
                    disturbanceController.SensorNoiseEnabled = !disturbanceController.SensorNoiseEnabled;
            }
#endif
        }

        private void SetMode(BeaconMovementMode mode)
        {
            if (targetManager != null) targetManager.SetPrimaryMovementMode(mode);
        }

        private void InitStyles()
        {
            if (stylesInitialized) return;

            headerStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 14,
                fontStyle = FontStyle.Bold,
                normal = { textColor = new Color(0.2f, 0.9f, 1f) }
            };

            labelStyle = new GUIStyle(GUI.skin.label)
            {
                fontSize = 11,
                normal = { textColor = Color.white }
            };

            buttonStyle = new GUIStyle(GUI.skin.button)
            {
                fontSize = 10
            };

            boxStyle = new GUIStyle(GUI.skin.box);
            Texture2D bgTex = new Texture2D(1, 1);
            bgTex.SetPixel(0, 0, new Color(0.08f, 0.1f, 0.14f, 0.88f));
            bgTex.Apply();
            boxStyle.normal.background = bgTex;

            stylesInitialized = true;
        }

        private void OnGUI()
        {
            if (!showOverlay) return;
            InitStyles();

            GUILayout.BeginArea(new Rect(15, 15, 360, 520), boxStyle);
            GUILayout.Space(6);

            GUILayout.Label("OPTINAV SIMULATION TESTBED", headerStyle);
            GUILayout.Label("FSOC Virtual Camera & Target Tracking Foundation", labelStyle);
            GUILayout.Space(4);

            // 1. Target Status
            GUILayout.Label("── TARGET SIMULATION ──", headerStyle);
            if (targetManager != null && targetManager.PrimaryBeacon != null)
            {
                var b = targetManager.PrimaryBeacon;
                GUILayout.Label($"Mode: {b.MovementMode} | Visible: {(b.IsVisible ? "<color=green>YES</color>" : "<color=red>DROPOUT</color>")}", labelStyle);
                GUILayout.Label($"Pos: ({b.CurrentPosition.x:F2}, {b.CurrentPosition.y:F2}, {b.CurrentPosition.z:F2}) m", labelStyle);
                GUILayout.Label($"Vel: ({b.CurrentVelocity.x:F2}, {b.CurrentVelocity.y:F2}, {b.CurrentVelocity.z:F2}) m/s (Speed: {b.CurrentVelocity.magnitude:F2})", labelStyle);
            }
            else
            {
                GUILayout.Label("No Beacon Detected", labelStyle);
            }

            GUILayout.Space(4);

            // 2. Camera Rig Status
            GUILayout.Label("── VIRTUAL CAMERA RIG ──", headerStyle);
            if (cameraRig != null)
            {
                GUILayout.Label($"Pan Angle:  {cameraRig.CurrentPanAngle:F2}° (Limits: [{cameraRig.MinPan:F0}°, {cameraRig.MaxPan:F0}°])", labelStyle);
                GUILayout.Label($"Tilt Angle: {cameraRig.CurrentTiltAngle:F2}° (Limits: [{cameraRig.MinTilt:F0}°, {cameraRig.MaxTilt:F0}°])", labelStyle);
            }
            else
            {
                GUILayout.Label("No Camera Rig Detected", labelStyle);
            }

            GUILayout.Space(4);

            // 3. Disturbances
            GUILayout.Label("── ACTIVE DISTURBANCES ──", headerStyle);
            if (disturbanceController != null)
            {
                var dc = disturbanceController;
                string vibStr = dc.VibrationEnabled ? "<color=orange>ON</color>" : "<color=grey>OFF</color>";
                string jitStr = dc.JitterEnabled ? "<color=orange>ON</color>" : "<color=grey>OFF</color>";
                string hazeStr = dc.HazeEnabled ? "<color=orange>ON</color>" : "<color=grey>OFF</color>";
                string blurStr = dc.BlurEnabled ? $"<color=orange>ON ({dc.CurrentBlurFactor:F2})</color>" : "<color=grey>OFF</color>";
                string noiseStr = dc.SensorNoiseEnabled ? $"<color=orange>ON ({dc.CurrentNoiseLevel:F2})</color>" : "<color=grey>OFF</color>";
                string dropStr = dc.VisibilityLossEnabled ? (dc.IsDropoutActive ? "<color=red>ACTIVE DROPOUT</color>" : "<color=green>ENABLED (STANDBY)</color>") : "<color=grey>OFF</color>";

                GUILayout.Label($"Vibration: {vibStr} | Jitter: {jitStr} | Haze: {hazeStr}", labelStyle);
                GUILayout.Label($"Camera Blur: {blurStr} | Sensor Noise: {noiseStr}", labelStyle);
                GUILayout.Label($"Beacon Visibility Dropout: {dropStr}", labelStyle);
            }
            else
            {
                GUILayout.Label("No Disturbance Controller Detected", labelStyle);
            }

            GUILayout.Space(6);

            // 4. Quick Test Toggles
            GUILayout.Label("── INTERACTIVE TEST CONTROLS ──", headerStyle);

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Stationary (1)", buttonStyle)) SetMode(BeaconMovementMode.Stationary);
            if (GUILayout.Button("Linear (2)", buttonStyle)) SetMode(BeaconMovementMode.Linear);
            if (GUILayout.Button("Circular (3)", buttonStyle)) SetMode(BeaconMovementMode.Circular);
            GUILayout.EndHorizontal();

            GUILayout.BeginHorizontal();
            if (GUILayout.Button("Accel (4)", buttonStyle)) SetMode(BeaconMovementMode.Accelerating);
            if (GUILayout.Button("Random (5)", buttonStyle)) SetMode(BeaconMovementMode.RandomBounded);
            if (GUILayout.Button("Reset", buttonStyle) && targetManager != null) targetManager.ResetSimulation();
            GUILayout.EndHorizontal();

            if (disturbanceController != null)
            {
                GUILayout.BeginHorizontal();
                if (GUILayout.Button($"Vibration [V]: {(disturbanceController.VibrationEnabled ? "OFF" : "ON")}", buttonStyle))
                    disturbanceController.VibrationEnabled = !disturbanceController.VibrationEnabled;
                if (GUILayout.Button($"Jitter [J]: {(disturbanceController.JitterEnabled ? "OFF" : "ON")}", buttonStyle))
                    disturbanceController.JitterEnabled = !disturbanceController.JitterEnabled;
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button($"Blur [B]: {(disturbanceController.BlurEnabled ? "OFF" : "ON")}", buttonStyle))
                    disturbanceController.BlurEnabled = !disturbanceController.BlurEnabled;
                if (GUILayout.Button($"Noise [N]: {(disturbanceController.SensorNoiseEnabled ? "OFF" : "ON")}", buttonStyle))
                    disturbanceController.SensorNoiseEnabled = !disturbanceController.SensorNoiseEnabled;
                GUILayout.EndHorizontal();

                GUILayout.BeginHorizontal();
                if (GUILayout.Button($"Haze [H]: {(disturbanceController.HazeEnabled ? "OFF" : "ON")}", buttonStyle))
                    disturbanceController.HazeEnabled = !disturbanceController.HazeEnabled;
                if (GUILayout.Button($"Loss [L]: {(disturbanceController.VisibilityLossEnabled ? "OFF" : "ON")}", buttonStyle))
                    disturbanceController.VisibilityLossEnabled = !disturbanceController.VisibilityLossEnabled;
                GUILayout.EndHorizontal();
            }

            GUILayout.Space(4);
            float fps = 1.0f / Mathf.Max(0.0001f, Time.smoothDeltaTime);
            GUILayout.Label($"FPS: {fps:F0} | Sim Time: {(simulationManager != null ? simulationManager.ElapsedTime : Time.time):F1}s", labelStyle);

            GUILayout.EndArea();
        }
    }
}
