using System;
using System.Collections;
using System.IO;
using UnityEditor;
using UnityEngine;
using OPTINAV.Targets;
using OPTINAV.CameraSystem;
using OPTINAV.Disturbances;
using OPTINAV.Detection;
using OPTINAV.Tracking;
using OPTINAV.Control;

namespace OPTINAV.Editor
{
    /// <summary>
    /// Automated Integration Test Runner for validating the complete M1 -> M5 pipeline.
    /// Executes Test 1 (Stationary), Test 2 (Moving), Test 3 (Disturbance),
    /// Test 4 (Dropout), and Test 5 (Reacquisition).
    /// Logs detailed telemetry and verifies pass/fail criteria.
    /// </summary>
    [InitializeOnLoad]
    public static class OPTINAVIntegrationTestRunner
    {
        private const string CommandPath = "Temp/integration_command.txt";
        private const string LogPath = "Outputs/integration_tests/test_execution.log";

        static OPTINAVIntegrationTestRunner()
        {
            EditorApplication.update += CheckIntegrationCommand;
        }

        private static void CheckIntegrationCommand()
        {
            if (!File.Exists(CommandPath)) return;

            try
            {
                string cmd = File.ReadAllText(CommandPath).Trim().ToLower();
                File.Delete(CommandPath);

                if (cmd == "status")
                {
                    DumpStatus();
                }
                else if (cmd.StartsWith("run_"))
                {
                    string testName = cmd.Substring(4);
                    StartTestSequence(testName);
                }
                else if (cmd == "run" || cmd == "all")
                {
                    StartTestSequence("all");
                }
            }
            catch (Exception ex)
            {
                Debug.LogError($"[OPTINAV Test Runner] Error processing command: {ex.Message}");
            }
        }

        private static void StartTestSequence(string testName)
        {
            EditorCoroutineRunner.StartCoroutine(ExecuteTestSequenceCoroutine(testName));
        }

        private static void LogTelemetry(string msg)
        {
            Debug.Log($"[INTEGRATION_TEST] {msg}");
            try
            {
                Directory.CreateDirectory("Outputs/integration_tests");
                File.AppendAllText(LogPath, $"[{DateTime.UtcNow:yyyy-MM-ddTHH:mm:ss.fffZ}] {msg}\n");
            }
            catch {}
        }

        private static void EnsurePipelineConnected(OPTINAVDetectionBridge bridge, OPTINAVKalmanTracker tracker, OPTINAVCoarseAlignmentController controller)
        {
            if (bridge != null && tracker != null)
            {
                var bridgeField = typeof(OPTINAVKalmanTracker).GetField("detectionBridge", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                bridgeField?.SetValue(tracker, bridge);

                bridge.OnDetectionReceived -= OnBridgeDetectionToTracker;
                bridge.OnDetectionReceived += OnBridgeDetectionToTracker;
                LogTelemetry($"[OPTINAV Pipeline] M3->M4 event linked to active bridge ({bridge.gameObject.name} id={bridge.GetInstanceID()}).");
            }

            if (controller != null && tracker != null)
            {
                var trackerField = typeof(OPTINAVCoarseAlignmentController).GetField("tracker", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                trackerField?.SetValue(controller, tracker);
                controller.SyncPidLimitsWithRig();
            }
        }

        private static void OnBridgeDetectionToTracker(DetectionResult res)
        {
            var tracker = OPTINAVKalmanTracker.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVKalmanTracker>();
            if (tracker != null)
            {
                tracker.ProcessDetection(res);
            }
        }

        public static void DumpStatus()
        {
            var bridge = OPTINAVDetectionBridge.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVDetectionBridge>();
            var tracker = OPTINAVKalmanTracker.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVKalmanTracker>();
            var controller = UnityEngine.Object.FindFirstObjectByType<OPTINAVCoarseAlignmentController>();
            var rig = UnityEngine.Object.FindFirstObjectByType<OPTINAVCameraRig>();
            var targetMgr = OPTINAVTargetManager.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVTargetManager>();
            var dist = OPTINAVDisturbanceController.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVDisturbanceController>();

            EnsurePipelineConnected(bridge, tracker, controller);

            LogTelemetry("--- SYSTEM TELEMETRY SNAPSHOT ---");
            if (targetMgr != null && targetMgr.PrimaryBeacon != null)
            {
                var b = targetMgr.PrimaryBeacon;
                LogTelemetry($"M1 Target: Mode={b.MovementMode}, Visible={b.IsVisible}, Pos=({b.CurrentPosition.x:F2}, {b.CurrentPosition.y:F2}, {b.CurrentPosition.z:F2})");
            }
            if (rig != null)
            {
                LogTelemetry($"M2 Camera: Pan={rig.CurrentPanAngle:F2}°, Tilt={rig.CurrentTiltAngle:F2}°, TargetPan={rig.TargetPanAngle:F2}°, TargetTilt={rig.TargetTiltAngle:F2}°");
            }
            if (dist != null)
            {
                LogTelemetry($"M2 Disturb: Vib={dist.VibrationEnabled}, Jitter={dist.JitterEnabled}, Haze={dist.HazeEnabled}, Blur={dist.BlurEnabled}, Noise={dist.SensorNoiseEnabled}");
            }
            if (bridge != null)
            {
                var d = bridge.LatestResult;
                LogTelemetry($"M3 DetBridge ({bridge.gameObject.name} id={bridge.GetInstanceID()}): Connected={bridge.IsClientConnected}, TotalRecv={bridge.TotalResultsReceived}, Det={d.detected}, Pt=({d.pixelCenterX:F1}, {d.pixelCenterY:F1}), Conf={d.confidence:F2}, Latency={d.latencyMs:F1}ms");
            }
            if (tracker != null)
            {
                var t = tracker.LatestResult;
                var bridgeField = typeof(OPTINAVKalmanTracker).GetField("detectionBridge", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var assignedBridge = bridgeField != null ? bridgeField.GetValue(tracker) as OPTINAVDetectionBridge : null;
                var assignedBridgeName = assignedBridge != null ? $"{assignedBridge.gameObject.name}(id={assignedBridge.GetInstanceID()})" : "NULL";
                LogTelemetry($"M4 Tracker ({tracker.gameObject.name} id={tracker.GetInstanceID()} enabled={tracker.enabled} active={tracker.gameObject.activeInHierarchy} bridge={assignedBridgeName}): State={t.state}, HasTrack={t.hasTrack}, HasMeas={t.hasMeasurement}, TrackedPt=({t.trackedPixelX:F1}, {t.trackedPixelY:F1}), PredPt=({t.predictedPixelX:F1}, {t.predictedPixelY:F1}), Misses={t.consecutiveMisses}");
            }
            if (controller != null)
            {
                var cmdPan = (rig != null) ? rig.TargetPanAngle : 0f;
                var cmdTilt = (rig != null) ? rig.TargetTiltAngle : 0f;
                LogTelemetry($"M5 Control ({controller.gameObject.name} id={controller.GetInstanceID()}): State={controller.ActiveTrackingState}, Aligned={controller.IsTargetAligned}, PxErr={controller.PixelErrorTotal:F1}px, AngErr={controller.AngularErrorTotal:F2}°, CmdPan={cmdPan:F2}°, CmdTilt={cmdTilt:F2}°");
            }
            LogTelemetry("---------------------------------");
        }

        private static void ResetPipelineBaseline(
            OPTINAVTargetManager targetMgr, OPTINAVCameraRig rig,
            OPTINAVKalmanTracker tracker, OPTINAVCoarseAlignmentController controller,
            OPTINAVDisturbanceController dist)
        {
            if (dist != null)
            {
                dist.VibrationEnabled = false;
                dist.JitterEnabled = false;
                dist.HazeEnabled = false;
                dist.BlurEnabled = false;
                dist.SensorNoiseEnabled = false;
                dist.VisibilityLossEnabled = false;
            }
            if (targetMgr != null)
            {
                targetMgr.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
                targetMgr.SetPrimaryTargetVisibility(true);
                targetMgr.SetDistractorsActive(false);
            }
            if (tracker != null)
            {
                tracker.ResetTracker();
            }
            if (controller != null)
            {
                controller.ResetControllers();
                var cmdPanField = typeof(OPTINAVCoarseAlignmentController).GetField("commandedPanAngle", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                var cmdTiltField = typeof(OPTINAVCoarseAlignmentController).GetField("commandedTiltAngle", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                cmdPanField?.SetValue(controller, 0f);
                cmdTiltField?.SetValue(controller, 0f);
            }
            if (rig != null)
            {
                rig.ResetOrientation();
                rig.SetTargetAngles(0f, 0f);
            }
        }

        private static IEnumerator ExecuteTestSequenceCoroutine(string testName)
        {
            LogTelemetry($"========================================================");
            LogTelemetry($"STARTING INTEGRATION TEST: {testName.ToUpper()}");
            LogTelemetry($"========================================================");

            var bridge = OPTINAVDetectionBridge.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVDetectionBridge>();
            var tracker = OPTINAVKalmanTracker.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVKalmanTracker>();
            var controller = UnityEngine.Object.FindFirstObjectByType<OPTINAVCoarseAlignmentController>();
            var rig = UnityEngine.Object.FindFirstObjectByType<OPTINAVCameraRig>();
            var targetMgr = OPTINAVTargetManager.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVTargetManager>();
            var dist = OPTINAVDisturbanceController.Instance ?? UnityEngine.Object.FindFirstObjectByType<OPTINAVDisturbanceController>();

            if (targetMgr == null || rig == null || tracker == null || controller == null)
            {
                LogTelemetry($"[ERROR] Subsystems not found. Scene may not be in Play Mode.");
                yield break;
            }

            EnsurePipelineConnected(bridge, tracker, controller);

            // Phase 0: Reset to neutral baseline
            ResetPipelineBaseline(targetMgr, rig, tracker, controller, dist);
            yield return new WaitForSeconds(1.0f);

            switch (testName.ToLower())
            {
                case "all":
                    yield return ExecuteTest1Stationary(targetMgr, rig, bridge, tracker, controller);
                    yield return new WaitForSeconds(1.5f);
                    yield return ExecuteTest2Moving(targetMgr, rig, bridge, tracker, controller);
                    yield return new WaitForSeconds(1.5f);
                    yield return ExecuteTest3Disturbance(targetMgr, rig, bridge, tracker, controller, dist);
                    yield return new WaitForSeconds(1.5f);
                    yield return ExecuteTest4Dropout(targetMgr, rig, bridge, tracker, controller);
                    yield return new WaitForSeconds(1.5f);
                    yield return ExecuteTest5Reacquisition(targetMgr, rig, bridge, tracker, controller);
                    break;

                case "test1_stationary":
                case "stationary":
                    yield return ExecuteTest1Stationary(targetMgr, rig, bridge, tracker, controller);
                    break;

                case "test2_moving":
                case "moving":
                    yield return ExecuteTest2Moving(targetMgr, rig, bridge, tracker, controller);
                    break;

                case "test3_disturbance":
                case "disturbance":
                    yield return ExecuteTest3Disturbance(targetMgr, rig, bridge, tracker, controller, dist);
                    break;

                case "test4_dropout":
                case "dropout":
                    yield return ExecuteTest4Dropout(targetMgr, rig, bridge, tracker, controller);
                    break;

                case "test5_reacquisition":
                case "reacquisition":
                    yield return ExecuteTest5Reacquisition(targetMgr, rig, bridge, tracker, controller);
                    break;
            }

            LogTelemetry($"========================================================");
            LogTelemetry($"INTEGRATION TEST RUN COMPLETE: {testName.ToUpper()}");
            LogTelemetry($"========================================================");
        }

        private static IEnumerator ExecuteTest1Stationary(
            OPTINAVTargetManager targetMgr, OPTINAVCameraRig rig,
            OPTINAVDetectionBridge bridge, OPTINAVKalmanTracker tracker,
            OPTINAVCoarseAlignmentController controller)
        {
            LogTelemetry("--- TEST 1: STATIONARY BEACON VALIDATION ---");
            ResetPipelineBaseline(targetMgr, rig, tracker, controller, null);
            for (int i = 0; i < 10; i++)
            {
                yield return new WaitForSeconds(0.1f);
                var d = bridge.LatestResult;
                var t = tracker.LatestResult;
                LogTelemetry($"[Baseline +{(i+1)*0.1f:F1}s] Det={d.detected}(pt={d.pixelCenterX:F1},{d.pixelCenterY:F1}), M4={t.state}(pt={t.trackedPixelX:F1},{t.trackedPixelY:F1}), M5_State={controller.ActiveTrackingState}, M5_Err={controller.PixelErrorTotal:F1}px, Gimbal=({rig.CurrentPanAngle:F2}°, {rig.CurrentTiltAngle:F2}°), TargetAngles=({rig.TargetPanAngle:F2}°, {rig.TargetTiltAngle:F2}°)");
            }

            float initialPan = rig.CurrentPanAngle;
            float initialTilt = rig.CurrentTiltAngle;
            float initialPixelErr = controller.PixelErrorTotal;
            LogTelemetry($"Initial Baseline: Pan={initialPan:F2}°, Tilt={initialTilt:F2}°, PxErr={initialPixelErr:F1}px");

            float duration = 6.0f;
            float elapsed = 0f;
            bool detectedSeen = false;
            bool trackingStateAchieved = false;
            float finalPixelErr = 999f;
            float finalPan = 0f;
            float finalTilt = 0f;

            while (elapsed < duration)
            {
                yield return new WaitForSeconds(0.5f);
                elapsed += 0.5f;

                var d = bridge.LatestResult;
                var t = tracker.LatestResult;
                if (d.detected) detectedSeen = true;
                if (t.state == TrackingState.TRACKING) trackingStateAchieved = true;

                finalPixelErr = controller.PixelErrorTotal;
                finalPan = rig.CurrentPanAngle;
                finalTilt = rig.CurrentTiltAngle;

                LogTelemetry($"[{elapsed:F1}s] Det={d.detected}(pt={d.pixelCenterX:F1},{d.pixelCenterY:F1}), M4={t.state}(pt={t.trackedPixelX:F1},{t.trackedPixelY:F1}), M5_Err={finalPixelErr:F1}px, Gimbal=({finalPan:F2}°, {finalTilt:F2}°), Aligned={controller.IsTargetAligned}");
            }

            bool pass = detectedSeen && trackingStateAchieved && controller.IsTargetAligned;
            LogTelemetry($"TEST 1 RESULT: {(pass ? "PASS" : "FAIL")}");
            LogTelemetry($"  Criteria: Detected={detectedSeen}, TrackingState={trackingStateAchieved}, Aligned={controller.IsTargetAligned}, FinalErr={finalPixelErr:F1}px, FinalAngles=({finalPan:F2}°, {finalTilt:F2}°)");
        }

        private static IEnumerator ExecuteTest2Moving(
            OPTINAVTargetManager targetMgr, OPTINAVCameraRig rig,
            OPTINAVDetectionBridge bridge, OPTINAVKalmanTracker tracker,
            OPTINAVCoarseAlignmentController controller)
        {
            LogTelemetry("--- TEST 2: MOVING BEACON (LINEAR / CIRCULAR) VALIDATION ---");
            targetMgr.SetPrimaryMovementMode(BeaconMovementMode.Linear);
            targetMgr.SetPrimaryTargetVisibility(true);
            yield return new WaitForSeconds(0.5f);

            float duration = 8.0f;
            float elapsed = 0f;
            int trackingFrames = 0;
            int totalFrames = 0;
            float minPan = float.MaxValue, maxPan = float.MinValue;

            while (elapsed < duration)
            {
                yield return new WaitForSeconds(0.5f);
                elapsed += 0.5f;
                totalFrames++;

                var d = bridge.LatestResult;
                var t = tracker.LatestResult;
                if (t.state == TrackingState.TRACKING) trackingFrames++;

                float pan = rig.CurrentPanAngle;
                float tilt = rig.CurrentTiltAngle;
                minPan = Mathf.Min(minPan, pan);
                maxPan = Mathf.Max(maxPan, pan);

                LogTelemetry($"[{elapsed:F1}s] TargetPos={targetMgr.PrimaryTargetPosition.x:F1}, M3Det={d.detected}, M4State={t.state}, M5Err={controller.PixelErrorTotal:F1}px, CamAngles=({pan:F2}°, {tilt:F2}°)");
            }

            float panRange = maxPan - minPan;
            bool cameraFollowed = panRange > 2.0f;
            bool trackingActive = (float)trackingFrames / totalFrames >= 0.75f;
            bool pass = cameraFollowed && trackingActive;

            LogTelemetry($"TEST 2 RESULT: {(pass ? "PASS" : "FAIL")}");
            LogTelemetry($"  Criteria: TrackingActive={trackingActive} ({trackingFrames}/{totalFrames}), CameraFollowed={cameraFollowed} (PanRange={panRange:F2}°)");
            targetMgr.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
        }

        private static IEnumerator ExecuteTest3Disturbance(
            OPTINAVTargetManager targetMgr, OPTINAVCameraRig rig,
            OPTINAVDetectionBridge bridge, OPTINAVKalmanTracker tracker,
            OPTINAVCoarseAlignmentController controller,
            OPTINAVDisturbanceController dist)
        {
            LogTelemetry("--- TEST 3: CAMERA / PLATFORM DISTURBANCE VALIDATION ---");
            targetMgr.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
            targetMgr.SetPrimaryTargetVisibility(true);

            if (dist != null)
            {
                dist.VibrationEnabled = true;
                dist.JitterEnabled = true;
                dist.HazeEnabled = true;
                dist.BlurEnabled = true;
                dist.BlurFactor = 0.35f;
                dist.SensorNoiseEnabled = true;
                dist.SensorNoiseLevel = 0.20f;
            }

            LogTelemetry($"Disturbances Enabled: Vibration=ON, Jitter=ON, Haze=ON, Blur=0.35, Noise=0.20");

            float duration = 6.0f;
            float elapsed = 0f;
            int detectedFrames = 0;
            int trackingFrames = 0;
            int totalFrames = 0;

            while (elapsed < duration)
            {
                yield return new WaitForSeconds(0.5f);
                elapsed += 0.5f;
                totalFrames++;

                var d = bridge.LatestResult;
                var t = tracker.LatestResult;
                if (d.detected) detectedFrames++;
                if (t.state == TrackingState.TRACKING || t.state == TrackingState.COASTING) trackingFrames++;

                LogTelemetry($"[{elapsed:F1}s] Det={d.detected}(conf={d.confidence:F2}), State={t.state}, M5Err={controller.PixelErrorTotal:F1}px, Gimbal=({rig.CurrentPanAngle:F2}°, {rig.CurrentTiltAngle:F2}°)");
            }

            if (dist != null)
            {
                dist.VibrationEnabled = false;
                dist.JitterEnabled = false;
                dist.HazeEnabled = false;
                dist.BlurEnabled = false;
                dist.SensorNoiseEnabled = false;
            }

            bool detRateAcceptable = (float)detectedFrames / totalFrames >= 0.60f;
            bool trackingFunctional = (float)trackingFrames / totalFrames >= 0.75f;
            bool pass = detRateAcceptable && trackingFunctional;

            LogTelemetry($"TEST 3 RESULT: {(pass ? "PASS" : "FAIL")}");
            LogTelemetry($"  Criteria: DetectionRate={detectedFrames}/{totalFrames}, TrackingFunctional={trackingFrames}/{totalFrames}");
        }

        private static IEnumerator ExecuteTest4Dropout(
            OPTINAVTargetManager targetMgr, OPTINAVCameraRig rig,
            OPTINAVDetectionBridge bridge, OPTINAVKalmanTracker tracker,
            OPTINAVCoarseAlignmentController controller)
        {
            LogTelemetry("--- TEST 4: SHORT BEACON DROPOUT VALIDATION ---");
            targetMgr.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
            targetMgr.SetPrimaryTargetVisibility(true);
            yield return new WaitForSeconds(2.0f);

            var preDropoutTrack = tracker.LatestResult;
            LogTelemetry($"[PRE-DROPOUT] State={preDropoutTrack.state}, HasMeas={preDropoutTrack.hasMeasurement}, Pt=({preDropoutTrack.trackedPixelX:F1}, {preDropoutTrack.trackedPixelY:F1})");

            // Cause dropout
            LogTelemetry(">>> TRIGGERING BEACON VISIBILITY LOSS (DROPOUT) <<<");
            bool enteredCoasting = false;
            Action<TrackingState, TrackingState> stateListener = (prev, next) =>
            {
                if (next == TrackingState.COASTING)
                {
                    enteredCoasting = true;
                }
            };
            tracker.OnTrackingStateChanged += stateListener;

            targetMgr.SetPrimaryTargetVisibility(false);

            bool noMeasurementDuringCoasting = true;
            bool staleCoordNotUsed = true;
            float dropoutDuration = 2.0f;
            float elapsed = 0f;
            float logTimer = 0f;

            while (elapsed < dropoutDuration)
            {
                yield return new WaitForSeconds(0.04f);
                elapsed += 0.04f;
                logTimer += 0.04f;

                var d = bridge.LatestResult;
                var t = tracker.LatestResult;

                if (t.state == TrackingState.COASTING)
                {
                    enteredCoasting = true;
                    if (t.hasMeasurement) noMeasurementDuringCoasting = false;
                    // Stale coordinates check: M3 should be -1,-1
                    if (d.detected) staleCoordNotUsed = false;
                }

                if (logTimer >= 0.25f || t.state == TrackingState.COASTING)
                {
                    logTimer = 0f;
                    LogTelemetry($"[DROPOUT {elapsed:F2}s] M3_Det={d.detected}(pt={d.pixelCenterX:F1},{d.pixelCenterY:F1}), M4_State={t.state}, M4_HasMeas={t.hasMeasurement}, M4_Tracked=({t.trackedPixelX:F1},{t.trackedPixelY:F1}), Misses={t.consecutiveMisses}");
                }
            }

            tracker.OnTrackingStateChanged -= stateListener;

            // Restore visibility
            LogTelemetry(">>> RESTORING BEACON VISIBILITY <<<");
            targetMgr.SetPrimaryTargetVisibility(true);
            yield return new WaitForSeconds(1.5f);

            var postTrack = tracker.LatestResult;
            var postDet = bridge.LatestResult;
            LogTelemetry($"[POST-DROPOUT] M3_Det={postDet.detected}, M4_State={postTrack.state}, M4_HasMeas={postTrack.hasMeasurement}, Misses={postTrack.consecutiveMisses}");

            bool pass = enteredCoasting && noMeasurementDuringCoasting && (postTrack.state == TrackingState.TRACKING);
            LogTelemetry($"TEST 4 RESULT: {(pass ? "PASS" : "FAIL")}");
            LogTelemetry($"  Criteria: EnteredCoasting={enteredCoasting}, NoMeasDuringCoasting={noMeasurementDuringCoasting}, StaleNotUsed={staleCoordNotUsed}, ResumedTracking={postTrack.state == TrackingState.TRACKING}");
        }

        private static IEnumerator ExecuteTest5Reacquisition(
            OPTINAVTargetManager targetMgr, OPTINAVCameraRig rig,
            OPTINAVDetectionBridge bridge, OPTINAVKalmanTracker tracker,
            OPTINAVCoarseAlignmentController controller)
        {
            LogTelemetry("--- TEST 5: REACQUISITION VALIDATION ---");
            targetMgr.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
            targetMgr.SetPrimaryTargetVisibility(true);
            yield return new WaitForSeconds(1.5f);

            LogTelemetry(">>> CAUSING DROPOUT FOR REACQUISITION TEST <<<");
            targetMgr.SetPrimaryTargetVisibility(false);
            yield return new WaitForSeconds(1.5f);

            var duringTrack = tracker.LatestResult;
            LogTelemetry($"During Dropout: M4_State={duringTrack.state}, Misses={duringTrack.consecutiveMisses}");

            LogTelemetry(">>> RESTORING BEACON TARGET <<<");
            float t0 = Time.realtimeSinceStartup;
            targetMgr.SetPrimaryTargetVisibility(true);

            bool reacquired = false;
            float reacquisitionTime = -1f;
            float timeout = 4.0f;
            float elapsed = 0f;

            while (elapsed < timeout)
            {
                yield return new WaitForSeconds(0.1f);
                elapsed += 0.1f;

                var d = bridge.LatestResult;
                var t = tracker.LatestResult;

                if (t.state == TrackingState.TRACKING && d.detected)
                {
                    reacquired = true;
                    reacquisitionTime = Time.realtimeSinceStartup - t0;
                    LogTelemetry($"[REACQUIRED in {reacquisitionTime:F3}s] M3_Det={d.detected}, M4_State={t.state}, M4_MeasAccepted={t.hasMeasurement}, MissesReset={t.consecutiveMisses == 0}");
                    break;
                }
            }

            bool pass = reacquired && controller.ControlEnabled;
            LogTelemetry($"TEST 5 RESULT: {(pass ? "PASS" : "FAIL")}");
            LogTelemetry($"  Criteria: Reacquired={reacquired}, Time={reacquisitionTime:F3}s, M5_Active={controller.ControlEnabled}");
        }
    }

    /// <summary>
    /// Editor coroutine runner helper that supports nested IEnumerators and WaitForSeconds delays.
    /// </summary>
    public static class EditorCoroutineRunner
    {
        public static void StartCoroutine(IEnumerator coroutine)
        {
            var stack = new System.Collections.Generic.Stack<IEnumerator>();
            stack.Push(coroutine);
            double waitDeadline = 0;

            EditorApplication.CallbackFunction update = null;
            update = () =>
            {
                try
                {
                    if (EditorApplication.timeSinceStartup < waitDeadline)
                        return;

                    while (stack.Count > 0)
                    {
                        var top = stack.Peek();
                        if (top.MoveNext())
                        {
                            object current = top.Current;
                            if (current is IEnumerator nested)
                            {
                                stack.Push(nested);
                            }
                            else if (current is WaitForSeconds wfs)
                            {
                                var field = typeof(WaitForSeconds).GetField("m_Seconds", System.Reflection.BindingFlags.NonPublic | System.Reflection.BindingFlags.Instance);
                                float sec = field != null ? (float)field.GetValue(wfs) : 0f;
                                waitDeadline = EditorApplication.timeSinceStartup + sec;
                                return;
                            }
                            else if (current is WaitForSecondsRealtime wfsr)
                            {
                                waitDeadline = EditorApplication.timeSinceStartup + wfsr.waitTime;
                                return;
                            }
                            else if (current is float sec)
                            {
                                waitDeadline = EditorApplication.timeSinceStartup + sec;
                                return;
                            }
                            else
                            {
                                return;
                            }
                        }
                        else
                        {
                            stack.Pop();
                        }
                    }

                    EditorApplication.update -= update;
                }
                catch (Exception ex)
                {
                    Debug.LogError($"[EditorCoroutineRunner] Exception in coroutine: {ex}");
                    EditorApplication.update -= update;
                }
            };
            EditorApplication.update += update;
        }
    }
}
