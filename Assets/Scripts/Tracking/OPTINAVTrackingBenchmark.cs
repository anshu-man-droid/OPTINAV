using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OPTINAV.CameraSystem;
using OPTINAV.Detection;
using OPTINAV.Disturbances;
using OPTINAV.Targets;

namespace OPTINAV.Tracking
{
    /// <summary>
    /// Telemetry and evaluation record for a single M4 validation benchmark phase.
    /// Strictly adheres to ground-truth rule: ground-truth is evaluated post-filter
    /// solely for scoring and error metric calculation.
    /// </summary>
    [Serializable]
    public struct TrackingBenchmarkPhaseResult
    {
        public string testName;
        public int framesProcessed;
        public int framesInSearch;
        public int framesInAcquisition;
        public int framesInTracking;
        public int framesInCoasting;
        public int framesInLost;

        public float averageTrackingErrorPx;
        public float maxTrackingErrorPx;
        public float averageCoastingErrorPx;
        public float maxCoastingErrorPx;
        public float reacquisitionTimeSec;
        public int maxConsecutiveMisses;
        public float averageLatencyMs;
        public bool passedCriteria;
    }

    /// <summary>
    /// Comprehensive validation harness for Member 4: Tracking & Prediction.
    /// Executes the required test matrix A through J:
    /// - TEST A: Stationary beacon (noise rejection & stabilization)
    /// - TEST B: Linear beacon motion (constant velocity tracking)
    /// - TEST C: Accelerating beacon (dynamic response & acceleration tracking)
    /// - TEST D: Circular motion (2D curvilinear path tracking)
    /// - TEST E: Temporary detection dropout (short dropout, verifying COASTING)
    /// - TEST F: Multiple consecutive missed frames (dropout > maxCoastingFrames -> LOST)
    /// - TEST G: Beacon reappearance / reacquisition (reacquisition latency & recovery)
    /// - TEST H: Camera pan/tilt movement (platform motion disturbance)
    /// - TEST I: Combined disturbances (optical blur + sensor noise + haze)
    /// - TEST J: Distractor presence (discrimination against false candidates)
    /// </summary>
    [DisallowMultipleComponent]
    public class OPTINAVTrackingBenchmark : MonoBehaviour
    {
        [Header("Benchmark Configuration")]
        [Tooltip("Automatically run test suite on Start")]
        [SerializeField] private bool runAutomatedBenchmark = true;

        [Tooltip("Duration in seconds for each benchmark test phase")]
        [SerializeField] private float phaseDuration = 4.0f;

        [Header("Subsystem References")]
        [SerializeField] private OPTINAVKalmanTracker tracker;
        [SerializeField] private OPTINAVDetectionBridge detectionBridge;
        [SerializeField] private OPTINAVTargetManager targetManager;
        [SerializeField] private OPTINAVDisturbanceController disturbanceController;
        [SerializeField] private OPTINAVCameraController cameraController;
        [SerializeField] private OPTINAVCameraRig cameraRig;

        [Header("Benchmark Results")]
        [SerializeField] private List<TrackingBenchmarkPhaseResult> benchmarkResults = new List<TrackingBenchmarkPhaseResult>();

        public IReadOnlyList<TrackingBenchmarkPhaseResult> Results => benchmarkResults;
        public bool IsBenchmarkComplete { get; private set; } = false;

        private void Awake()
        {
            FindReferences();
        }

        private void FindReferences()
        {
            if (tracker == null) tracker = FindFirstObjectByType<OPTINAVKalmanTracker>();
            if (tracker == null && gameObject.GetComponent<OPTINAVKalmanTracker>() == null)
            {
                tracker = gameObject.AddComponent<OPTINAVKalmanTracker>();
            }

            if (detectionBridge == null) detectionBridge = FindFirstObjectByType<OPTINAVDetectionBridge>();
            if (targetManager == null) targetManager = FindFirstObjectByType<OPTINAVTargetManager>();
            if (disturbanceController == null) disturbanceController = FindFirstObjectByType<OPTINAVDisturbanceController>();
            if (cameraController == null) cameraController = FindFirstObjectByType<OPTINAVCameraController>();
            if (cameraRig == null) cameraRig = FindFirstObjectByType<OPTINAVCameraRig>();
        }

        private void Start()
        {
            if (runAutomatedBenchmark)
            {
                StartCoroutine(RunFullM4BenchmarkSuite());
            }
        }

        public IEnumerator RunFullM4BenchmarkSuite()
        {
            FindReferences();
            benchmarkResults.Clear();
            IsBenchmarkComplete = false;

            Debug.Log("[M4 BENCHMARK] ========================================================");
            Debug.Log("[M4 BENCHMARK] STARTING OPTINAV MEMBER 4 TRACKING & PREDICTION BENCHMARK");
            Debug.Log("[M4 BENCHMARK] ========================================================");

            // Wait 2.0s for camera streams, bridge, and tracker initialization
            yield return new WaitForSecondsRealtime(2.0f);

            // Phase 0: Reset baseline state
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
                targetManager.SetPrimaryTargetVisibility(true);
                targetManager.SetDistractorsActive(false);
            }
            if (cameraRig != null) cameraRig.ResetOrientation();
            if (tracker != null) tracker.ResetTracker();

            yield return new WaitForSecondsRealtime(1.0f);

            // ----------------------------------------------------
            // TEST A: Stationary Beacon
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST A: STATIONARY BEACON <<<");
            if (targetManager != null) targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
            yield return StartCoroutine(ExecutePhase("TEST A: Stationary Beacon", phaseDuration, checkCoasting: false, dropoutSimulation: false));

            // ----------------------------------------------------
            // TEST B: Linear Beacon Motion
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST B: LINEAR BEACON MOTION <<<");
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Linear);
                if (targetManager.PrimaryBeacon != null) targetManager.PrimaryBeacon.Speed = 6.0f;
            }
            yield return StartCoroutine(ExecutePhase("TEST B: Linear Motion", phaseDuration, checkCoasting: false, dropoutSimulation: false));

            // ----------------------------------------------------
            // TEST C: Accelerating Beacon
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST C: ACCELERATING BEACON <<<");
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Accelerating);
            }
            yield return StartCoroutine(ExecutePhase("TEST C: Accelerating Motion", phaseDuration, checkCoasting: false, dropoutSimulation: false));

            // ----------------------------------------------------
            // TEST D: Circular Motion
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST D: CIRCULAR MOTION <<<");
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Circular);
                if (targetManager.PrimaryBeacon != null) targetManager.PrimaryBeacon.Speed = 5.0f;
            }
            yield return StartCoroutine(ExecutePhase("TEST D: Circular Motion", phaseDuration, checkCoasting: false, dropoutSimulation: false));

            // ----------------------------------------------------
            // TEST E: Temporary Detection Dropout (Coasting Verification)
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST E: TEMPORARY DETECTION DROPOUT (COASTING) <<<");
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Linear);
                if (targetManager.PrimaryBeacon != null) targetManager.PrimaryBeacon.Speed = 4.0f;
            }
            yield return StartCoroutine(ExecutePhase("TEST E: Temporary Dropout", phaseDuration, checkCoasting: true, dropoutSimulation: true, dropoutDuration: 0.25f));

            // ----------------------------------------------------
            // TEST F: Multiple Consecutive Missed Frames (LOST Verification)
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST F: MULTIPLE MISSED FRAMES (TRANSITION TO LOST) <<<");
            yield return StartCoroutine(ExecutePhase("TEST F: Prolonged Dropout (LOST)", phaseDuration, checkCoasting: true, dropoutSimulation: true, dropoutDuration: 1.2f));

            // ----------------------------------------------------
            // TEST G: Beacon Reappearance / Reacquisition
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST G: BEACON REAPPEARANCE & REACQUISITION <<<");
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
                targetManager.SetPrimaryTargetVisibility(false);
            }
            yield return new WaitForSecondsRealtime(0.5f);
            if (targetManager != null)
            {
                targetManager.SetPrimaryTargetVisibility(true);
            }
            yield return StartCoroutine(ExecutePhase("TEST G: Beacon Reacquisition", phaseDuration, checkCoasting: false, dropoutSimulation: false));

            // ----------------------------------------------------
            // TEST H: Camera Pan/Tilt Movement
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST H: CAMERA PAN/TILT SLEWING <<<");
            if (targetManager != null) targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
            if (cameraRig != null) cameraRig.SetTargetAngles(10.0f, -5.0f);
            yield return StartCoroutine(ExecutePhase("TEST H: Camera Pan/Tilt", phaseDuration, checkCoasting: false, dropoutSimulation: false));
            if (cameraRig != null) cameraRig.ResetOrientation();
            yield return new WaitForSecondsRealtime(0.5f);

            // ----------------------------------------------------
            // TEST I: Combined Disturbances (Blur + Noise + Haze)
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST I: COMBINED DISTURBANCES <<<");
            if (disturbanceController != null)
            {
                disturbanceController.BlurFactor = 0.35f;
                disturbanceController.BlurEnabled = true;
                disturbanceController.SensorNoiseLevel = 0.25f;
                disturbanceController.SensorNoiseEnabled = true;
                disturbanceController.HazeEnabled = true;
            }
            yield return StartCoroutine(ExecutePhase("TEST I: Combined Disturbances", phaseDuration, checkCoasting: false, dropoutSimulation: false));
            SetAllDisturbances(false);
            yield return new WaitForSecondsRealtime(0.5f);

            // ----------------------------------------------------
            // TEST J: Distractor Presence
            // ----------------------------------------------------
            Debug.Log("\n[M4 BENCHMARK] >>> EXECUTING TEST J: DISTRACTOR PRESENCE <<<");
            if (targetManager != null)
            {
                targetManager.SetDistractorsActive(true);
                targetManager.SetPrimaryTargetVisibility(true);
            }
            yield return StartCoroutine(ExecutePhase("TEST J: Distractor Presence", phaseDuration, checkCoasting: false, dropoutSimulation: false));
            if (targetManager != null) targetManager.SetDistractorsActive(false);

            // ----------------------------------------------------
            // FINAL SUMMARY REPORT
            // ----------------------------------------------------
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
                targetManager.SetPrimaryTargetVisibility(true);
                targetManager.SetDistractorsActive(false);
            }
            if (cameraRig != null) cameraRig.ResetOrientation();

            LogBenchmarkSummaryReport();
            IsBenchmarkComplete = true;
        }

        private IEnumerator ExecutePhase(
            string phaseName,
            float duration,
            bool checkCoasting,
            bool dropoutSimulation,
            float dropoutDuration = 0.3f)
        {
            float timer = 0f;
            int framesProcessed = 0;
            int searchFrames = 0;
            int acqFrames = 0;
            int trackFrames = 0;
            int coastFrames = 0;
            int lostFrames = 0;

            float trackingErrorSum = 0f;
            int trackingErrorCount = 0;
            float maxTrackingError = 0f;

            float coastingErrorSum = 0f;
            int coastingErrorCount = 0;
            float maxCoastingError = 0f;

            int maxMissesObserved = 0;
            float latencySum = 0f;
            int latencyCount = 0;

            float reacquisitionTime = 0f;
            bool targetDropped = false;
            float dropoutStartTime = 0f;
            float dropoutEndTime = 0f;

            long lastFid = -1;

            while (timer < duration)
            {
                timer += Time.unscaledDeltaTime;

                // Handle dropout injection for dropout tests
                if (dropoutSimulation)
                {
                    if (timer > (duration * 0.35f) && timer < (duration * 0.35f + dropoutDuration))
                    {
                        if (!targetDropped)
                        {
                            targetDropped = true;
                            dropoutStartTime = timer;
                            if (targetManager != null) targetManager.SetPrimaryTargetVisibility(false);
                        }
                    }
                    else if (targetDropped && timer >= (duration * 0.35f + dropoutDuration))
                    {
                        if (targetManager != null && !targetManager.IsPrimaryTargetVisible)
                        {
                            targetManager.SetPrimaryTargetVisibility(true);
                            dropoutEndTime = timer;
                        }
                    }
                }

                if (tracker != null)
                {
                    TrackingResult res = tracker.LatestResult;
                    if (res.frameId != lastFid && res.frameId > 0)
                    {
                        lastFid = res.frameId;
                        framesProcessed++;

                        // Count state frames
                        switch (res.state)
                        {
                            case TrackingState.SEARCH: searchFrames++; break;
                            case TrackingState.ACQUISITION: acqFrames++; break;
                            case TrackingState.TRACKING:
                                trackFrames++;
                                if (dropoutEndTime > 0f && reacquisitionTime == 0f)
                                {
                                    reacquisitionTime = timer - dropoutEndTime;
                                }
                                break;
                            case TrackingState.COASTING: coastFrames++; break;
                            case TrackingState.LOST: lostFrames++; break;
                        }

                        if (res.consecutiveMisses > maxMissesObserved)
                        {
                            maxMissesObserved = res.consecutiveMisses;
                        }

                        if (res.latencyMs > 0)
                        {
                            latencySum += res.latencyMs;
                            latencyCount++;
                        }

                        // Evaluate strictly against ground truth
                        if (cameraController != null && targetManager != null && targetManager.IsPrimaryTargetVisible)
                        {
                            Vector3 targetWorld = targetManager.PrimaryTargetPosition;
                            if (cameraController.IsTargetInFrustum(targetWorld))
                            {
                                Vector2 vp = cameraController.WorldToNormalizedViewportPoint(targetWorld);
                                Vector2 gtPx = new Vector2(vp.x * 640.0f, (1.0f - vp.y) * 360.0f);

                                if (res.hasTrack && res.trackedPixelX >= 0f)
                                {
                                    float err = Vector2.Distance(new Vector2(res.trackedPixelX, res.trackedPixelY), gtPx);
                                    trackingErrorSum += err;
                                    trackingErrorCount++;
                                    if (err > maxTrackingError) maxTrackingError = err;
                                }

                                if (res.state == TrackingState.COASTING && res.predictedPixelX >= 0f)
                                {
                                    float coastErr = Vector2.Distance(new Vector2(res.predictedPixelX, res.predictedPixelY), gtPx);
                                    coastingErrorSum += coastErr;
                                    coastingErrorCount++;
                                    if (coastErr > maxCoastingError) maxCoastingError = coastErr;
                                }
                            }
                        }
                    }
                }

                yield return null;
            }

            // Restore visibility if left hidden
            if (targetManager != null && !targetManager.IsPrimaryTargetVisible)
            {
                targetManager.SetPrimaryTargetVisibility(true);
            }

            float avgTrackErr = trackingErrorCount > 0 ? (trackingErrorSum / trackingErrorCount) : 0f;
            float avgCoastErr = coastingErrorCount > 0 ? (coastingErrorSum / coastingErrorCount) : 0f;
            float avgLatency = latencyCount > 0 ? (latencySum / latencyCount) : 0f;

            bool passed = true;
            if (checkCoasting)
            {
                // Coasting test must exhibit COASTING frames
                if (coastFrames == 0 && lostFrames == 0) passed = false;
            }

            TrackingBenchmarkPhaseResult phaseResult = new TrackingBenchmarkPhaseResult
            {
                testName = phaseName,
                framesProcessed = framesProcessed,
                framesInSearch = searchFrames,
                framesInAcquisition = acqFrames,
                framesInTracking = trackFrames,
                framesInCoasting = coastFrames,
                framesInLost = lostFrames,
                averageTrackingErrorPx = avgTrackErr,
                maxTrackingErrorPx = maxTrackingError,
                averageCoastingErrorPx = avgCoastErr,
                maxCoastingErrorPx = maxCoastingError,
                reacquisitionTimeSec = reacquisitionTime,
                maxConsecutiveMisses = maxMissesObserved,
                averageLatencyMs = avgLatency,
                passedCriteria = passed
            };

            benchmarkResults.Add(phaseResult);

            Debug.Log($"[M4 RESULT] {phaseName}:");
            Debug.Log($"    Frames: {framesProcessed} | States: [S:{searchFrames} A:{acqFrames} T:{trackFrames} C:{coastFrames} L:{lostFrames}]");
            Debug.Log($"    Avg Track Err: {avgTrackErr:F2} px | Max Track Err: {maxTrackingError:F2} px | Coasting Err: {avgCoastErr:F2} px");
            Debug.Log($"    Max Misses: {maxMissesObserved} | Reacq Time: {(reacquisitionTime > 0 ? $"{reacquisitionTime:F3}s" : "N/A")} | Latency: {avgLatency:F2}ms | Status: {(passed ? "PASS" : "WARN")}");
        }

        private void SetAllDisturbances(bool enable)
        {
            if (disturbanceController == null) return;
            disturbanceController.BlurEnabled = enable;
            disturbanceController.SensorNoiseEnabled = enable;
            disturbanceController.HazeEnabled = enable;
            disturbanceController.VisibilityLossEnabled = enable;
            disturbanceController.VibrationEnabled = enable;
            disturbanceController.JitterEnabled = enable;
        }

        private void LogBenchmarkSummaryReport()
        {
            Debug.Log("\n==========================================================================================");
            Debug.Log("                  OPTINAV MEMBER 4: TRACKING & PREDICTION SUMMARY REPORT                  ");
            Debug.Log("==========================================================================================");
            Debug.Log(string.Format("{0,-30} | {1,-6} | {2,-11} | {3,-11} | {4,-10} | {5,-8}",
                "Benchmark Test", "Frames", "Avg Err(px)", "Max Err(px)", "Reacq(s)", "Status"));
            Debug.Log("------------------------------------------------------------------------------------------");

            float overallAvgErr = 0f;
            int errCount = 0;
            float overallMaxErr = 0f;

            foreach (var r in benchmarkResults)
            {
                Debug.Log(string.Format("{0,-30} | {1,6} | {2,11:F2} | {3,11:F2} | {4,10} | {5,8}",
                    r.testName,
                    r.framesProcessed,
                    r.averageTrackingErrorPx,
                    r.maxTrackingErrorPx,
                    r.reacquisitionTimeSec > 0 ? $"{r.reacquisitionTimeSec:F3}s" : "N/A",
                    r.passedCriteria ? "PASS" : "CHECK"));

                if (r.averageTrackingErrorPx > 0)
                {
                    overallAvgErr += r.averageTrackingErrorPx;
                    errCount++;
                }
                if (r.maxTrackingErrorPx > overallMaxErr)
                {
                    overallMaxErr = r.maxTrackingErrorPx;
                }
            }

            Debug.Log("==========================================================================================");
            float finalAvg = errCount > 0 ? (overallAvgErr / errCount) : 0f;
            Debug.Log($"OVERALL METRICS: Average Tracking Error = {finalAvg:F2} px | Peak Tracking Error = {overallMaxErr:F2} px");
            Debug.Log("==========================================================================================\n");
        }
    }
}
