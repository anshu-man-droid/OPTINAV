using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using OPTINAV.CameraSystem;
using OPTINAV.Disturbances;
using OPTINAV.Targets;

namespace OPTINAV.Detection
{
    /// <summary>
    /// Test record for a single M3 validation test phase.
    /// Stores all metrics required by the Member 3 specification.
    /// </summary>
    [Serializable]
    public struct TestPhaseResult
    {
        public string testName;
        public int framesProcessed;
        public int detections;
        public int missedDetections;
        public int falseDetections;
        public float detectionRate;
        public float falsePositiveRate;
        public float averageConfidence;
        public float averageDetectionLatencyMs;
        public float averagePixelError;
    }

    /// <summary>
    /// Automated test runner for Member 3 Beacon Detection validation.
    /// Steps through the complete Test Matrix:
    /// - TEST A: Clean image (baseline)
    /// - TEST B: Optical blur
    /// - TEST C: Sensor noise
    /// - TEST D: Atmospheric haze
    /// - TEST E: Beacon visibility dropout
    /// - TEST F: Combined blur + noise + haze
    /// - TEST G: Pan/tilt camera movement
    /// - TEST H: Distractor present (orange distractors)
    /// - TEST I: Multiple candidates (multiple cyan targets)
    /// Evaluates detector output against ground-truth for benchmarking and logs full telemetry.
    /// </summary>
    [DisallowMultipleComponent]
    public class OPTINAVM3TestRunner : MonoBehaviour
    {
        [Header("Test Configuration")]
        [Tooltip("Automatically execute test suite on Start")]
        [SerializeField] private bool runAutomatedTest = true;

        [Tooltip("Duration in seconds for each test phase")]
        [SerializeField] private float phaseDuration = 4.0f;

        [Header("Subsystem References")]
        [SerializeField] private OPTINAVDisturbanceController disturbanceController;
        [SerializeField] private OPTINAVTargetManager targetManager;
        [SerializeField] private OPTINAVCameraRig cameraRig;
        [SerializeField] private OPTINAVCameraController cameraController;
        [SerializeField] private OPTINAVFrameCapture frameCapture;
        [SerializeField] private OPTINAVDetectionBridge detectionBridge;

        [Header("Test Results Summary")]
        [SerializeField] private List<TestPhaseResult> testResults = new List<TestPhaseResult>();

        public IReadOnlyList<TestPhaseResult> Results => testResults;
        public bool IsTestComplete { get; private set; } = false;

        private void Awake()
        {
            FindReferences();
        }

        private void FindReferences()
        {
            if (disturbanceController == null) disturbanceController = FindFirstObjectByType<OPTINAVDisturbanceController>();
            if (targetManager == null) targetManager = FindFirstObjectByType<OPTINAVTargetManager>();
            if (cameraRig == null) cameraRig = FindFirstObjectByType<OPTINAVCameraRig>();
            if (cameraController == null) cameraController = FindFirstObjectByType<OPTINAVCameraController>();
            if (frameCapture == null) frameCapture = FindFirstObjectByType<OPTINAVFrameCapture>();
            if (detectionBridge == null) detectionBridge = FindFirstObjectByType<OPTINAVDetectionBridge>();

            if (detectionBridge == null && gameObject.GetComponent<OPTINAVDetectionBridge>() == null)
            {
                detectionBridge = gameObject.AddComponent<OPTINAVDetectionBridge>();
            }

            if (gameObject.GetComponent<OPTINAVDetectionVisualizer>() == null)
            {
                gameObject.AddComponent<OPTINAVDetectionVisualizer>();
            }
        }

        private void Start()
        {
            if (runAutomatedTest)
            {
                StartCoroutine(RunFullM3TestMatrix());
            }
        }

        private IEnumerator RunFullM3TestMatrix()
        {
            FindReferences();
            testResults.Clear();
            IsTestComplete = false;

            Debug.Log("[M3 TEST RUNNER] ========================================================");
            Debug.Log("[M3 TEST RUNNER] STARTING OPTINAV MEMBER 3 VALIDATION TEST SUITE");
            Debug.Log("[M3 TEST RUNNER] ========================================================");

            // Wait 2.5 seconds for frame streaming and detection bridge to establish
            yield return new WaitForSecondsRealtime(2.5f);

            // Phase 0: Reset state
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
                targetManager.SetPrimaryTargetVisibility(true);
                targetManager.SetDistractorsActive(false);
            }
            if (cameraRig != null) cameraRig.ResetOrientation();
            yield return new WaitForSecondsRealtime(1.0f);

            // ----------------------------------------------------
            // TEST A: Clean Image (Baseline)
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST A: CLEAN IMAGE (ALL DISTURBANCES OFF) <<<");
            SetAllDisturbances(false);
            yield return StartCoroutine(ExecutePhase("TEST A: Clean Image", phaseDuration));

            // ----------------------------------------------------
            // TEST B: Optical Blur
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST B: OPTICAL BLUR (Factor = 0.5) <<<");
            SetAllDisturbances(false);
            if (disturbanceController != null)
            {
                disturbanceController.BlurFactor = 0.5f;
                disturbanceController.BlurEnabled = true;
            }
            yield return StartCoroutine(ExecutePhase("TEST B: Optical Blur", phaseDuration));

            // ----------------------------------------------------
            // TEST C: Sensor Noise
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST C: SENSOR NOISE (Level = 0.35) <<<");
            SetAllDisturbances(false);
            if (disturbanceController != null)
            {
                disturbanceController.SensorNoiseLevel = 0.35f;
                disturbanceController.SensorNoiseEnabled = true;
            }
            yield return StartCoroutine(ExecutePhase("TEST C: Sensor Noise", phaseDuration));

            // ----------------------------------------------------
            // TEST D: Atmospheric Haze
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST D: ATMOSPHERIC HAZE <<<");
            SetAllDisturbances(false);
            if (disturbanceController != null)
            {
                disturbanceController.HazeEnabled = true;
            }
            yield return StartCoroutine(ExecutePhase("TEST D: Atmospheric Haze", phaseDuration));

            // ----------------------------------------------------
            // TEST E: Beacon Visibility Loss (Dropout)
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST E: BEACON DROPOUT (VISIBILITY LOSS) <<<");
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetPrimaryTargetVisibility(false);
            }
            yield return StartCoroutine(ExecutePhase("TEST E: Beacon Dropout", phaseDuration));
            if (targetManager != null)
            {
                targetManager.SetPrimaryTargetVisibility(true);
            }
            yield return new WaitForSecondsRealtime(0.5f);

            // ----------------------------------------------------
            // TEST F: Combined Blur + Noise + Haze
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST F: COMBINED DISTURBANCES (BLUR + NOISE + HAZE) <<<");
            SetAllDisturbances(false);
            if (disturbanceController != null)
            {
                disturbanceController.BlurFactor = 0.4f;
                disturbanceController.BlurEnabled = true;
                disturbanceController.SensorNoiseLevel = 0.3f;
                disturbanceController.SensorNoiseEnabled = true;
                disturbanceController.HazeEnabled = true;
            }
            yield return StartCoroutine(ExecutePhase("TEST F: Combined Disturbances", phaseDuration));

            // ----------------------------------------------------
            // TEST G: Pan/Tilt Camera Movement & Target Motion
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST G: PAN/TILT SLEWING & TARGET MOTION <<<");
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Linear);
            }
            if (cameraRig != null)
            {
                cameraRig.SetTargetAngles(12f, -6f);
            }
            yield return StartCoroutine(ExecutePhase("TEST G: Motion & Pan/Tilt", phaseDuration));
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
            }
            if (cameraRig != null)
            {
                cameraRig.ResetOrientation();
            }
            yield return new WaitForSecondsRealtime(0.5f);

            // ----------------------------------------------------
            // TEST H: Distractor Present (Orange distractors)
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST H: DISTRACTORS PRESENT (ORANGE OBJECTS) <<<");
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetDistractorsActive(true);
                targetManager.SetPrimaryTargetVisibility(true);
            }
            yield return StartCoroutine(ExecutePhase("TEST H: Distractors Present", phaseDuration));
            if (targetManager != null)
            {
                targetManager.SetDistractorsActive(false);
            }
            yield return new WaitForSecondsRealtime(0.5f);

            // ----------------------------------------------------
            // TEST I: Multiple Candidates (Secondary cyan target)
            // ----------------------------------------------------
            Debug.Log("\n[M3 TEST RUNNER] >>> EXECUTING TEST I: MULTIPLE CANDIDATES <<<");
            SetAllDisturbances(false);

            // Spawn temporary secondary cyan object
            GameObject secondaryCandidate = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            secondaryCandidate.name = "OPTINAV_SecondaryCandidate_Cyan";
            secondaryCandidate.transform.position = new Vector3(8f, 15f, 60f);
            secondaryCandidate.transform.localScale = new Vector3(0.9f, 0.9f, 0.9f);
            var secRenderer = secondaryCandidate.GetComponent<MeshRenderer>();
            if (targetManager != null && targetManager.PrimaryBeacon != null)
            {
                var primaryRenderer = targetManager.PrimaryBeacon.GetComponentInChildren<MeshRenderer>();
                if (primaryRenderer != null && secRenderer != null)
                {
                    secRenderer.sharedMaterial = primaryRenderer.sharedMaterial;
                }
            }

            yield return StartCoroutine(ExecutePhase("TEST I: Multiple Candidates", phaseDuration));

            Destroy(secondaryCandidate);
            yield return new WaitForSecondsRealtime(0.5f);

            // ----------------------------------------------------
            // FINALIZE & PRINT SUMMARY REPORT
            // ----------------------------------------------------
            SetAllDisturbances(false);
            if (targetManager != null)
            {
                targetManager.SetPrimaryMovementMode(BeaconMovementMode.Stationary);
                targetManager.SetPrimaryTargetVisibility(true);
                targetManager.SetDistractorsActive(false);
            }
            if (cameraRig != null) cameraRig.ResetOrientation();

            LogCompleteSummaryReport();
            IsTestComplete = true;
        }

        private IEnumerator ExecutePhase(string phaseName, float duration)
        {
            float timer = 0f;
            int framesProcessed = 0;
            int detections = 0;
            int missedDetections = 0;
            int falseDetections = 0;
            int trueNegatives = 0;

            float confidenceSum = 0f;
            float latencySum = 0f;
            float pixelErrorSum = 0f;
            int validErrorCount = 0;

            long lastProcessedFrameId = -1;

            while (timer < duration)
            {
                timer += Time.unscaledDeltaTime;

                if (detectionBridge != null)
                {
                    DetectionResult res = detectionBridge.LatestResult;
                    if (res.frameId != lastProcessedFrameId && res.frameId > 0)
                    {
                        lastProcessedFrameId = res.frameId;
                        framesProcessed++;

                        // Evaluate against ground-truth
                        bool targetShouldBeVisible = targetManager != null && targetManager.IsPrimaryTargetVisible;
                        bool inFrustum = false;
                        Vector2 gt640 = Vector2.zero;

                        if (targetShouldBeVisible && cameraController != null)
                        {
                            Vector3 worldPos = targetManager.PrimaryTargetPosition;
                            if (cameraController.IsTargetInFrustum(worldPos))
                            {
                                inFrustum = true;
                                Vector2 vp = cameraController.WorldToNormalizedViewportPoint(worldPos);
                                gt640 = new Vector2(vp.x * 640.0f, (1.0f - vp.y) * 360.0f);
                            }
                        }

                        if (targetShouldBeVisible && inFrustum)
                        {
                            // Ground truth: TARGET IS PRESENT
                            if (res.detected)
                            {
                                detections++; // True Positive
                                confidenceSum += res.confidence;
                                latencySum += res.latencyMs;

                                float err = Vector2.Distance(new Vector2(res.pixelCenterX, res.pixelCenterY), gt640);
                                pixelErrorSum += err;
                                validErrorCount++;
                            }
                            else
                            {
                                missedDetections++; // False Negative
                            }
                        }
                        else
                        {
                            // Ground truth: TARGET IS ABSENT / DROPOUT
                            if (res.detected)
                            {
                                falseDetections++; // False Positive
                                confidenceSum += res.confidence;
                                latencySum += res.latencyMs;
                            }
                            else
                            {
                                trueNegatives++; // True Negative
                            }
                        }
                    }
                }

                yield return null;
            }

            int positiveGroundTruth = detections + missedDetections;
            float detRate = positiveGroundTruth > 0 ? ((float)detections / positiveGroundTruth * 100f) : (falseDetections == 0 ? 100f : 0f);
            float fpRate = (framesProcessed > 0) ? ((float)falseDetections / framesProcessed * 100f) : 0f;
            float avgConf = (detections + falseDetections) > 0 ? (confidenceSum / (detections + falseDetections)) : 0f;
            float avgLat = (detections + falseDetections) > 0 ? (latencySum / (detections + falseDetections)) : 0f;
            float avgErr = validErrorCount > 0 ? (pixelErrorSum / validErrorCount) : -1f;

            TestPhaseResult phaseRes = new TestPhaseResult
            {
                testName = phaseName,
                framesProcessed = framesProcessed,
                detections = detections,
                missedDetections = missedDetections,
                falseDetections = falseDetections,
                detectionRate = detRate,
                falsePositiveRate = fpRate,
                averageConfidence = avgConf,
                averageDetectionLatencyMs = avgLat,
                averagePixelError = avgErr
            };

            testResults.Add(phaseRes);

            Debug.Log($"[M3 TEST RESULT] {phaseName}:");
            Debug.Log($"    Frames: {framesProcessed} | Detections: {detections} | Missed: {missedDetections} | False Det: {falseDetections}");
            Debug.Log($"    Det Rate: {detRate:F1}% | FP Rate: {fpRate:F1}% | Avg Conf: {avgConf:F3} | Avg Latency: {avgLat:F1}ms | Avg Pixel Err: {(avgErr >= 0 ? $"{avgErr:F2} px" : "N/A")}");
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

        private void LogCompleteSummaryReport()
        {
            Debug.Log("\n========================================================================================");
            Debug.Log("                  OPTINAV MEMBER 3 — BEACON DETECTION VALIDATION REPORT                  ");
            Debug.Log("========================================================================================");
            Debug.Log(string.Format("{0,-30} | {1,6} | {2,4} | {3,4} | {4,4} | {5,7} | {6,6} | {7,8} | {8,8}",
                "Test Phase", "Frames", "Det", "Miss", "FP", "DetRate", "Conf", "Latency", "PixelErr"));
            Debug.Log("----------------------------------------------------------------------------------------");

            foreach (var r in testResults)
            {
                string errStr = r.averagePixelError >= 0 ? $"{r.averagePixelError:F2} px" : "N/A";
                Debug.Log(string.Format("{0,-30} | {1,6} | {2,4} | {3,4} | {4,4} | {5,6:F1}% | {6,6:F3} | {7,6:F1}ms | {8,8}",
                    r.testName, r.framesProcessed, r.detections, r.missedDetections, r.falseDetections,
                    r.detectionRate, r.averageConfidence, r.averageDetectionLatencyMs, errStr));
            }

            Debug.Log("========================================================================================\n");
        }
    }
}
