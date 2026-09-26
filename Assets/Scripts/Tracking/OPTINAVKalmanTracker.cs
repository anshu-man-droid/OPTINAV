using System;
using System.Diagnostics;
using UnityEngine;
using OPTINAV.Detection;
using Debug = UnityEngine.Debug;

namespace OPTINAV.Tracking
{
    /// <summary>
    /// Core 2D Constant-Velocity Kalman Filter and Tracking Subsystem for Member 4.
    /// Consumes M3 DetectionResult from OPTINAVDetectionBridge, executes numerically stable
    /// Kalman prediction and correction, manages tracking states via OPTINAVTrackingStateMachine,
    /// handles missed detections / temporary coasting / reacquisition, and outputs TrackingResult.
    /// Strictly decoupled from M5 (PID/gimbal) and completely independent of ground truth.
    /// </summary>
    [DisallowMultipleComponent]
    public class OPTINAVKalmanTracker : MonoBehaviour
    {
        public static OPTINAVKalmanTracker Instance { get; private set; }

        [Header("Bridge Connection")]
        [Tooltip("Reference to the M3 detection bridge. If null, will be auto-located.")]
        [SerializeField] private OPTINAVDetectionBridge detectionBridge;

        [Tooltip("Automatically subscribe to detection bridge events on Start")]
        [SerializeField] private bool autoSubscribeBridge = true;

        [Header("Kalman Filter Tuning")]
        [Tooltip("Process noise spectral density / continuous acceleration variance (pixels/s^2)^2")]
        [SerializeField] private float processNoise = 120.0f;

        [Tooltip("Nominal measurement noise variance (pixels^2). R = sigma_z^2")]
        [SerializeField] private float measurementNoise = 4.0f;

        [Tooltip("Initial position uncertainty / variance when initiating track (pixels^2)")]
        [SerializeField] private float initialPositionUncertainty = 50.0f;

        [Tooltip("Initial velocity uncertainty / variance when initiating track ((pixels/s)^2)")]
        [SerializeField] private float initialVelocityUncertainty = 500.0f;

        [Tooltip("Scale measurement noise inversely with M3 detection confidence")]
        [SerializeField] private bool adaptiveMeasurementNoise = true;

        [Header("Tracking State Machine Tuning")]
        [Tooltip("Consecutive valid detections required before transitioning from ACQUISITION to TRACKING")]
        [SerializeField] private int acquisitionHitsRequired = 3;

        [Tooltip("Maximum prediction/coasting frames allowed during detection dropout before declaring track LOST")]
        [SerializeField] private int maxCoastingFrames = 8;

        [Tooltip("Consecutive missed frames during ACQUISITION before falling back to SEARCH")]
        [SerializeField] private int acquisitionMissThreshold = 2;

        [Header("Outlier Gating (Validation / Distractor Rejection)")]
        [Tooltip("Enable chi-square / Mahalanobis distance gating to reject spurious detections")]
        [SerializeField] private bool enableGating = false;

        [Tooltip("Mahalanobis distance squared threshold for 2-DOF measurement gating (default 25.0 ~ 99.99%)")]
        [SerializeField] private float gatingThreshold = 25.0f;

        [Header("Live Tracking Telemetry (Read-Only)")]
        [SerializeField] private TrackingResult latestResult = TrackingResult.CreateEmpty();
        [SerializeField] private TrackingState currentState = TrackingState.SEARCH;
        [SerializeField] private bool hasActiveTrack = false;
        [SerializeField] private float currentVelocityX = 0f;
        [SerializeField] private float currentVelocityY = 0f;
        [SerializeField] private float trackingFPS = 0f;

        // Public APIs for Member 5 (PID / Pan-Tilt Control) and Member 6 (Dashboard)
        public TrackingResult LatestResult => latestResult;
        public bool HasTrack => latestResult.hasTrack;
        public bool HasMeasurement => latestResult.hasMeasurement;
        public TrackingState State => latestResult.state;
        public Vector2 TrackedPixel => latestResult.TrackedPixel;
        public Vector2 PredictedPixel => latestResult.PredictedPixel;
        public Vector2 Velocity => latestResult.Velocity;
        public float Confidence => latestResult.confidence;
        public int ConsecutiveMisses => latestResult.consecutiveMisses;
        public float CovarianceX => latestResult.covarianceX;
        public float CovarianceY => latestResult.covarianceY;
        public float TrackingFPS => trackingFPS;
        public OPTINAVTrackingStateMachine StateMachine => stateMachine;

        // Configurable Property Accessors
        public float ProcessNoise { get => processNoise; set => processNoise = Mathf.Max(0.001f, value); }
        public float MeasurementNoise { get => measurementNoise; set => measurementNoise = Mathf.Max(0.001f, value); }
        public float InitialPositionUncertainty { get => initialPositionUncertainty; set => initialPositionUncertainty = Mathf.Max(0.001f, value); }
        public float InitialVelocityUncertainty { get => initialVelocityUncertainty; set => initialVelocityUncertainty = Mathf.Max(0.001f, value); }
        public int MaxCoastingFrames
        {
            get => maxCoastingFrames;
            set
            {
                maxCoastingFrames = Mathf.Max(1, value);
                if (stateMachine != null) stateMachine.MaxCoastingFrames = maxCoastingFrames;
            }
        }
        public int AcquisitionHitsRequired
        {
            get => acquisitionHitsRequired;
            set
            {
                acquisitionHitsRequired = Mathf.Max(1, value);
                if (stateMachine != null) stateMachine.AcquisitionHitsRequired = acquisitionHitsRequired;
            }
        }
        public bool EnableGating { get => enableGating; set => enableGating = value; }
        public float GatingThreshold { get => gatingThreshold; set => gatingThreshold = Mathf.Max(1.0f, value); }
        public bool AdaptiveMeasurementNoise { get => adaptiveMeasurementNoise; set => adaptiveMeasurementNoise = value; }

        // Public Events
        public event Action<TrackingResult> OnTrackingUpdated;
        public event Action<TrackingState, TrackingState> OnTrackingStateChanged;

        // Internal State Machine & Kalman State
        private OPTINAVTrackingStateMachine stateMachine;
        private KalmanFilter2D kalmanFilter;
        private double lastTimestamp = 0;
        private long lastProcessedFrameId = -1;
        private readonly Stopwatch perfStopwatch = new Stopwatch();

        // FPS Calculation
        private int fpsCounter = 0;
        private float lastFpsTime = 0f;

        private void Awake()
        {
            if (Instance == null)
            {
                Instance = this;
            }
            else if (Instance != this)
            {
                if (Application.isPlaying)
                {
                    Destroy(gameObject);
                    return;
                }
            }

            InitializeComponents();
        }

        private void InitializeComponents()
        {
            stateMachine = new OPTINAVTrackingStateMachine(acquisitionHitsRequired, maxCoastingFrames, acquisitionMissThreshold);
            stateMachine.OnStateTransition += HandleStateTransition;

            kalmanFilter = new KalmanFilter2D(processNoise, measurementNoise, initialPositionUncertainty, initialVelocityUncertainty);
            latestResult = TrackingResult.CreateEmpty();
        }

        private void Start()
        {
            if (detectionBridge == null)
            {
                detectionBridge = FindFirstObjectByType<OPTINAVDetectionBridge>();
            }

            if (autoSubscribeBridge && detectionBridge != null)
            {
                detectionBridge.OnDetectionReceived += OnBridgeDetectionReceived;
            }
        }

        private void OnDestroy()
        {
            if (detectionBridge != null && autoSubscribeBridge)
            {
                detectionBridge.OnDetectionReceived -= OnBridgeDetectionReceived;
            }

            if (stateMachine != null)
            {
                stateMachine.OnStateTransition -= HandleStateTransition;
            }

            if (Instance == this)
            {
                Instance = null;
            }
        }

        private void HandleStateTransition(TrackingState prev, TrackingState next)
        {
            currentState = next;
            OnTrackingStateChanged?.Invoke(prev, next);
        }

        private void OnBridgeDetectionReceived(DetectionResult detection)
        {
            ProcessDetection(detection);
        }

        /// <summary>
        /// Primary processing pipeline for incoming optical beacon detections.
        /// Pure M4 logic: Predict -> Gating -> Correct -> State Machine -> TrackingResult.
        /// </summary>
        /// <param name="detection">The raw M3 detection result</param>
        /// <param name="overrideDt">Optional explicit time step in seconds (useful for offline replay/testing)</param>
        /// <returns>Computed M4 TrackingResult</returns>
        public TrackingResult ProcessDetection(DetectionResult detection, float? overrideDt = null)
        {
            if (stateMachine == null || kalmanFilter == null)
            {
                InitializeComponents();
            }

            perfStopwatch.Restart();

            // Compute delta time in seconds
            float dt;
            if (overrideDt.HasValue)
            {
                dt = Mathf.Clamp(overrideDt.Value, 0.001f, 0.5f);
            }
            else
            {
                double currentTs = detection.detectionTimestamp > 0 ? detection.detectionTimestamp : (Time.timeAsDouble > 0 ? Time.timeAsDouble : 0.0);
                if (lastTimestamp > 0 && currentTs > lastTimestamp)
                {
                    dt = Mathf.Clamp((float)(currentTs - lastTimestamp), 0.001f, 0.5f);
                }
                else
                {
                    dt = Time.deltaTime > 0 ? Mathf.Clamp(Time.deltaTime, 0.001f, 0.5f) : (1.0f / 30.0f);
                }
                lastTimestamp = currentTs;
            }

            lastProcessedFrameId = detection.frameId;

            // Synchronize parameters with filter
            kalmanFilter.ProcessNoise = processNoise;
            kalmanFilter.MeasurementNoise = measurementNoise;
            kalmanFilter.InitialPositionUncertainty = initialPositionUncertainty;
            kalmanFilter.InitialVelocityUncertainty = initialVelocityUncertainty;

            bool isDetectionValid = detection.detected && detection.pixelCenterX >= 0f && detection.pixelCenterY >= 0f;

            // Step 1: Predict step (Advance state kinematics if track exists)
            Vector2 priorPredictedPos = Vector2.one * -1f;
            if (stateMachine.HasTrack && kalmanFilter.IsInitialized)
            {
                kalmanFilter.Predict(dt);
                priorPredictedPos = kalmanFilter.Position;
            }

            // Step 2: Gating / Outlier rejection (if active and tracking)
            bool measurementAccepted = isDetectionValid;
            if (isDetectionValid && stateMachine.IsTracking && enableGating && kalmanFilter.IsInitialized)
            {
                float d2 = kalmanFilter.ComputeMahalanobisDistanceSq(detection.pixelCenterX, detection.pixelCenterY);
                if (d2 > gatingThreshold)
                {
                    // Rejected as statistical outlier (e.g. distractor jump)
                    measurementAccepted = false;
                }
            }

            // Step 3: Advance state machine
            TrackingState oldState = stateMachine.CurrentState;
            TrackingState newState = stateMachine.Step(measurementAccepted);

            // Step 4: Handle State & Filter Updates
            Vector2 finalTrackedPos;
            Vector2 velocity;
            float covX, covY;
            float resultConfidence;

            switch (newState)
            {
                case TrackingState.SEARCH:
                    // No active track
                    kalmanFilter.Reset();
                    finalTrackedPos = Vector2.one * -1f;
                    priorPredictedPos = Vector2.one * -1f;
                    velocity = Vector2.zero;
                    covX = 0f;
                    covY = 0f;
                    resultConfidence = 0f;
                    break;

                case TrackingState.ACQUISITION:
                    if (measurementAccepted)
                    {
                        if (!kalmanFilter.IsInitialized || oldState == TrackingState.SEARCH || oldState == TrackingState.LOST)
                        {
                            // First acquisition hit: initialize filter directly at measurement
                            kalmanFilter.Initialize(detection.pixelCenterX, detection.pixelCenterY);
                            priorPredictedPos = kalmanFilter.Position;
                        }
                        else
                        {
                            // Subsequent acquisition hit: Kalman correction
                            float effR = GetEffectiveMeasurementNoise(detection.confidence);
                            kalmanFilter.Correct(detection.pixelCenterX, detection.pixelCenterY, effR);
                        }

                        finalTrackedPos = kalmanFilter.Position;
                        velocity = kalmanFilter.Velocity;
                        covX = kalmanFilter.CovarianceX;
                        covY = kalmanFilter.CovarianceY;
                        resultConfidence = detection.confidence;
                    }
                    else
                    {
                        // Acquisition missed frame
                        finalTrackedPos = kalmanFilter.IsInitialized ? kalmanFilter.Position : (Vector2.one * -1f);
                        velocity = kalmanFilter.IsInitialized ? kalmanFilter.Velocity : Vector2.zero;
                        covX = kalmanFilter.IsInitialized ? kalmanFilter.CovarianceX : 0f;
                        covY = kalmanFilter.IsInitialized ? kalmanFilter.CovarianceY : 0f;
                        resultConfidence = 0f;
                    }
                    break;

                case TrackingState.TRACKING:
                    if (measurementAccepted)
                    {
                        if (!kalmanFilter.IsInitialized)
                        {
                            kalmanFilter.Initialize(detection.pixelCenterX, detection.pixelCenterY);
                            priorPredictedPos = kalmanFilter.Position;
                        }
                        else
                        {
                            float effR = GetEffectiveMeasurementNoise(detection.confidence);
                            kalmanFilter.Correct(detection.pixelCenterX, detection.pixelCenterY, effR);
                        }

                        finalTrackedPos = kalmanFilter.Position;
                        velocity = kalmanFilter.Velocity;
                        covX = kalmanFilter.CovarianceX;
                        covY = kalmanFilter.CovarianceY;
                        resultConfidence = detection.confidence;
                    }
                    else
                    {
                        // Should not typically happen as Step(false) transitions to COASTING, but handled safely
                        finalTrackedPos = kalmanFilter.Position;
                        velocity = kalmanFilter.Velocity;
                        covX = kalmanFilter.CovarianceX;
                        covY = kalmanFilter.CovarianceY;
                        resultConfidence = 0.5f;
                    }
                    break;

                case TrackingState.COASTING:
                    // Dropout: NO measurement update performed!
                    // Prediction was already advanced in Step 1.
                    // Output current predicted state as the tracked position
                    finalTrackedPos = kalmanFilter.Position;
                    priorPredictedPos = kalmanFilter.Position;
                    velocity = kalmanFilter.Velocity;
                    covX = kalmanFilter.CovarianceX;
                    covY = kalmanFilter.CovarianceY;

                    // Decayed confidence during coasting
                    float coastDecay = Mathf.Clamp01(1.0f - (float)stateMachine.ConsecutiveMisses / (maxCoastingFrames + 1));
                    resultConfidence = Mathf.Max(0.1f, latestResult.confidence * coastDecay);
                    break;

                case TrackingState.LOST:
                default:
                    // Active track has been invalidated
                    kalmanFilter.Reset();
                    finalTrackedPos = Vector2.one * -1f;
                    priorPredictedPos = Vector2.one * -1f;
                    velocity = Vector2.zero;
                    covX = 0f;
                    covY = 0f;
                    resultConfidence = 0f;
                    break;
            }

            perfStopwatch.Stop();
            float procLatency = (float)perfStopwatch.Elapsed.TotalMilliseconds;

            // Construct immutable TrackingResult
            TrackingResult res = new TrackingResult
            {
                frameId = detection.frameId,
                hasTrack = stateMachine.HasTrack,
                hasMeasurement = measurementAccepted && (newState == TrackingState.ACQUISITION || newState == TrackingState.TRACKING),
                trackedPixelX = finalTrackedPos.x,
                trackedPixelY = finalTrackedPos.y,
                predictedPixelX = priorPredictedPos.x,
                predictedPixelY = priorPredictedPos.y,
                velocityX = velocity.x,
                velocityY = velocity.y,
                confidence = resultConfidence,
                consecutiveMisses = stateMachine.ConsecutiveMisses,
                state = newState,
                covarianceX = covX,
                covarianceY = covY,
                timestamp = detection.detectionTimestamp > 0 ? detection.detectionTimestamp : Time.timeAsDouble,
                latencyMs = procLatency
            };

            latestResult = res;
            hasActiveTrack = res.hasTrack;
            currentState = res.state;
            currentVelocityX = res.velocityX;
            currentVelocityY = res.velocityY;

            fpsCounter++;

            // Surface telemetry to subscribers (M5, M6, benchmarks)
            OnTrackingUpdated?.Invoke(latestResult);

            return latestResult;
        }

        private float GetEffectiveMeasurementNoise(float detectionConfidence)
        {
            if (!adaptiveMeasurementNoise) return measurementNoise;
            float conf = Mathf.Clamp(detectionConfidence, 0.15f, 1.0f);
            return measurementNoise / conf;
        }

        private void Update()
        {
            float now = Time.unscaledTime;
            if (now - lastFpsTime >= 1.0f)
            {
                trackingFPS = fpsCounter / (now - lastFpsTime);
                fpsCounter = 0;
                lastFpsTime = now;
            }
        }

        /// <summary>
        /// Explicitly resets tracking state machine, filter covariances, and kinematic state.
        /// </summary>
        public void ResetTracker()
        {
            stateMachine?.Reset();
            kalmanFilter?.Reset();
            latestResult = TrackingResult.CreateEmpty(lastProcessedFrameId);
            hasActiveTrack = false;
            currentState = TrackingState.SEARCH;
            currentVelocityX = 0f;
            currentVelocityY = 0f;
            lastTimestamp = 0;
        }

        // ==============================================================================
        // INTERNAL 2D CONSTANT-VELOCITY KALMAN FILTER MATHEMATICS
        // ==============================================================================
        /// <summary>
        /// Self-contained, zero-allocation 2D Constant Velocity Kalman Filter.
        /// State vector: [x, y, vx, vy]^T.
        /// Measurements: [zx, zy]^T.
        /// Features Continuous White Noise Acceleration process noise, Joseph stabilized covariance update,
        /// and closed-form 2x2 innovation inversion.
        /// </summary>
        private class KalmanFilter2D
        {
            // State Vector [x, y, vx, vy]
            private float x;
            private float y;
            private float vx;
            private float vy;

            // 4x4 State Covariance Matrix P (Stored as flat array for cache locality)
            private readonly float[] P = new float[16];

            // Reusable scratch buffers for zero-allocation prediction & correction
            private readonly float[] M = new float[16];
            private readonly float[] A = new float[16];
            private readonly float[] AP = new float[16];
            private readonly float[] newP = new float[16];
            private readonly float[] Kvec0 = new float[4];
            private readonly float[] Kvec1 = new float[4];

            // Filter Configuration
            public float ProcessNoise { get; set; }
            public float MeasurementNoise { get; set; }
            public float InitialPositionUncertainty { get; set; }
            public float InitialVelocityUncertainty { get; set; }

            public bool IsInitialized { get; private set; }

            public Vector2 Position => new Vector2(x, y);
            public Vector2 Velocity => new Vector2(vx, vy);
            public float CovarianceX => P[0];  // P[0,0]
            public float CovarianceY => P[5];  // P[1,1]
            public float CovarianceVx => P[10]; // P[2,2]
            public float CovarianceVy => P[15]; // P[3,3]

            public KalmanFilter2D(float q, float r, float pInitPos, float pInitVel)
            {
                ProcessNoise = q;
                MeasurementNoise = r;
                InitialPositionUncertainty = pInitPos;
                InitialVelocityUncertainty = pInitVel;
                Reset();
            }

            public void Initialize(float initX, float initY, float initVx = 0f, float initVy = 0f)
            {
                x = initX;
                y = initY;
                vx = initVx;
                vy = initVy;

                // Reset P to diagonal initial uncertainties
                Array.Clear(P, 0, 16);
                P[0] = InitialPositionUncertainty;  // P[0,0]
                P[5] = InitialPositionUncertainty;  // P[1,1]
                P[10] = InitialVelocityUncertainty; // P[2,2]
                P[15] = InitialVelocityUncertainty; // P[3,3]

                IsInitialized = true;
            }

            public void Reset()
            {
                x = 0f;
                y = 0f;
                vx = 0f;
                vy = 0f;
                Array.Clear(P, 0, 16);
                IsInitialized = false;
            }

            /// <summary>
            /// Advance state and covariance through time interval dt using constant velocity transition:
            /// x^- = F * x
            /// P^- = F * P * F^T + Q
            /// </summary>
            public void Predict(float dt)
            {
                if (!IsInitialized) return;

                // 1. State Prediction: x = x + vx*dt, y = y + vy*dt
                x += vx * dt;
                y += vy * dt;
                // vx and vy remain constant under CV assumption

                // 2. Continuous White Noise Acceleration (CWNA) Process Noise Matrix Q(dt)
                float dt2 = dt * dt;
                float dt3 = dt2 * dt;
                float qPos = (dt3 / 3.0f) * ProcessNoise;
                float qCross = (dt2 / 2.0f) * ProcessNoise;
                float qVel = dt * ProcessNoise;

                // 3. Covariance Prediction: P_new = F * P * F^T + Q
                // Intermediate M = F * P
                // Row 0: M[0,j] = P[0,j] + dt * P[2,j]
                // Row 1: M[1,j] = P[1,j] + dt * P[3,j]
                // Row 2: M[2,j] = P[2,j]
                // Row 3: M[3,j] = P[3,j]
                for (int j = 0; j < 4; j++)
                {
                    M[j] = P[j] + dt * P[8 + j];
                    M[4 + j] = P[4 + j] + dt * P[12 + j];
                    M[8 + j] = P[8 + j];
                    M[12 + j] = P[12 + j];
                }

                // Next: P_pred = M * F^T + Q
                // Col 0: P_pred[i,0] = M[i,0] + dt * M[i,2]
                // Col 1: P_pred[i,1] = M[i,1] + dt * M[i,3]
                // Col 2: P_pred[i,2] = M[i,2]
                // Col 3: P_pred[i,3] = M[i,3]
                for (int i = 0; i < 4; i++)
                {
                    int row = i * 4;
                    P[row + 0] = M[row + 0] + dt * M[row + 2];
                    P[row + 1] = M[row + 1] + dt * M[row + 3];
                    P[row + 2] = M[row + 2];
                    P[row + 3] = M[row + 3];
                }

                // Add Q elements
                P[0] += qPos;   // Q[0,0]
                P[2] += qCross; // Q[0,2]
                P[5] += qPos;   // Q[1,1]
                P[7] += qCross; // Q[1,3]
                P[8] += qCross; // Q[2,0]
                P[10] += qVel;  // Q[2,2]
                P[13] += qCross;// Q[3,1]
                P[15] += qVel;  // Q[3,3]

                // Enforce symmetry
                EnforceSymmetry();
            }

            /// <summary>
            /// Update state and covariance with a valid 2D measurement (zx, zy) and measurement noise R:
            /// y = z - H * x
            /// S = H * P * H^T + R
            /// K = P * H^T * S^-1
            /// x = x + K * y
            /// P = (I - K*H) * P * (I - K*H)^T + K * R * K^T  (Joseph Form)
            /// </summary>
            public void Correct(float zx, float zy, float r)
            {
                if (!IsInitialized)
                {
                    Initialize(zx, zy);
                    return;
                }

                // 1. Innovation residual: y = z - H * x
                float yx = zx - x;
                float yy = zy - y;

                // 2. Innovation Covariance S = H * P * H^T + R
                // H = [1 0 0 0; 0 1 0 0], so H * P * H^T is the top-left 2x2 of P
                float S00 = P[0] + r;
                float S01 = P[1];
                float S10 = P[4];
                float S11 = P[5] + r;

                // 3. Invert 2x2 S matrix
                float det = S00 * S11 - S01 * S10;
                if (Mathf.Abs(det) < 1e-7f)
                {
                    // Regularize singularity
                    det = 1e-7f;
                }
                float invDet = 1.0f / det;
                float invS00 = S11 * invDet;
                float invS01 = -S01 * invDet;
                float invS10 = -S10 * invDet;
                float invS11 = S00 * invDet;

                // 4. Kalman Gain K = P * H^T * S^-1
                // P * H^T is 4x2 consisting of the first two columns of P
                // K is 4x2
                float K00 = P[0] * invS00 + P[1] * invS10;
                float K01 = P[0] * invS01 + P[1] * invS11;

                float K10 = P[4] * invS00 + P[5] * invS10;
                float K11 = P[4] * invS01 + P[5] * invS11;

                float K20 = P[8] * invS00 + P[9] * invS10;
                float K21 = P[8] * invS01 + P[9] * invS11;

                float K30 = P[12] * invS00 + P[13] * invS10;
                float K31 = P[12] * invS01 + P[13] * invS11;

                // 5. State Update: x = x + K * y
                x += K00 * yx + K01 * yy;
                y += K10 * yx + K11 * yy;
                vx += K20 * yx + K21 * yy;
                vy += K30 * yx + K31 * yy;

                // 6. Covariance Update using Joseph Stabilized Form:
                // A = (I - K * H)
                // P_updated = A * P * A^T + K * R * K^T
                // H = [1 0 0 0; 0 1 0 0]
                // K * H is 4x4 with only columns 0 and 1 non-zero:
                // Row i: [Ki0, Ki1, 0, 0]
                for (int i = 0; i < 4; i++)
                {
                    int row = i * 4;
                    float ki0 = (i == 0 ? K00 : (i == 1 ? K10 : (i == 2 ? K20 : K30)));
                    float ki1 = (i == 0 ? K01 : (i == 1 ? K11 : (i == 2 ? K21 : K31)));

                    A[row + 0] = (i == 0 ? 1f : 0f) - ki0;
                    A[row + 1] = (i == 1 ? 1f : 0f) - ki1;
                    A[row + 2] = (i == 2 ? 1f : 0f);
                    A[row + 3] = (i == 3 ? 1f : 0f);
                }

                // Compute AP = A * P
                for (int i = 0; i < 4; i++)
                {
                    int iRow = i * 4;
                    for (int j = 0; j < 4; j++)
                    {
                        float sum = 0f;
                        for (int k = 0; k < 4; k++)
                        {
                            sum += A[iRow + k] * P[k * 4 + j];
                        }
                        AP[iRow + j] = sum;
                    }
                }

                // Compute APA_T = AP * A^T
                for (int i = 0; i < 4; i++)
                {
                    int iRow = i * 4;
                    for (int j = 0; j < 4; j++)
                    {
                        float sum = 0f;
                        for (int k = 0; k < 4; k++)
                        {
                            sum += AP[iRow + k] * A[j * 4 + k];
                        }
                        newP[iRow + j] = sum;
                    }
                }

                // Add K * R * K^T (where R is diagonal with r)
                // (K * R * K^T)_ij = r * (Ki0 * Kj0 + Ki1 * Kj1)
                Kvec0[0] = K00; Kvec0[1] = K10; Kvec0[2] = K20; Kvec0[3] = K30;
                Kvec1[0] = K01; Kvec1[1] = K11; Kvec1[2] = K21; Kvec1[3] = K31;
                for (int i = 0; i < 4; i++)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        newP[i * 4 + j] += r * (Kvec0[i] * Kvec0[j] + Kvec1[i] * Kvec1[j]);
                    }
                }

                Array.Copy(newP, P, 16);
                EnforceSymmetry();
            }

            /// <summary>
            /// Computes Mahalanobis distance squared d^2 = y^T * S^-1 * y for statistical gating.
            /// </summary>
            public float ComputeMahalanobisDistanceSq(float zx, float zy)
            {
                if (!IsInitialized) return 0f;

                float yx = zx - x;
                float yy = zy - y;

                float S00 = P[0] + MeasurementNoise;
                float S01 = P[1];
                float S10 = P[4];
                float S11 = P[5] + MeasurementNoise;

                float det = S00 * S11 - S01 * S10;
                if (Mathf.Abs(det) < 1e-7f) det = 1e-7f;

                float invDet = 1.0f / det;
                float invS00 = S11 * invDet;
                float invS01 = -S01 * invDet;
                float invS10 = -S10 * invDet;
                float invS11 = S00 * invDet;

                // d^2 = y^T * S^-1 * y
                float d2 = yx * (invS00 * yx + invS01 * yy) + yy * (invS10 * yx + invS11 * yy);
                return Mathf.Max(0f, d2);
            }

            private void EnforceSymmetry()
            {
                for (int i = 0; i < 4; i++)
                {
                    for (int j = i + 1; j < 4; j++)
                    {
                        float avg = 0.5f * (P[i * 4 + j] + P[j * 4 + i]);
                        P[i * 4 + j] = avg;
                        P[j * 4 + i] = avg;
                    }
                    // Numerical floor on diagonal variances
                    P[i * 4 + i] = Mathf.Max(P[i * 4 + i], 1e-6f);
                }
            }
        }
    }
}
