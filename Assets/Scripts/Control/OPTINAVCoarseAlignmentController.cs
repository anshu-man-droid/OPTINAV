using System;
using UnityEngine;
using OPTINAV.CameraSystem;
using OPTINAV.Tracking;

namespace OPTINAV.Control
{
    /// <summary>
    /// Member 5 Coarse-Alignment & Camera Tracking Controller.
    /// Closes the optical tracking loop:
    /// 1. Consumes kinematic TrackingResult from Member 4 (OPTINAVKalmanTracker).
    /// 2. Converts pixel-space tracking error to line-of-sight (LOS) angular error using camera optics and FOV.
    /// 3. Applies independent Pan and Tilt discrete PID controllers (OPTINAVPIDController).
    /// 4. Manages tracking lifecycle states (SEARCH, ACQUISITION, TRACKING, COASTING, LOST).
    /// 5. Commands the 2-DOF gimbal via OPTINAVCameraRig while respecting mechanical angular limits.
    /// 
    /// Strict Architectural Boundaries:
    /// - Does NOT use target ground-truth position for control (evaluation only).
    /// - Does NOT modify camera transforms directly (delegates to OPTINAVCameraRig.SetTargetAngles).
    /// - Freezes integral accumulation during COASTING to prevent windup during sensor dropouts.
    /// </summary>
    [DisallowMultipleComponent]
    public class OPTINAVCoarseAlignmentController : MonoBehaviour
    {
        [Header("Subsystem References")]
        [Tooltip("Direct reference to the 2-DOF Pan/Tilt camera rig")]
        [SerializeField] private OPTINAVCameraRig cameraRig;

        [Tooltip("Direct reference to the camera optical controller")]
        [SerializeField] private OPTINAVCameraController cameraController;

        [Tooltip("Direct reference to the Member 4 Kalman tracker")]
        [SerializeField] private OPTINAVKalmanTracker tracker;

        [Header("Control Loop Configuration")]
        [Tooltip("Master toggle for closed-loop camera tracking control")]
        [SerializeField] private bool controlEnabled = true;

        [Tooltip("Image sensor resolution width (matches M3/M4 frame capture)")]
        [SerializeField] private int frameWidth = 640;

        [Tooltip("Image sensor resolution height (matches M3/M4 frame capture)")]
        [SerializeField] private int frameHeight = 360;

        [Header("Pan Axis PID Controller (Azimuth / Yaw)")]
        [SerializeField] private OPTINAVPIDController panPID = new OPTINAVPIDController(
            kp: 0.65f,
            ki: 0.04f,
            kd: 0.06f,
            minOutput: -45.0f,
            maxOutput: 45.0f,
            deadband: 0.05f,
            filterTimeConstant: 0.02f
        );

        [Header("Tilt Axis PID Controller (Elevation / Pitch)")]
        [SerializeField] private OPTINAVPIDController tiltPID = new OPTINAVPIDController(
            kp: 0.65f,
            ki: 0.04f,
            kd: 0.06f,
            minOutput: -35.0f,
            maxOutput: 35.0f,
            deadband: 0.05f,
            filterTimeConstant: 0.02f
        );

        [Header("Search & Lost Scan Configuration")]
        [Tooltip("Execute search pattern scan when target is in SEARCH or LOST state")]
        [SerializeField] private bool enableSearchPattern = false;

        [Tooltip("Horizontal pan amplitude in degrees for search pattern")]
        [SerializeField] private float searchPanAmplitude = 25.0f;

        [Tooltip("Vertical tilt amplitude in degrees for search pattern")]
        [SerializeField] private float searchTiltAmplitude = 8.0f;

        [Tooltip("Pan search scan frequency in Hz")]
        [SerializeField] private float searchPanFrequency = 0.2f;

        [Tooltip("Tilt search scan frequency in Hz")]
        [SerializeField] private float searchTiltFrequency = 0.05f;

        [Header("Alignment Thresholds & Performance")]
        [Tooltip("Pixel tolerance radius around optical center to declare target aligned")]
        [SerializeField] private float alignmentTolerancePixels = 6.0f;

        [Tooltip("Angular tolerance in degrees to declare target aligned")]
        [SerializeField] private float alignmentToleranceDegrees = 0.5f;

        [Header("Live Telemetry & Diagnostics")]
        [SerializeField] private TrackingState currentTrackingState = TrackingState.SEARCH;
        [SerializeField] private bool isTargetAligned = false;
        [SerializeField] private float currentPixelErrorX = 0f;
        [SerializeField] private float currentPixelErrorY = 0f;
        [SerializeField] private float currentPixelErrorTotal = 0f;
        [SerializeField] private float currentPanErrorDeg = 0f;
        [SerializeField] private float currentTiltErrorDeg = 0f;
        [SerializeField] private float currentAngularErrorTotal = 0f;
        [SerializeField] private float commandedPanAngle = 0f;
        [SerializeField] private float commandedTiltAngle = 0f;
        [SerializeField] private float controlUpdateRateHz = 0f;

        // Public Telemetry APIs for Member 6 Dashboard & Benchmarking
        public bool ControlEnabled { get => controlEnabled; set => controlEnabled = value; }
        public bool IsTargetAligned => isTargetAligned;
        public float PixelErrorTotal => currentPixelErrorTotal;
        public float AngularErrorTotal => currentAngularErrorTotal;
        public float PanErrorDegrees => currentPanErrorDeg;
        public float TiltErrorDegrees => currentTiltErrorDeg;
        public float CommandedPan => commandedPanAngle;
        public float CommandedTilt => commandedTiltAngle;
        public TrackingState ActiveTrackingState => currentTrackingState;
        public OPTINAVPIDController PanPID => panPID;
        public OPTINAVPIDController TiltPID => tiltPID;
        public float AlignmentTolerancePixels => alignmentTolerancePixels;
        public float AlignmentToleranceDegrees => alignmentToleranceDegrees;

        // Internal State
        private TrackingResult latestTrackingResult = TrackingResult.CreateEmpty();
        private bool hasReceivedTrackingResult = false;
        private long lastProcessedTrackingFrameId = -1;
        private float lastControlTime = -1f;
        private float searchTimer = 0f;
        private int updateCounter = 0;
        private float rateTimer = 0f;

        private void Awake()
        {
            FindReferences();
        }

        private void OnEnable()
        {
            FindReferences();
            SubscribeToTracker();
        }

        private void OnDisable()
        {
            UnsubscribeFromTracker();
            ResetControllers();
        }

        private void Start()
        {
            FindReferences();
            SubscribeToTracker();
            SyncPidLimitsWithRig();
        }

        private void FindReferences()
        {
            if (cameraRig == null) cameraRig = FindFirstObjectByType<OPTINAVCameraRig>();
            if (cameraController == null) cameraController = FindFirstObjectByType<OPTINAVCameraController>();
            if (tracker == null) tracker = FindFirstObjectByType<OPTINAVKalmanTracker>();

            if (cameraRig != null)
            {
                commandedPanAngle = cameraRig.CurrentPanAngle;
                commandedTiltAngle = cameraRig.CurrentTiltAngle;
            }
        }

        private void SubscribeToTracker()
        {
            if (tracker != null)
            {
                tracker.OnTrackingUpdated -= HandleTrackingUpdated;
                tracker.OnTrackingUpdated += HandleTrackingUpdated;
                tracker.OnTrackingStateChanged -= HandleTrackingStateChanged;
                tracker.OnTrackingStateChanged += HandleTrackingStateChanged;
            }
        }

        private void UnsubscribeFromTracker()
        {
            if (tracker != null)
            {
                tracker.OnTrackingUpdated -= HandleTrackingUpdated;
                tracker.OnTrackingStateChanged -= HandleTrackingStateChanged;
            }
        }

        /// <summary>
        /// Ensures PID saturation limits respect the physical mechanical limits of the camera rig
        /// while preserving the configured PID output/slew limits.
        /// </summary>
        public void SyncPidLimitsWithRig()
        {
            if (cameraRig == null) return;

            // Pan travel limit: preserve configured limits bounded by physical rig travel
            float maxPanTravel = (cameraRig.MaxPan - cameraRig.MinPan);
            panPID.MinOutput = Mathf.Max(panPID.MinOutput, -maxPanTravel);
            panPID.MaxOutput = Mathf.Min(panPID.MaxOutput, maxPanTravel);

            // Tilt travel limit: preserve configured limits bounded by physical rig travel
            float maxTiltTravel = (cameraRig.MaxTilt - cameraRig.MinTilt);
            tiltPID.MinOutput = Mathf.Max(tiltPID.MinOutput, -maxTiltTravel);
            tiltPID.MaxOutput = Mathf.Min(tiltPID.MaxOutput, maxTiltTravel);
        }

        private void HandleTrackingStateChanged(TrackingState previousState, TrackingState newState)
        {
            currentTrackingState = newState;

            // When target is lost or entering search, reset PID integrators to avoid residual kick
            if (newState == TrackingState.SEARCH || newState == TrackingState.LOST)
            {
                ResetControllers();
            }
        }

        private void HandleTrackingUpdated(TrackingResult result)
        {
            latestTrackingResult = result;
            hasReceivedTrackingResult = true;
            currentTrackingState = result.state;
        }

        private void Update()
        {
            if (!controlEnabled || cameraRig == null) return;

            float dt = Time.deltaTime;
            if (dt <= 0f) return;

            // Update performance measurement
            rateTimer += dt;
            if (rateTimer >= 1.0f)
            {
                controlUpdateRateHz = updateCounter / rateTimer;
                updateCounter = 0;
                rateTimer = 0f;
            }

            // Fallback poll tracker state if event subscription was delayed
            if (!hasReceivedTrackingResult && tracker != null)
            {
                latestTrackingResult = tracker.LatestResult;
                currentTrackingState = tracker.State;
            }

            // Execute state-dependent alignment control logic
            switch (currentTrackingState)
            {
                case TrackingState.TRACKING:
                case TrackingState.ACQUISITION:
                case TrackingState.COASTING:
                {
                    // Guard against repeated closed-loop execution on the same stale measurement at render rate
                    bool isNewResult = (latestTrackingResult.frameId != lastProcessedTrackingFrameId && latestTrackingResult.frameId > 0);
                    if (isNewResult)
                    {
                        lastProcessedTrackingFrameId = latestTrackingResult.frameId;
                        float now = Time.time;
                        float controlDt = (lastControlTime > 0f) ? Mathf.Clamp(now - lastControlTime, 0.001f, 0.5f) : (dt > 0f ? dt : 0.04f);
                        lastControlTime = now;
                        updateCounter++;

                        if (currentTrackingState == TrackingState.TRACKING)
                        {
                            ExecuteClosedLoopTracking(controlDt, allowIntegral: true);
                        }
                        else if (currentTrackingState == TrackingState.ACQUISITION)
                        {
                            ExecuteClosedLoopTracking(controlDt, allowIntegral: false);
                        }
                        else // COASTING
                        {
                            ExecuteCoastingTracking(controlDt);
                        }
                    }
                    break;
                }

                case TrackingState.LOST:
                case TrackingState.SEARCH:
                default:
                    ExecuteSearchOrHold(dt);
                    break;
            }
        }

        /// <summary>
        /// Closed-loop optical tracking: centers detected beacon on camera crosshair.
        /// </summary>
        private void ExecuteClosedLoopTracking(float dt, bool allowIntegral)
        {
            Vector2 targetPixel = latestTrackingResult.TrackedPixel;

            // Validate tracked coordinates are within sensor bounds
            if (targetPixel.x < 0f || targetPixel.y < 0f)
            {
                return;
            }

            // 1. Calculate optical center error in pixels
            // Sensor center is (width/2, height/2). Origin (0,0) is top-left.
            float centerX = frameWidth * 0.5f;
            float centerY = frameHeight * 0.5f;

            currentPixelErrorX = targetPixel.x - centerX;
            currentPixelErrorY = targetPixel.y - centerY;
            currentPixelErrorTotal = Mathf.Sqrt(currentPixelErrorX * currentPixelErrorX + currentPixelErrorY * currentPixelErrorY);

            // 2. Convert pixel error to angular line-of-sight (LOS) error (degrees)
            ComputeAngularErrors(currentPixelErrorX, currentPixelErrorY, out currentPanErrorDeg, out currentTiltErrorDeg);
            currentAngularErrorTotal = Mathf.Sqrt(currentPanErrorDeg * currentPanErrorDeg + currentTiltErrorDeg * currentTiltErrorDeg);

            // 3. Update Alignment status
            isTargetAligned = (currentPixelErrorTotal <= alignmentTolerancePixels) ||
                              (currentAngularErrorTotal <= alignmentToleranceDegrees);

            // 4. Configure integral term behavior
            panPID.EnableAntiWindup = true;
            tiltPID.EnableAntiWindup = true;
            panPID.FreezeIntegration = !allowIntegral;
            tiltPID.FreezeIntegration = !allowIntegral;

            // 5. Evaluate discrete PID controllers
            float panCorrection = panPID.Calculate(currentPanErrorDeg, dt);
            float tiltCorrection = tiltPID.Calculate(currentTiltErrorDeg, dt);

            // 6. Compute new commanded target angles
            // Slew angle = current actual angle + PID correction
            float targetPan = cameraRig.CurrentPanAngle + panCorrection;
            float targetTilt = cameraRig.CurrentTiltAngle + tiltCorrection;

            // 7. Clamp within physical gimbal limits
            commandedPanAngle = Mathf.Clamp(targetPan, cameraRig.MinPan, cameraRig.MaxPan);
            commandedTiltAngle = Mathf.Clamp(targetTilt, cameraRig.MinTilt, cameraRig.MaxTilt);

            // 8. Command the gimbal mechanism via existing API
            cameraRig.SetTargetAngles(commandedPanAngle, commandedTiltAngle);
        }

        /// <summary>
        /// Handles COASTING during sensor dropouts / temporary occlusions.
        /// Continues tracking using Kalman predicted positions while strictly freezing integration.
        /// </summary>
        private void ExecuteCoastingTracking(float dt)
        {
            // Use Kalman predicted position during dropout
            Vector2 predPixel = latestTrackingResult.PredictedPixel;
            if (predPixel.x < 0f || predPixel.y < 0f)
            {
                predPixel = latestTrackingResult.TrackedPixel;
            }

            if (predPixel.x >= 0f && predPixel.x <= frameWidth && predPixel.y >= 0f && predPixel.y <= frameHeight)
            {
                float centerX = frameWidth * 0.5f;
                float centerY = frameHeight * 0.5f;

                currentPixelErrorX = predPixel.x - centerX;
                currentPixelErrorY = predPixel.y - centerY;
                currentPixelErrorTotal = Mathf.Sqrt(currentPixelErrorX * currentPixelErrorX + currentPixelErrorY * currentPixelErrorY);

                ComputeAngularErrors(currentPixelErrorX, currentPixelErrorY, out currentPanErrorDeg, out currentTiltErrorDeg);
                currentAngularErrorTotal = Mathf.Sqrt(currentPanErrorDeg * currentPanErrorDeg + currentTiltErrorDeg * currentTiltErrorDeg);

                // Anti-windup: Strictly suppress integral accumulation during coasting
                panPID.FreezeIntegration = true;
                tiltPID.FreezeIntegration = true;

                float panCorrection = panPID.Calculate(currentPanErrorDeg, dt);
                float tiltCorrection = tiltPID.Calculate(currentTiltErrorDeg, dt);

                float targetPan = cameraRig.CurrentPanAngle + panCorrection;
                float targetTilt = cameraRig.CurrentTiltAngle + tiltCorrection;

                commandedPanAngle = Mathf.Clamp(targetPan, cameraRig.MinPan, cameraRig.MaxPan);
                commandedTiltAngle = Mathf.Clamp(targetTilt, cameraRig.MinTilt, cameraRig.MaxTilt);

                cameraRig.SetTargetAngles(commandedPanAngle, commandedTiltAngle);
            }

            isTargetAligned = false;
        }

        /// <summary>
        /// Handles SEARCH / LOST states when no target track is available.
        /// Executes search scan or smoothly holds/boresights gimbal.
        /// </summary>
        private void ExecuteSearchOrHold(float dt)
        {
            isTargetAligned = false;
            currentPixelErrorTotal = -1f;
            currentAngularErrorTotal = -1f;

            if (enableSearchPattern)
            {
                searchTimer += dt;
                // Sinusoidal search scan pattern
                float searchPan = Mathf.Sin(searchTimer * Mathf.PI * 2.0f * searchPanFrequency) * searchPanAmplitude;
                float searchTilt = Mathf.Sin(searchTimer * Mathf.PI * 2.0f * searchTiltFrequency) * searchTiltAmplitude;

                commandedPanAngle = Mathf.Clamp(searchPan, cameraRig.MinPan, cameraRig.MaxPan);
                commandedTiltAngle = Mathf.Clamp(searchTilt, cameraRig.MinTilt, cameraRig.MaxTilt);
                cameraRig.SetTargetAngles(commandedPanAngle, commandedTiltAngle);
            }
            else
            {
                // Smoothly hold commanded orientation without continuing to drive toward stale limits
                cameraRig.SetTargetAngles(commandedPanAngle, commandedTiltAngle);
            }
        }

        /// <summary>
        /// Converts image-space pixel error to angular azimuth and elevation error in degrees.
        /// Derived from camera optics (FOV and aspect ratio).
        /// </summary>
        private void ComputeAngularErrors(float pxErrX, float pxErrY, out float panErrDeg, out float tiltErrDeg)
        {
            // Vertical FOV in degrees
            float vfov = (cameraController != null && cameraController.FOV > 0f) ? cameraController.FOV : 60.0f;
            float aspect = (cameraController != null && cameraController.AspectRatio > 0f) ? cameraController.AspectRatio : ((float)frameWidth / frameHeight);

            // Horizontal FOV derived via pinhole optical geometry:
            // tan(HFOV/2) = tan(VFOV/2) * AspectRatio
            float vfovRad = vfov * Mathf.Deg2Rad;
            float hfovRad = 2.0f * Mathf.Atan(Mathf.Tan(vfovRad * 0.5f) * aspect);
            float hfov = hfovRad * Mathf.Rad2Deg;

            // Angular resolution (degrees per pixel)
            float degPerPxX = hfov / frameWidth;
            float degPerPxY = vfov / frameHeight;

            // Coordinate sign conventions:
            // - Horizontal: Target to right (pxErrX > 0) requires camera to Pan Right (+Pan)
            panErrDeg = pxErrX * degPerPxX;

            // - Vertical: In image space, Y increases downwards.
            // Target below optical center (pxErrY > 0) requires camera to Pitch Down (-Tilt)
            tiltErrDeg = -pxErrY * degPerPxY;
        }

        /// <summary>
        /// Resets all internal PID controller accumulators and filters.
        /// </summary>
        public void ResetControllers()
        {
            panPID?.Reset();
            tiltPID?.Reset();
            isTargetAligned = false;
            currentPixelErrorTotal = 0f;
            currentAngularErrorTotal = 0f;
            searchTimer = 0f;
            lastProcessedTrackingFrameId = -1;
            lastControlTime = -1f;
            if (cameraRig != null)
            {
                commandedPanAngle = cameraRig.CurrentPanAngle;
                commandedTiltAngle = cameraRig.CurrentTiltAngle;
            }
        }
    }
}
