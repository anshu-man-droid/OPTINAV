using UnityEngine;

namespace OPTINAV.CameraSystem
{
    /// <summary>
    /// Controls the 2-DOF Pan/Tilt gimbal mechanism for the virtual FSOC tracking camera.
    /// Drives separate transforms for Pan (Yaw around Y) and Tilt (Pitch around X),
    /// enforces physical angular limits, supports optional slewing velocity limits,
    /// and layers relative disturbance offsets cleanly over commanded tracking angles.
    /// </summary>
    public class OPTINAVCameraRig : MonoBehaviour
    {
        [Header("Gimbal Axis Transforms")]
        [Tooltip("Transform responsible solely for horizontal pan (Yaw)")]
        [SerializeField] private Transform panAxis;

        [Tooltip("Transform responsible solely for vertical tilt (Pitch), child of PanAxis")]
        [SerializeField] private Transform tiltAxis;

        [Tooltip("Direct reference to the camera transform under TiltAxis")]
        [SerializeField] private Transform cameraTransform;

        [Header("Commanded Angles (Degrees)")]
        [Tooltip("Target pan angle in degrees (-left, +right)")]
        [Range(-180f, 180f)]
        [SerializeField] private float targetPanAngle = 0f;

        [Tooltip("Target tilt angle in degrees (-down, +up)")]
        [Range(-90f, 90f)]
        [SerializeField] private float targetTiltAngle = 0f;

        [Header("Physical Angular Limits (Degrees)")]
        [SerializeField] private float minPanAngle = -90f;
        [SerializeField] private float maxPanAngle = 90f;
        [SerializeField] private float minTiltAngle = -45f;
        [SerializeField] private float maxTiltAngle = 45f;

        [Header("Dynamics & Slewing Limits")]
        [Tooltip("Enable mechanical angular velocity clamping (slew rate limit)")]
        [SerializeField] private bool limitAngularVelocity = false;

        [Tooltip("Maximum pan/tilt angular speed in degrees/second")]
        [SerializeField] private float maxAngularVelocity = 60f;

        // Current actual angles (excluding disturbances)
        private float currentPanAngle = 0f;
        private float currentTiltAngle = 0f;

        // Relative disturbance offsets (from platform vibration & jitter)
        private Vector3 disturbancePositionOffset = Vector3.zero;
        private Quaternion disturbanceRotationOffset = Quaternion.identity;

        // Public APIs for Member 5 (PID tracking) and Member 6 (Telemetry)
        public Transform PanTransform => panAxis;
        public Transform TiltTransform => tiltAxis;
        public Transform CameraTransform => cameraTransform;

        public float CurrentPanAngle => currentPanAngle;
        public float CurrentTiltAngle => currentTiltAngle;
        public float TargetPanAngle => targetPanAngle;
        public float TargetTiltAngle => targetTiltAngle;

        public float MinPan => minPanAngle;
        public float MaxPan => maxPanAngle;
        public float MinTilt => minTiltAngle;
        public float MaxTilt => maxTiltAngle;

        public Vector3 LookDirection => cameraTransform != null ? cameraTransform.forward : (tiltAxis != null ? tiltAxis.forward : transform.forward);

        private void Awake()
        {
            ValidateTransforms();
            currentPanAngle = Mathf.Clamp(targetPanAngle, minPanAngle, maxPanAngle);
            currentTiltAngle = Mathf.Clamp(targetTiltAngle, minTiltAngle, maxTiltAngle);
            ApplyTransformRotations();
        }

        private void OnValidate()
        {
            targetPanAngle = Mathf.Clamp(targetPanAngle, minPanAngle, maxPanAngle);
            targetTiltAngle = Mathf.Clamp(targetTiltAngle, minTiltAngle, maxTiltAngle);
            if (!Application.isPlaying)
            {
                ValidateTransforms();
                currentPanAngle = targetPanAngle;
                currentTiltAngle = targetTiltAngle;
                ApplyTransformRotations();
            }
        }

        private void ValidateTransforms()
        {
            if (panAxis == null)
            {
                panAxis = transform.Find("PanAxis");
            }
            if (panAxis != null && tiltAxis == null)
            {
                tiltAxis = panAxis.Find("TiltAxis");
            }
            if (tiltAxis != null && cameraTransform == null)
            {
                var cam = tiltAxis.GetComponentInChildren<Camera>();
                if (cam != null) cameraTransform = cam.transform;
            }
        }

        private void Update()
        {
            float dt = Time.deltaTime;
            float clampedTargetPan = Mathf.Clamp(targetPanAngle, minPanAngle, maxPanAngle);
            float clampedTargetTilt = Mathf.Clamp(targetTiltAngle, minTiltAngle, maxTiltAngle);

            if (limitAngularVelocity && dt > 0f)
            {
                float maxDelta = maxAngularVelocity * dt;
                currentPanAngle = Mathf.MoveTowards(currentPanAngle, clampedTargetPan, maxDelta);
                currentTiltAngle = Mathf.MoveTowards(currentTiltAngle, clampedTargetTilt, maxDelta);
            }
            else
            {
                currentPanAngle = clampedTargetPan;
                currentTiltAngle = clampedTargetTilt;
            }

            ApplyTransformRotations();
        }

        private void LateUpdate()
        {
            // Apply disturbance offsets cleanly in LateUpdate without corrupting commanded angles
            if (cameraTransform != null)
            {
                cameraTransform.localPosition = disturbancePositionOffset;
                cameraTransform.localRotation = disturbanceRotationOffset;
            }
        }

        private void ApplyTransformRotations()
        {
            if (panAxis != null)
            {
                // Pan rotates around local Y axis (Yaw)
                panAxis.localRotation = Quaternion.Euler(0f, currentPanAngle, 0f);
            }
            if (tiltAxis != null)
            {
                // Tilt rotates around local X axis (Pitch: -tilt for standard pitch up/down convention)
                tiltAxis.localRotation = Quaternion.Euler(-currentTiltAngle, 0f, 0f);
            }
        }

        /// <summary>
        /// Command target pan and tilt angles directly (called by PID Tracking Controller or manual control).
        /// </summary>
        public void SetTargetAngles(float pan, float tilt)
        {
            targetPanAngle = Mathf.Clamp(pan, minPanAngle, maxPanAngle);
            targetTiltAngle = Mathf.Clamp(tilt, minTiltAngle, maxTiltAngle);
        }

        public void SetPan(float pan)
        {
            targetPanAngle = Mathf.Clamp(pan, minPanAngle, maxPanAngle);
        }

        public void SetTilt(float tilt)
        {
            targetTiltAngle = Mathf.Clamp(tilt, minTiltAngle, maxTiltAngle);
        }

        public void ResetOrientation()
        {
            SetTargetAngles(0f, 0f);
            currentPanAngle = 0f;
            currentTiltAngle = 0f;
            disturbancePositionOffset = Vector3.zero;
            disturbanceRotationOffset = Quaternion.identity;
            ApplyTransformRotations();
            if (cameraTransform != null)
            {
                cameraTransform.localPosition = Vector3.zero;
                cameraTransform.localRotation = Quaternion.identity;
            }
        }

        /// <summary>
        /// Layers relative disturbance position and rotation offsets on the camera without altering commanded gimbal angles.
        /// </summary>
        public void SetDisturbanceOffsets(Vector3 posOffset, Quaternion rotOffset)
        {
            disturbancePositionOffset = posOffset;
            disturbanceRotationOffset = rotOffset;
        }
    }
}
