using UnityEngine;

namespace OPTINAV.CameraSystem
{
    /// <summary>
    /// Configuration and optical model controller for the OPTINAV virtual camera.
    /// Manages field-of-view (FOV), sensor reference resolution, camera projection metrics,
    /// and acts as the high-level interface between optical tracking code and the mechanical rig.
    /// </summary>
    [RequireComponent(typeof(Camera))]
    public class OPTINAVCameraController : MonoBehaviour
    {
        [Header("Gimbal Mechanical Rig")]
        [SerializeField] private OPTINAVCameraRig cameraRig;

        [Header("Optical Sensor Parameters")]
        [Tooltip("Field of View in degrees (vertical or horizontal depending on gate fit)")]
        [Range(1f, 120f)]
        [SerializeField] private float fieldOfView = 60f;

        [Tooltip("Nominal optical sensor capture resolution")]
        [SerializeField] private Vector2Int referenceResolution = new Vector2Int(1920, 1080);

        [Tooltip("Near clipping plane (meters)")]
        [SerializeField] private float nearClipPlane = 0.1f;

        [Tooltip("Far clipping plane (meters)")]
        [SerializeField] private float farClipPlane = 500f;

        private Camera attachedCamera;

        // Public APIs for Member 3 (AI Detection) & Member 4 (Kalman)
        public Camera CameraComponent => attachedCamera;
        public OPTINAVCameraRig Rig => cameraRig;
        public float FOV => fieldOfView;
        public Vector2Int ReferenceResolution => referenceResolution;
        public float AspectRatio => referenceResolution.y > 0 ? (float)referenceResolution.x / referenceResolution.y : 1.7778f;

        public float PanAngle => cameraRig != null ? cameraRig.CurrentPanAngle : 0f;
        public float TiltAngle => cameraRig != null ? cameraRig.CurrentTiltAngle : 0f;

        private void Awake()
        {
            attachedCamera = GetComponent<Camera>();
            if (cameraRig == null)
            {
                cameraRig = GetComponentInParent<OPTINAVCameraRig>();
            }
            ApplyCameraSettings();
        }

        private void OnValidate()
        {
            if (attachedCamera == null)
            {
                attachedCamera = GetComponent<Camera>();
            }
            if (cameraRig == null)
            {
                cameraRig = GetComponentInParent<OPTINAVCameraRig>();
            }
            ApplyCameraSettings();
        }

        public void ApplyCameraSettings()
        {
            if (attachedCamera != null)
            {
                attachedCamera.fieldOfView = fieldOfView;
                attachedCamera.nearClipPlane = nearClipPlane;
                attachedCamera.farClipPlane = farClipPlane;
            }
        }

        public void SetFOV(float fov)
        {
            fieldOfView = Mathf.Clamp(fov, 1f, 120f);
            if (attachedCamera != null)
            {
                attachedCamera.fieldOfView = fieldOfView;
            }
        }

        public void SetPanTilt(float pan, float tilt)
        {
            if (cameraRig != null)
            {
                cameraRig.SetTargetAngles(pan, tilt);
            }
        }

        /// <summary>
        /// Projects a world space coordinate into normalized sensor coordinates [0,1].
        /// </summary>
        public Vector2 WorldToNormalizedViewportPoint(Vector3 worldPoint)
        {
            if (attachedCamera == null) return Vector2.zero;
            Vector3 vp = attachedCamera.WorldToViewportPoint(worldPoint);
            return new Vector2(vp.x, vp.y);
        }

        /// <summary>
        /// Projects a world space coordinate into pixel sensor coordinates based on reference resolution.
        /// </summary>
        public Vector2 WorldToReferencePixelPoint(Vector3 worldPoint)
        {
            Vector2 norm = WorldToNormalizedViewportPoint(worldPoint);
            return new Vector2(norm.x * referenceResolution.x, norm.y * referenceResolution.y);
        }

        /// <summary>
        /// Checks if a world position lies within the active camera frustum and in front of the lens.
        /// </summary>
        public bool IsTargetInFrustum(Vector3 worldPoint)
        {
            if (attachedCamera == null) return false;
            Vector3 vp = attachedCamera.WorldToViewportPoint(worldPoint);
            return vp.z > 0f && vp.x >= 0f && vp.x <= 1f && vp.y >= 0f && vp.y <= 1f;
        }
    }
}
