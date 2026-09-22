using UnityEngine;
using OPTINAV.CameraSystem;
using OPTINAV.Targets;

namespace OPTINAV.Disturbances
{
    /// <summary>
    /// Coordinates environmental and physical disturbances for the OPTINAV simulation:
    /// 1. Platform vibration (positional harmonic displacement)
    /// 2. Camera angular jitter (rotational high-frequency perturbation)
    /// 3. Image blur approximation
    /// 4. Sensor noise approximation
    /// 5. Atmospheric haze / fog
    /// 6. Temporary beacon visibility loss (occlusion / dropout)
    /// </summary>
    public class OPTINAVDisturbanceController : MonoBehaviour
    {
        public static OPTINAVDisturbanceController Instance { get; private set; }

        [Header("Target & Camera References")]
        [SerializeField] private OPTINAVCameraRig cameraRig;
        [SerializeField] private OPTINAVTargetManager targetManager;

        [Header("1. Platform Vibration")]
        [Tooltip("Toggle platform physical translational vibration")]
        [SerializeField] private bool enableVibration = false;

        [Tooltip("Vibration translational amplitude in meters [X, Y, Z]")]
        [SerializeField] private Vector3 vibrationAmplitude = new Vector3(0.02f, 0.02f, 0.01f);

        [Tooltip("Vibration fundamental frequencies in Hz [X, Y, Z]")]
        [SerializeField] private Vector3 vibrationFrequency = new Vector3(8.0f, 12.0f, 5.0f);

        [Header("2. Camera Angular Jitter")]
        [Tooltip("Toggle high-frequency angular jitter")]
        [SerializeField] private bool enableJitter = false;

        [Tooltip("Jitter angular amplitude in degrees (Yaw, Pitch, Roll)")]
        [SerializeField] private Vector3 jitterAmplitude = new Vector3(0.3f, 0.3f, 0.15f);

        [Tooltip("Jitter pseudo-random frequency in Hz")]
        [SerializeField] private float jitterFrequency = 15.0f;

        [Tooltip("Seed for deterministic jitter generation")]
        [SerializeField] private int jitterSeed = 101;

        [Header("3. Optical Blur (Full-Screen Post-Processing)")]
        [Tooltip("Toggle optical/motion blur simulation on camera")]
        [SerializeField] private bool enableBlur = false;

        [Tooltip("Normalized simulated blur amount (0 to 1)")]
        [Range(0f, 1f)]
        [SerializeField] private float blurFactor = 0.35f;

        [Tooltip("Maximum blur radius scaling factor in texels")]
        [SerializeField] private float blurRadiusScale = 14.0f;

        [Header("4. Sensor Noise (Optical Detector Simulation)")]
        [Tooltip("Toggle sensor noise simulation on camera")]
        [SerializeField] private bool enableSensorNoise = false;

        [Tooltip("Simulated sensor noise level / SNR penalty (0 to 1)")]
        [Range(0f, 1f)]
        [SerializeField] private float sensorNoiseLevel = 0.2f;

        [Tooltip("Deterministic pseudo-random seed for sensor noise")]
        [SerializeField] private int sensorNoiseSeed = 101;

        [Tooltip("Toggle dynamic per-frame temporal noise animation")]
        [SerializeField] private bool dynamicSensorNoise = true;

        [Header("Image Disturbance Material")]
        [Tooltip("Post-processing material using Hidden/OPTINAV/CameraImageDisturbance")]
        [SerializeField] private Material imageDisturbanceMaterial;

        [Header("5. Atmospheric Haze")]
        [Tooltip("Toggle atmospheric haze/fog")]
        [SerializeField] private bool enableHaze = false;

        [Tooltip("Exponential atmospheric fog density")]
        [Range(0.001f, 0.1f)]
        [SerializeField] private float hazeDensity = 0.012f;

        [Tooltip("Atmospheric haze scattering color")]
        [SerializeField] private Color hazeColor = new Color(0.75f, 0.8f, 0.88f, 1f);

        [Header("6. Beacon Visibility Loss (Dropout)")]
        [Tooltip("Toggle periodic beacon visibility loss (stress tests Coasting -> Re-acquisition)")]
        [SerializeField] private bool enableVisibilityLoss = false;

        [Tooltip("Interval in seconds between visibility dropout events")]
        [SerializeField] private float visibilityLossInterval = 8.0f;

        [Tooltip("Duration in seconds of each visibility dropout")]
        [SerializeField] private float visibilityLossDuration = 2.0f;

        // Cached shader property IDs for zero-allocation performance
        private static readonly int PropBlurFactor = Shader.PropertyToID("_BlurFactor");
        private static readonly int PropBlurRadiusScale = Shader.PropertyToID("_BlurRadiusScale");
        private static readonly int PropSensorNoiseLevel = Shader.PropertyToID("_SensorNoiseLevel");
        private static readonly int PropNoiseSeed = Shader.PropertyToID("_NoiseSeed");
        private static readonly int PropNoiseTime = Shader.PropertyToID("_NoiseTime");
        private static readonly int PropNoiseDynamic = Shader.PropertyToID("_NoiseDynamic");

        // Runtime states
        private float simulationTime = 0f;
        private float dropoutTimer = 0f;
        private bool isCurrentlyDroppedOut = false;
        private Vector3 currentPosOffset = Vector3.zero;
        private Quaternion currentRotOffset = Quaternion.identity;

        // APIs for Member 6 (Telemetry), UI, & Runtime control
        public bool VibrationEnabled { get => enableVibration; set => enableVibration = value; }
        public bool JitterEnabled { get => enableJitter; set => enableJitter = value; }
        public bool BlurEnabled { get => enableBlur; set { enableBlur = value; UpdateImageDisturbanceParameters(); } }
        public bool SensorNoiseEnabled { get => enableSensorNoise; set { enableSensorNoise = value; UpdateImageDisturbanceParameters(); } }
        public bool HazeEnabled { get => enableHaze; set => SetHazeEnabled(value); }
        public bool VisibilityLossEnabled { get => enableVisibilityLoss; set => enableVisibilityLoss = value; }

        public float CurrentBlurFactor => enableBlur ? blurFactor : 0f;
        public float CurrentNoiseLevel => enableSensorNoise ? sensorNoiseLevel : 0f;
        public float BlurFactor { get => blurFactor; set { blurFactor = Mathf.Clamp01(value); UpdateImageDisturbanceParameters(); } }
        public float BlurRadiusScale { get => blurRadiusScale; set { blurRadiusScale = Mathf.Max(0f, value); UpdateImageDisturbanceParameters(); } }
        public float SensorNoiseLevel { get => sensorNoiseLevel; set { sensorNoiseLevel = Mathf.Clamp01(value); UpdateImageDisturbanceParameters(); } }
        public int SensorNoiseSeed { get => sensorNoiseSeed; set { sensorNoiseSeed = value; UpdateImageDisturbanceParameters(); } }
        public bool DynamicSensorNoise { get => dynamicSensorNoise; set { dynamicSensorNoise = value; UpdateImageDisturbanceParameters(); } }

        public bool IsDropoutActive => isCurrentlyDroppedOut;
        public Vector3 CurrentPositionOffset => currentPosOffset;
        public Quaternion CurrentRotationOffset => currentRotOffset;
        public Material ImageDisturbanceMaterial => imageDisturbanceMaterial;

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

            if (cameraRig == null) cameraRig = FindFirstObjectByType<OPTINAVCameraRig>();
            if (targetManager == null) targetManager = FindFirstObjectByType<OPTINAVTargetManager>();

            FindDisturbanceMaterial();
            ApplyAtmosphereHaze();
            UpdateImageDisturbanceParameters();
        }

        private void OnValidate()
        {
            FindDisturbanceMaterial();
            ApplyAtmosphereHaze();
            UpdateImageDisturbanceParameters();
        }

        private void FindDisturbanceMaterial()
        {
            if (imageDisturbanceMaterial == null)
            {
#if UNITY_EDITOR
                imageDisturbanceMaterial = UnityEditor.AssetDatabase.LoadAssetAtPath<Material>("Assets/Materials/M_OPTINAV_CameraDisturbance.mat");
#endif
            }
        }

        public void SetHazeEnabled(bool enabled)
        {
            enableHaze = enabled;
            ApplyAtmosphereHaze();
        }

        private void ApplyAtmosphereHaze()
        {
            RenderSettings.fog = enableHaze;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogDensity = hazeDensity;
            RenderSettings.fogColor = hazeColor;
        }

        private void Update()
        {
            float dt = Time.deltaTime;
            simulationTime += dt;

            UpdateVibrationAndJitter();
            UpdateVisibilityLoss(dt);
            UpdateImageDisturbanceParameters();
        }

        private void UpdateImageDisturbanceParameters()
        {
            if (imageDisturbanceMaterial == null) return;

            float activeBlur = enableBlur ? blurFactor : 0f;
            float activeNoise = enableSensorNoise ? sensorNoiseLevel : 0f;

            imageDisturbanceMaterial.SetFloat(PropBlurFactor, activeBlur);
            imageDisturbanceMaterial.SetFloat(PropBlurRadiusScale, blurRadiusScale);
            imageDisturbanceMaterial.SetFloat(PropSensorNoiseLevel, activeNoise);
            imageDisturbanceMaterial.SetFloat(PropNoiseSeed, (float)sensorNoiseSeed);
            imageDisturbanceMaterial.SetFloat(PropNoiseTime, dynamicSensorNoise ? Time.time * 25.0f : 0f);
            imageDisturbanceMaterial.SetFloat(PropNoiseDynamic, dynamicSensorNoise ? 1f : 0f);
        }

        private void UpdateVibrationAndJitter()
        {
            // Calculate Positional Vibration (Sinusoidal harmonics)
            if (enableVibration)
            {
                float vx = Mathf.Sin(simulationTime * Mathf.PI * 2f * vibrationFrequency.x) * vibrationAmplitude.x;
                float vy = Mathf.Cos(simulationTime * Mathf.PI * 2f * vibrationFrequency.y) * vibrationAmplitude.y;
                float vz = Mathf.Sin(simulationTime * Mathf.PI * 2f * vibrationFrequency.z) * vibrationAmplitude.z;
                currentPosOffset = new Vector3(vx, vy, vz);
            }
            else
            {
                currentPosOffset = Vector3.zero;
            }

            // Calculate Rotational Jitter (Deterministic multi-frequency Perlin noise)
            if (enableJitter)
            {
                float seedOffset = jitterSeed * 0.1f;
                float t = simulationTime * jitterFrequency;
                float jYaw = (Mathf.PerlinNoise(seedOffset + t, 0f) * 2f - 1f) * jitterAmplitude.x;
                float jPitch = (Mathf.PerlinNoise(seedOffset + 25f, t) * 2f - 1f) * jitterAmplitude.y;
                float jRoll = (Mathf.PerlinNoise(seedOffset + 50f, t * 0.7f) * 2f - 1f) * jitterAmplitude.z;
                currentRotOffset = Quaternion.Euler(jPitch, jYaw, jRoll);
            }
            else
            {
                currentRotOffset = Quaternion.identity;
            }

            // Layer onto camera rig without modifying original commanded angles
            if (cameraRig != null)
            {
                cameraRig.SetDisturbanceOffsets(currentPosOffset, currentRotOffset);
            }
        }

        private void UpdateVisibilityLoss(float dt)
        {
            if (!enableVisibilityLoss)
            {
                if (isCurrentlyDroppedOut)
                {
                    RestoreVisibility();
                }
                dropoutTimer = 0f;
                return;
            }

            dropoutTimer += dt;
            float cycleDuration = visibilityLossInterval + visibilityLossDuration;
            float cycleTime = dropoutTimer % cycleDuration;

            if (cycleTime >= visibilityLossInterval)
            {
                // In dropout window
                if (!isCurrentlyDroppedOut)
                {
                    isCurrentlyDroppedOut = true;
                    if (targetManager != null)
                    {
                        targetManager.SetPrimaryTargetVisibility(false);
                    }
                }
            }
            else
            {
                // Normal visible window
                if (isCurrentlyDroppedOut)
                {
                    RestoreVisibility();
                }
            }
        }

        private void RestoreVisibility()
        {
            isCurrentlyDroppedOut = false;
            if (targetManager != null)
            {
                targetManager.SetPrimaryTargetVisibility(true);
            }
        }

        private void OnDisable()
        {
            // Ensure disturbances clean up cleanly when disabled
            if (cameraRig != null)
            {
                cameraRig.SetDisturbanceOffsets(Vector3.zero, Quaternion.identity);
            }
            if (imageDisturbanceMaterial != null)
            {
                imageDisturbanceMaterial.SetFloat(PropBlurFactor, 0f);
                imageDisturbanceMaterial.SetFloat(PropSensorNoiseLevel, 0f);
            }
            RestoreVisibility();
        }
    }
}
