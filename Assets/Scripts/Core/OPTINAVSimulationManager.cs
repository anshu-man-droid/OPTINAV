using UnityEngine;
using OPTINAV.Environment;
using OPTINAV.Targets;
using OPTINAV.CameraSystem;
using OPTINAV.Disturbances;

namespace OPTINAV.Core
{
    /// <summary>
    /// Central coordinator and experiment orchestrator for the OPTINAV simulation testbed.
    /// Manages simulation lifecycle, configuration presets, and integrates environment,
    /// target, camera rig, and disturbance subsystems.
    /// </summary>
    public class OPTINAVSimulationManager : MonoBehaviour
    {
        public static OPTINAVSimulationManager Instance { get; private set; }

        [Header("Subsystem References")]
        [SerializeField] private OPTINAVEnvironmentManager environmentManager;
        [SerializeField] private OPTINAVTargetManager targetManager;
        [SerializeField] private OPTINAVCameraRig cameraRig;
        [SerializeField] private OPTINAVCameraController cameraController;
        [SerializeField] private OPTINAVDisturbanceController disturbanceController;

        [Header("Simulation State")]
        [SerializeField] private bool autoStartSimulation = true;
        [SerializeField] [Range(0.1f, 3.0f)] private float timeScale = 1.0f;

        private float elapsedSimulationTime = 0f;
        private bool isSimulationRunning = false;

        public bool IsRunning => isSimulationRunning;
        public float ElapsedTime => elapsedSimulationTime;

        public OPTINAVEnvironmentManager Environment => environmentManager;
        public OPTINAVTargetManager Targets => targetManager;
        public OPTINAVCameraRig Rig => cameraRig;
        public OPTINAVCameraController Camera => cameraController;
        public OPTINAVDisturbanceController Disturbances => disturbanceController;

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

            FindMissingReferences();
        }

        private void Start()
        {
            Time.timeScale = timeScale;
            if (autoStartSimulation)
            {
                StartSimulation();
            }
        }

        private void OnValidate()
        {
            FindMissingReferences();
            if (Application.isPlaying)
            {
                Time.timeScale = timeScale;
            }
        }

        public void FindMissingReferences()
        {
            if (environmentManager == null) environmentManager = FindFirstObjectByType<OPTINAVEnvironmentManager>();
            if (targetManager == null) targetManager = FindFirstObjectByType<OPTINAVTargetManager>();
            if (cameraRig == null) cameraRig = FindFirstObjectByType<OPTINAVCameraRig>();
            if (cameraController == null) cameraController = FindFirstObjectByType<OPTINAVCameraController>();
            if (disturbanceController == null) disturbanceController = FindFirstObjectByType<OPTINAVDisturbanceController>();
        }

        public void StartSimulation()
        {
            isSimulationRunning = true;
        }

        public void PauseSimulation()
        {
            isSimulationRunning = false;
        }

        public void ResetSimulation()
        {
            elapsedSimulationTime = 0f;
            if (targetManager != null) targetManager.ResetSimulation();
            if (cameraRig != null) cameraRig.ResetOrientation();
            isSimulationRunning = true;
        }

        private void Update()
        {
            if (!isSimulationRunning) return;
            elapsedSimulationTime += Time.deltaTime;
        }
    }
}
