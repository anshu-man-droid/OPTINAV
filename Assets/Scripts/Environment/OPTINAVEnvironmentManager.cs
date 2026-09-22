using UnityEngine;

namespace OPTINAV.Environment
{
    /// <summary>
    /// Central manager for the OPTINAV 3D simulation environment.
    /// Defines world bounds, reference coordinates, and central environmental parameters
    /// for Free Space Optical Communication (FSOC) terminal alignment simulation.
    /// </summary>
    [ExecuteAlways]
    public class OPTINAVEnvironmentManager : MonoBehaviour
    {
        public static OPTINAVEnvironmentManager Instance { get; private set; }

        [Header("Simulation World Reference")]
        [Tooltip("Simulation origin reference point (nominally 0,0,0)")]
        [SerializeField] private Vector3 simulationOrigin = Vector3.zero;

        [Tooltip("Active simulation area dimensions (Width X, Height Y, Depth Z) in meters")]
        [SerializeField] private Vector3 simulationDimensions = new Vector3(200f, 60f, 250f);

        [Header("Environmental References")]
        [Tooltip("Reference to the ground plane GameObject")]
        [SerializeField] private GameObject groundPlane;

        [Tooltip("Main directional light simulating sun/sky illumination")]
        [SerializeField] private Light mainSunLight;

        [Header("Atmospheric Conditions")]
        [SerializeField] private bool enableAtmosphericHaze = false;
        [SerializeField] private float hazeDensity = 0.015f;
        [SerializeField] private Color hazeColor = new Color(0.72f, 0.78f, 0.85f, 1f);

        public Vector3 Origin => simulationOrigin;
        public Vector3 Dimensions => simulationDimensions;
        public Bounds SimulationBounds => new Bounds(simulationOrigin + new Vector3(0, simulationDimensions.y * 0.5f, simulationDimensions.z * 0.5f), simulationDimensions);

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

            ApplyAtmosphericSettings();
        }

        private void OnValidate()
        {
            ApplyAtmosphericSettings();
        }

        public void ApplyAtmosphericSettings()
        {
            RenderSettings.fog = enableAtmosphericHaze;
            RenderSettings.fogMode = FogMode.ExponentialSquared;
            RenderSettings.fogDensity = hazeDensity;
            RenderSettings.fogColor = hazeColor;
        }

        public void SetHaze(bool enabled, float density = -1f)
        {
            enableAtmosphericHaze = enabled;
            if (density >= 0f)
            {
                hazeDensity = density;
            }
            ApplyAtmosphericSettings();
        }

        /// <summary>
        /// Checks whether a given position in world space is inside the designated simulation volume.
        /// </summary>
        public bool IsWithinBounds(Vector3 position)
        {
            return SimulationBounds.Contains(position);
        }

        /// <summary>
        /// Clamps a world position to ensure it remains strictly inside the simulation bounds.
        /// </summary>
        public Vector3 ClampToBounds(Vector3 position)
        {
            Bounds b = SimulationBounds;
            return new Vector3(
                Mathf.Clamp(position.x, b.min.x, b.max.x),
                Mathf.Clamp(position.y, b.min.y, b.max.y),
                Mathf.Clamp(position.z, b.min.z, b.max.z)
            );
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = new Color(0f, 0.8f, 1f, 0.3f);
            Bounds b = SimulationBounds;
            Gizmos.DrawWireCube(b.center, b.size);

            // Draw Coordinate Axes at origin
            Gizmos.color = Color.red;   // X: Horizontal
            Gizmos.DrawRay(simulationOrigin, Vector3.right * 10f);
            Gizmos.color = Color.green; // Y: Vertical
            Gizmos.DrawRay(simulationOrigin, Vector3.up * 10f);
            Gizmos.color = Color.blue;  // Z: Optical Axis Depth
            Gizmos.DrawRay(simulationOrigin, Vector3.forward * 10f);
        }
    }
}
