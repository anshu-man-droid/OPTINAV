using UnityEngine;

namespace OPTINAV.Targets
{
    public enum BeaconMovementMode
    {
        Stationary,
        Linear,
        Circular,
        Accelerating,
        RandomBounded
    }

    /// <summary>
    /// Controls the physical movement and optical emission of the primary FSOC beacon.
    /// Supports deterministic trajectories (Stationary, Linear, Circular, Accelerating, Random Bounded),
    /// bounds containment, velocity calculation, and optical emission toggling.
    /// </summary>
    [SelectionBase]
    public class BeaconController : MonoBehaviour
    {
        [Header("Movement Configuration")]
        [Tooltip("Active trajectory mode")]
        [SerializeField] private BeaconMovementMode movementMode = BeaconMovementMode.Circular;

        [Tooltip("Nominal linear/tangential speed in meters/second")]
        [SerializeField] private float speed = 5.0f;

        [Tooltip("Radius for circular trajectory or amplitude for linear oscillation (meters)")]
        [SerializeField] private float amplitudeOrRadius = 8.0f;

        [Tooltip("Center origin of the beacon trajectory")]
        [SerializeField] private Vector3 centerPosition = new Vector3(0f, 15f, 60f);

        [Tooltip("Direction vector for linear trajectory mode")]
        [SerializeField] private Vector3 linearAxis = Vector3.right;

        [Tooltip("Plane of motion for circular mode: XY (transverse), XZ (horizontal), YZ (vertical)")]
        [SerializeField] private CircularPlane circularPlane = CircularPlane.XY;

        [Tooltip("Acceleration rate in m/s^2 for accelerating mode")]
        [SerializeField] private float acceleration = 2.0f;

        [Tooltip("Maximum speed clamp for accelerating mode (m/s)")]
        [SerializeField] private float maxSpeed = 25.0f;

        [Tooltip("Bounding box limiting all beacon movement")]
        [SerializeField] private Vector3 movementBoundsSize = new Vector3(40f, 25f, 50f);

        [Tooltip("Random seed for reproducible pseudo-random motion")]
        [SerializeField] private int randomSeed = 42;

        [Header("Optical Emission References")]
        [Tooltip("MeshRenderer of the beacon core")]
        [SerializeField] private MeshRenderer beaconRenderer;

        [Tooltip("Light component simulating laser/LED optical beacon emission")]
        [SerializeField] private Light beaconLight;

        [Tooltip("Emissive material color for optical identification")]
        [SerializeField] [ColorUsage(true, true)] private Color emissionColor = new Color(0f, 2f, 1f, 1f);

        // State variables
        private Vector3 currentPosition;
        private Vector3 currentVelocity;
        private bool isVisible = true;
        private float simulationTime = 0f;
        private float currentDynamicSpeed;
        private int linearSign = 1;
        private Vector3 randomNoiseSeedOffset;

        public enum CircularPlane { XY, XZ, YZ }

        // Public APIs for Member 3 (Detection), Member 4 (Kalman), Member 5 (PID)
        public BeaconMovementMode MovementMode => movementMode;
        public Vector3 CurrentPosition => currentPosition;
        public Vector3 CurrentVelocity => currentVelocity;
        public bool IsVisible => isVisible;
        public Vector3 CenterPosition { get => centerPosition; set => centerPosition = value; }
        public float Speed { get => speed; set => speed = value; }
        public float AmplitudeOrRadius { get => amplitudeOrRadius; set => amplitudeOrRadius = value; }

        public Bounds MovementBounds => new Bounds(centerPosition, movementBoundsSize);

        private void Awake()
        {
            if (beaconRenderer == null)
            {
                beaconRenderer = GetComponentInChildren<MeshRenderer>();
            }
            if (beaconLight == null)
            {
                beaconLight = GetComponentInChildren<Light>();
            }

            InitRandomSeed();
            ResetSimulation();
        }

        private void OnValidate()
        {
            if (linearAxis == Vector3.zero) linearAxis = Vector3.right;
            linearAxis.Normalize();
            if (speed < 0f) speed = 0f;
            if (amplitudeOrRadius < 0f) amplitudeOrRadius = 0f;
        }

        public void InitRandomSeed()
        {
            Random.InitState(randomSeed);
            randomNoiseSeedOffset = new Vector3(
                Random.Range(0f, 1000f),
                Random.Range(0f, 1000f),
                Random.Range(0f, 1000f)
            );
        }

        public void ResetSimulation()
        {
            simulationTime = 0f;
            currentDynamicSpeed = speed;
            linearSign = 1;
            currentPosition = centerPosition;
            currentVelocity = Vector3.zero;
            transform.position = currentPosition;
            SetVisibility(true);
        }

        public void SetMovementMode(BeaconMovementMode mode)
        {
            movementMode = mode;
            ResetSimulation();
        }

        private void Update()
        {
            float dt = Time.deltaTime;
            if (dt <= 0f) return;

            simulationTime += dt;
            Vector3 previousPosition = currentPosition;
            Vector3 targetPosition = centerPosition;

            switch (movementMode)
            {
                case BeaconMovementMode.Stationary:
                    targetPosition = centerPosition;
                    currentVelocity = Vector3.zero;
                    break;

                case BeaconMovementMode.Linear:
                    targetPosition = CalculateLinearPosition(simulationTime);
                    break;

                case BeaconMovementMode.Circular:
                    targetPosition = CalculateCircularPosition(simulationTime);
                    break;

                case BeaconMovementMode.Accelerating:
                    targetPosition = CalculateAcceleratingPosition(dt);
                    break;

                case BeaconMovementMode.RandomBounded:
                    targetPosition = CalculateRandomBoundedPosition(simulationTime);
                    break;
            }

            // Enforce simulation bounds
            targetPosition = ClampToBounds(targetPosition);

            // Compute empirical instantaneous velocity
            currentVelocity = (targetPosition - previousPosition) / dt;
            currentPosition = targetPosition;
            transform.position = currentPosition;
        }

        private Vector3 CalculateLinearPosition(float time)
        {
            // Sinusoidal oscillation along linearAxis bounded by amplitude
            float phase = (speed / Mathf.Max(0.001f, amplitudeOrRadius)) * time;
            float offset = Mathf.Sin(phase) * amplitudeOrRadius;
            return centerPosition + linearAxis.normalized * offset;
        }

        private Vector3 CalculateCircularPosition(float time)
        {
            float angularVelocity = speed / Mathf.Max(0.001f, amplitudeOrRadius);
            float angle = angularVelocity * time;
            float cos = Mathf.Cos(angle) * amplitudeOrRadius;
            float sin = Mathf.Sin(angle) * amplitudeOrRadius;

            Vector3 localOffset;
            switch (circularPlane)
            {
                case CircularPlane.XY:
                    localOffset = new Vector3(cos, sin, 0f);
                    break;
                case CircularPlane.XZ:
                    localOffset = new Vector3(cos, 0f, sin);
                    break;
                case CircularPlane.YZ:
                    localOffset = new Vector3(0f, cos, sin);
                    break;
                default:
                    localOffset = new Vector3(cos, sin, 0f);
                    break;
            }

            return centerPosition + localOffset;
        }

        private Vector3 CalculateAcceleratingPosition(float dt)
        {
            currentDynamicSpeed = Mathf.Min(maxSpeed, currentDynamicSpeed + acceleration * dt);
            Vector3 newPos = currentPosition + linearAxis.normalized * (linearSign * currentDynamicSpeed * dt);

            // Bounce upon reaching movement bounds
            Bounds b = MovementBounds;
            if (!b.Contains(newPos))
            {
                linearSign *= -1;
                newPos = ClampToBounds(newPos);
            }
            return newPos;
        }

        private Vector3 CalculateRandomBoundedPosition(float time)
        {
            float freq = Mathf.Max(0.01f, speed * 0.15f);
            float nx = Mathf.PerlinNoise(randomNoiseSeedOffset.x + time * freq, 0f) * 2f - 1f;
            float ny = Mathf.PerlinNoise(randomNoiseSeedOffset.y + time * freq, 50f) * 2f - 1f;
            float nz = Mathf.PerlinNoise(randomNoiseSeedOffset.z + time * freq, 100f) * 2f - 1f;

            Vector3 halfSize = movementBoundsSize * 0.5f;
            Vector3 offset = new Vector3(
                nx * halfSize.x,
                ny * halfSize.y,
                nz * halfSize.z
            );

            return centerPosition + offset;
        }

        private Vector3 ClampToBounds(Vector3 pos)
        {
            Bounds b = MovementBounds;
            return new Vector3(
                Mathf.Clamp(pos.x, b.min.x, b.max.x),
                Mathf.Clamp(pos.y, b.min.y, b.max.y),
                Mathf.Clamp(pos.z, b.min.z, b.max.z)
            );
        }

        /// <summary>
        /// Toggles beacon visibility (mesh and optical emission source).
        /// Used for visibility loss and re-acquisition stress tests.
        /// </summary>
        public void SetVisibility(bool visible)
        {
            isVisible = visible;
            if (beaconRenderer != null)
            {
                beaconRenderer.enabled = visible;
            }
            if (beaconLight != null)
            {
                beaconLight.enabled = visible;
            }
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = new Color(0f, 1f, 0.5f, 0.25f);
            Gizmos.DrawWireCube(centerPosition, movementBoundsSize);

            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(centerPosition, 0.5f);

            if (movementMode == BeaconMovementMode.Circular)
            {
                Gizmos.color = Color.yellow;
                // Draw circle approximate
                int segments = 32;
                Vector3 prevPoint = Vector3.zero;
                for (int i = 0; i <= segments; i++)
                {
                    float angle = (i / (float)segments) * Mathf.PI * 2f;
                    float cos = Mathf.Cos(angle) * amplitudeOrRadius;
                    float sin = Mathf.Sin(angle) * amplitudeOrRadius;
                    Vector3 pt = centerPosition + (circularPlane == CircularPlane.XY ? new Vector3(cos, sin, 0) : (circularPlane == CircularPlane.XZ ? new Vector3(cos, 0, sin) : new Vector3(0, cos, sin)));
                    if (i > 0)
                    {
                        Gizmos.DrawLine(prevPoint, pt);
                    }
                    prevPoint = pt;
                }
            }
        }
    }
}
