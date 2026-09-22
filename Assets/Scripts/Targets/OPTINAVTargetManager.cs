using System.Collections.Generic;
using UnityEngine;

namespace OPTINAV.Targets
{
    /// <summary>
    /// Central manager for FSOC beacon target and optional distractors.
    /// Exposes uniform APIs for target state (position, velocity, visibility) to AI detection,
    /// Kalman estimation, and pan-tilt tracking systems.
    /// </summary>
    public class OPTINAVTargetManager : MonoBehaviour
    {
        public static OPTINAVTargetManager Instance { get; private set; }

        [Header("Primary Target")]
        [Tooltip("Primary FSOC optical beacon controller")]
        [SerializeField] private BeaconController primaryBeacon;

        [Header("Distractor Targets")]
        [Tooltip("Optional distractor objects to test detection discrimination")]
        [SerializeField] private List<GameObject> distractorTargets = new List<GameObject>();

        [Tooltip("Enable or disable distractor objects")]
        [SerializeField] private bool enableDistractors = false;

        // APIs for Member 3 (Detection), Member 4 (Kalman), Member 5 (PID)
        public BeaconController PrimaryBeacon => primaryBeacon;
        public Vector3 PrimaryTargetPosition => primaryBeacon != null ? primaryBeacon.CurrentPosition : Vector3.zero;
        public Vector3 PrimaryTargetVelocity => primaryBeacon != null ? primaryBeacon.CurrentVelocity : Vector3.zero;
        public bool IsPrimaryTargetVisible => primaryBeacon != null && primaryBeacon.IsVisible;
        public BeaconMovementMode PrimaryMovementMode => primaryBeacon != null ? primaryBeacon.MovementMode : BeaconMovementMode.Stationary;
        public IReadOnlyList<GameObject> Distractors => distractorTargets;
        public bool DistractorsEnabled => enableDistractors;

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

            if (primaryBeacon == null)
            {
                primaryBeacon = FindFirstObjectByType<BeaconController>();
            }

            if (distractorTargets == null || distractorTargets.Count == 0)
            {
                var distractorsRoot = transform.Find("Distractors");
                if (distractorsRoot != null)
                {
                    distractorTargets = new List<GameObject>();
                    foreach (Transform child in distractorsRoot)
                    {
                        distractorTargets.Add(child.gameObject);
                    }
                }
            }

            ApplyDistractorState();
        }

        private void OnValidate()
        {
#if UNITY_EDITOR
            if (!Application.isPlaying)
            {
                UnityEditor.EditorApplication.delayCall += () =>
                {
                    if (this != null) ApplyDistractorState();
                };
                return;
            }
#endif
            ApplyDistractorState();
        }

        public void SetDistractorsActive(bool active)
        {
            enableDistractors = active;
            ApplyDistractorState();
        }

        private void ApplyDistractorState()
        {
            if (distractorTargets == null) return;
            foreach (var distractor in distractorTargets)
            {
                if (distractor != null)
                {
                    distractor.SetActive(enableDistractors);
                }
            }
        }

        public void ResetSimulation()
        {
            if (primaryBeacon != null)
            {
                primaryBeacon.ResetSimulation();
            }
        }

        public void SetPrimaryMovementMode(BeaconMovementMode mode)
        {
            if (primaryBeacon != null)
            {
                primaryBeacon.SetMovementMode(mode);
            }
        }

        public void SetPrimaryTargetVisibility(bool visible)
        {
            if (primaryBeacon != null)
            {
                primaryBeacon.SetVisibility(visible);
            }
        }
    }
}
