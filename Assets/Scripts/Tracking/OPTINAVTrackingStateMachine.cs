using System;
using UnityEngine;

namespace OPTINAV.Tracking
{
    /// <summary>
    /// Explicit tracking state machine governing lifecycle transitions for optical beacon tracking.
    /// Manages SEARCH -> ACQUISITION -> TRACKING -> COASTING -> LOST lifecycle.
    /// Fully configurable thresholds for acquisition confirmation, coasting tolerance, and loss invalidation.
    /// </summary>
    [Serializable]
    public class OPTINAVTrackingStateMachine
    {
        [Header("State Machine Thresholds")]
        [Tooltip("Consecutive valid detections required in ACQUISITION before confirming TRACKING")]
        [SerializeField] private int acquisitionHitsRequired = 3;

        [Tooltip("Maximum consecutive missed frames allowed in COASTING before declaring track LOST")]
        [SerializeField] private int maxCoastingFrames = 8;

        [Tooltip("Consecutive missed frames in ACQUISITION before aborting back to SEARCH")]
        [SerializeField] private int acquisitionMissThreshold = 2;

        [Header("Runtime State")]
        [SerializeField] private TrackingState currentState = TrackingState.SEARCH;
        [SerializeField] private int consecutiveHits = 0;
        [SerializeField] private int consecutiveMisses = 0;
        [SerializeField] private int totalFramesInState = 0;
        [SerializeField] private int totalReacquisitions = 0;

        // Public Properties & Threshold Accessors
        public TrackingState CurrentState => currentState;
        public int ConsecutiveHits => consecutiveHits;
        public int ConsecutiveMisses => consecutiveMisses;
        public int TotalFramesInState => totalFramesInState;
        public int TotalReacquisitions => totalReacquisitions;

        public int AcquisitionHitsRequired
        {
            get => acquisitionHitsRequired;
            set => acquisitionHitsRequired = Mathf.Max(1, value);
        }

        public int MaxCoastingFrames
        {
            get => maxCoastingFrames;
            set => maxCoastingFrames = Mathf.Max(1, value);
        }

        public int AcquisitionMissThreshold
        {
            get => acquisitionMissThreshold;
            set => acquisitionMissThreshold = Mathf.Max(1, value);
        }

        public bool HasTrack =>
            currentState == TrackingState.ACQUISITION ||
            currentState == TrackingState.TRACKING ||
            currentState == TrackingState.COASTING;

        public bool IsTracking => currentState == TrackingState.TRACKING;
        public bool IsCoasting => currentState == TrackingState.COASTING;
        public bool IsAcquiring => currentState == TrackingState.ACQUISITION;
        public bool IsLost => currentState == TrackingState.LOST;
        public bool IsSearching => currentState == TrackingState.SEARCH;

        /// <summary>
        /// Fired whenever the tracking state transitions (previousState, newState).
        /// </summary>
        public event Action<TrackingState, TrackingState> OnStateTransition;

        public OPTINAVTrackingStateMachine(int acquisitionHits = 3, int maxCoasting = 8, int acqMissLimit = 2)
        {
            acquisitionHitsRequired = Mathf.Max(1, acquisitionHits);
            maxCoastingFrames = Mathf.Max(1, maxCoasting);
            acquisitionMissThreshold = Mathf.Max(1, acqMissLimit);
            Reset();
        }

        /// <summary>
        /// Steps the tracking state machine based on whether a valid detection was received in the current frame.
        /// Returns the new active TrackingState.
        /// </summary>
        /// <param name="detectionValid">True if detector produced a valid detection with valid coordinates</param>
        /// <returns>Active TrackingState after transition</returns>
        public TrackingState Step(bool detectionValid)
        {
            TrackingState previousState = currentState;

            switch (currentState)
            {
                case TrackingState.SEARCH:
                    if (detectionValid)
                    {
                        consecutiveHits = 1;
                        consecutiveMisses = 0;
                        TransitionTo(TrackingState.ACQUISITION);
                    }
                    else
                    {
                        consecutiveHits = 0;
                        consecutiveMisses++;
                        totalFramesInState++;
                    }
                    break;

                case TrackingState.ACQUISITION:
                    if (detectionValid)
                    {
                        consecutiveHits++;
                        consecutiveMisses = 0;
                        if (consecutiveHits >= acquisitionHitsRequired)
                        {
                            TransitionTo(TrackingState.TRACKING);
                        }
                        else
                        {
                            totalFramesInState++;
                        }
                    }
                    else
                    {
                        consecutiveHits = 0;
                        consecutiveMisses++;
                        if (consecutiveMisses >= acquisitionMissThreshold)
                        {
                            // Unstable acquisition aborted
                            TransitionTo(TrackingState.SEARCH);
                        }
                        else
                        {
                            totalFramesInState++;
                        }
                    }
                    break;

                case TrackingState.TRACKING:
                    if (detectionValid)
                    {
                        consecutiveHits++;
                        consecutiveMisses = 0;
                        totalFramesInState++;
                    }
                    else
                    {
                        consecutiveHits = 0;
                        consecutiveMisses = 1;
                        TransitionTo(TrackingState.COASTING);
                    }
                    break;

                case TrackingState.COASTING:
                    if (detectionValid)
                    {
                        // Target reappeared! Kalman will correct and restore track
                        consecutiveHits = 1;
                        consecutiveMisses = 0;
                        totalReacquisitions++;
                        TransitionTo(TrackingState.TRACKING);
                    }
                    else
                    {
                        consecutiveMisses++;
                        if (consecutiveMisses > maxCoastingFrames)
                        {
                            // Coasting limit exceeded -> track is declared LOST
                            consecutiveHits = 0;
                            TransitionTo(TrackingState.LOST);
                        }
                        else
                        {
                            totalFramesInState++;
                        }
                    }
                    break;

                case TrackingState.LOST:
                    if (detectionValid)
                    {
                        // Immediate reacquisition sequence starts
                        consecutiveHits = 1;
                        consecutiveMisses = 0;
                        TransitionTo(TrackingState.ACQUISITION);
                    }
                    else
                    {
                        // Return to SEARCH baseline
                        consecutiveHits = 0;
                        consecutiveMisses++;
                        TransitionTo(TrackingState.SEARCH);
                    }
                    break;
            }

            return currentState;
        }

        private void TransitionTo(TrackingState newState)
        {
            if (currentState == newState)
            {
                totalFramesInState++;
                return;
            }

            TrackingState prev = currentState;
            currentState = newState;
            totalFramesInState = 1;
            OnStateTransition?.Invoke(prev, newState);
        }

        /// <summary>
        /// Explicitly resets the state machine back to clean SEARCH state.
        /// </summary>
        public void Reset()
        {
            TrackingState prev = currentState;
            currentState = TrackingState.SEARCH;
            consecutiveHits = 0;
            consecutiveMisses = 0;
            totalFramesInState = 0;
            if (prev != TrackingState.SEARCH)
            {
                OnStateTransition?.Invoke(prev, TrackingState.SEARCH);
            }
        }

        /// <summary>
        /// Explicitly sets state for unit testing or specific recovery workflows.
        /// </summary>
        public void ForceState(TrackingState state)
        {
            TransitionTo(state);
        }
    }
}
