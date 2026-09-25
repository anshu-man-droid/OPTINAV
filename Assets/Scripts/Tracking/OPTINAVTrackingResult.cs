using System;
using UnityEngine;

namespace OPTINAV.Tracking
{
    /// <summary>
    /// Explicit tracking lifecycle states for Member 4 Tracking & Prediction subsystem.
    /// </summary>
    public enum TrackingState
    {
        /// <summary>No valid detection received, no active track.</summary>
        SEARCH = 0,

        /// <summary>Candidate detections arriving, filter initializing and stabilizing.</summary>
        ACQUISITION = 1,

        /// <summary>Active track established and stabilized, regular Kalman correction.</summary>
        TRACKING = 2,

        /// <summary>Detection temporarily missing, continuing Kalman prediction only.</summary>
        COASTING = 3,

        /// <summary>Target absent beyond coasting limit, track cleared/invalidated.</summary>
        LOST = 4
    }

    /// <summary>
    /// Contract for optical beacon tracking and prediction results produced by Member 4.
    /// Exposes full kinematic state, filter uncertainties, tracking state machine status,
    /// and clean separation between measured and predicted coordinates.
    /// Directly consumed by Member 5 (PID / Pan-Tilt Control) and Member 6 (System Integration & GUI).
    /// </summary>
    [Serializable]
    public struct TrackingResult
    {
        [Tooltip("Identifier of the processed video/detection frame")]
        public long frameId;

        [Tooltip("True if the tracker maintains an active track (ACQUISITION, TRACKING, or COASTING)")]
        public bool hasTrack;

        [Tooltip("True if an actual M3 measurement was incorporated in this frame update")]
        public bool hasMeasurement;

        [Tooltip("Current best estimate of horizontal beacon position in image pixels [0, width)")]
        public float trackedPixelX;

        [Tooltip("Current best estimate of vertical beacon position in image pixels [0, height)")]
        public float trackedPixelY;

        [Tooltip("Prior Kalman predicted horizontal position before measurement update (pixels)")]
        public float predictedPixelX;

        [Tooltip("Prior Kalman predicted vertical position before measurement update (pixels)")]
        public float predictedPixelY;

        [Tooltip("Estimated image-space horizontal velocity in pixels/second")]
        public float velocityX;

        [Tooltip("Estimated image-space vertical velocity in pixels/second")]
        public float velocityY;

        [Tooltip("Confidence score [0, 1] of the track / latest measurement")]
        public float confidence;

        [Tooltip("Count of consecutive frames where beacon detection was missing")]
        public int consecutiveMisses;

        [Tooltip("Active tracking state machine state")]
        public TrackingState state;

        [Tooltip("Positional variance / uncertainty along X axis (pixels^2)")]
        public float covarianceX;

        [Tooltip("Positional variance / uncertainty along Y axis (pixels^2)")]
        public float covarianceY;

        [Tooltip("System timestamp of the tracking update (seconds)")]
        public double timestamp;

        [Tooltip("Processing latency of the Kalman tracking step in milliseconds")]
        public float latencyMs;

        // Convenience Accessors
        public Vector2 TrackedPixel => new Vector2(trackedPixelX, trackedPixelY);
        public Vector2 PredictedPixel => new Vector2(predictedPixelX, predictedPixelY);
        public Vector2 Velocity => new Vector2(velocityX, velocityY);
        public bool IsCoasting => state == TrackingState.COASTING;
        public bool IsTracking => state == TrackingState.TRACKING;
        public bool IsAcquiring => state == TrackingState.ACQUISITION;
        public bool IsLost => state == TrackingState.LOST;
        public bool IsSearching => state == TrackingState.SEARCH;

        // Normalized Viewport Accessors [0, 1] (Nominal frame size 640x360)
        public float NormalizedTrackedX => trackedPixelX >= 0f ? (trackedPixelX / 640.0f) : -1.0f;
        public float NormalizedTrackedY => trackedPixelY >= 0f ? (trackedPixelY / 360.0f) : -1.0f;
        public Vector2 NormalizedTrackedCenter => new Vector2(NormalizedTrackedX, NormalizedTrackedY);

        /// <summary>
        /// Creates an empty/default tracking result representing an uninitialized or reset tracker.
        /// </summary>
        public static TrackingResult CreateEmpty(long fid = 0)
        {
            return new TrackingResult
            {
                frameId = fid,
                hasTrack = false,
                hasMeasurement = false,
                trackedPixelX = -1f,
                trackedPixelY = -1f,
                predictedPixelX = -1f,
                predictedPixelY = -1f,
                velocityX = 0f,
                velocityY = 0f,
                confidence = 0f,
                consecutiveMisses = 0,
                state = TrackingState.SEARCH,
                covarianceX = 0f,
                covarianceY = 0f,
                timestamp = 0,
                latencyMs = 0f
            };
        }
    }
}
