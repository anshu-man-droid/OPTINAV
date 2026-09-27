using System;
using UnityEngine;

namespace OPTINAV.Control
{
    /// <summary>
    /// High-performance, discrete-time single-axis PID controller designed for optical tracking and camera gimbal alignment.
    /// Features:
    /// - Proportional, Integral, and Derivative control
    /// - Dynamic output saturation clamping (MinOutput / MaxOutput)
    /// - Integral anti-windup (conditional integration clamping during output saturation)
    /// - Dedicated integral accumulator limit
    /// - Low-pass filtered derivative (attenuates sensor/detection noise)
    /// - Configurable error deadband (prevents steady-state actuator hunting/chatter)
    /// - Allocation-free execution in Update/FixedUpdate loops
    /// </summary>
    [Serializable]
    public class OPTINAVPIDController
    {
        [Header("PID Gains")]
        [Tooltip("Proportional gain: responds directly to current tracking error")]
        [SerializeField] private float kp = 1.0f;

        [Tooltip("Integral gain: eliminates steady-state pointing error over time")]
        [SerializeField] private float ki = 0.0f;

        [Tooltip("Derivative gain: dampens oscillations and anticipates target motion")]
        [SerializeField] private float kd = 0.0f;

        [Header("Output Saturation Limits")]
        [Tooltip("Minimum allowable control output (e.g. minimum gimbal angular rate or angle)")]
        [SerializeField] private float minOutput = -60.0f;

        [Tooltip("Maximum allowable control output (e.g. maximum gimbal angular rate or angle)")]
        [SerializeField] private float maxOutput = 60.0f;

        [Header("Anti-Windup & Integral Clamping")]
        [Tooltip("Enable anti-windup: halts integration when output is saturated in direction of error")]
        [SerializeField] private bool enableAntiWindup = true;

        [Tooltip("Clamp the accumulated integral term to prevent runaway windup")]
        [SerializeField] private bool enableIntegralLimit = true;

        [Tooltip("Maximum absolute value allowed for the accumulated integral (integral accumulator clamp)")]
        [SerializeField] private float maxIntegral = 30.0f;

        [Header("Derivative Filtering & Noise Suppression")]
        [Tooltip("First-order low-pass filter time constant (tau in seconds) for derivative action. Set to 0 to disable filtering.")]
        [Range(0f, 0.5f)]
        [SerializeField] private float derivativeFilterTimeConstant = 0.02f;

        [Header("Deadband")]
        [Tooltip("Error threshold below which error is treated as zero to prevent mechanical hunting/chatter")]
        [SerializeField] private float deadband = 0.0f;

        // Internal State Variables (zero-allocation state)
        private float integral = 0f;
        private float lastError = 0f;
        private float filteredDerivative = 0f;
        private float lastOutput = 0f;
        private bool isFirstUpdate = true;
        private bool isSaturated = false;
        private bool freezeIntegration = false;

        // Telemetry & Diagnostics (allocated as fields for zero-allocation access)
        private float proportionalTerm = 0f;
        private float integralTerm = 0f;
        private float derivativeTerm = 0f;

        // Public Gain Properties
        public float Kp { get => kp; set => kp = value; }
        public float Ki { get => ki; set => ki = value; }
        public float Kd { get => kd; set => kd = value; }
        public bool FreezeIntegration { get => freezeIntegration; set => freezeIntegration = value; }

        // Limits & Tuning Properties
        public float MinOutput
        {
            get => minOutput;
            set
            {
                minOutput = value;
                if (maxOutput < minOutput) maxOutput = minOutput;
            }
        }

        public float MaxOutput
        {
            get => maxOutput;
            set
            {
                maxOutput = value;
                if (minOutput > maxOutput) minOutput = maxOutput;
            }
        }

        public bool EnableAntiWindup { get => enableAntiWindup; set => enableAntiWindup = value; }
        public bool EnableIntegralLimit { get => enableIntegralLimit; set => enableIntegralLimit = value; }
        public float MaxIntegral { get => maxIntegral; set => maxIntegral = Mathf.Max(0f, value); }
        public float DerivativeFilterTimeConstant { get => derivativeFilterTimeConstant; set => derivativeFilterTimeConstant = Mathf.Max(0f, value); }
        public float Deadband { get => deadband; set => deadband = Mathf.Max(0f, value); }

        // Telemetry & State Accessors
        public float LastError => lastError;
        public float LastOutput => lastOutput;
        public float ProportionalTerm => proportionalTerm;
        public float IntegralTerm => integralTerm;
        public float DerivativeTerm => derivativeTerm;
        public float AccumulatedIntegral => integral;
        public float FilteredDerivative => filteredDerivative;
        public bool IsSaturated => isSaturated;
        public bool IsFirstUpdate => isFirstUpdate;

        /// <summary>
        /// Default constructor.
        /// </summary>
        public OPTINAVPIDController()
        {
            Reset();
        }

        /// <summary>
        /// Parameterized constructor with standard PID configuration.
        /// </summary>
        public OPTINAVPIDController(
            float kp,
            float ki,
            float kd,
            float minOutput = -60f,
            float maxOutput = 60f,
            float deadband = 0f,
            float filterTimeConstant = 0.02f)
        {
            this.kp = kp;
            this.ki = ki;
            this.kd = kd;
            this.minOutput = minOutput;
            this.maxOutput = maxOutput;
            this.deadband = Mathf.Max(0f, deadband);
            this.derivativeFilterTimeConstant = Mathf.Max(0f, filterTimeConstant);
            Reset();
        }

        /// <summary>
        /// Resets all internal controller memory (integral accumulator, last error, filtered derivative).
        /// Call whenever tracking is lost, on target reacquisition, or when switching tracking modes.
        /// </summary>
        public void Reset()
        {
            integral = 0f;
            lastError = 0f;
            filteredDerivative = 0f;
            lastOutput = 0f;
            proportionalTerm = 0f;
            integralTerm = 0f;
            derivativeTerm = 0f;
            isSaturated = false;
            freezeIntegration = false;
            isFirstUpdate = true;
        }

        /// <summary>
        /// Computes the PID control output given an instantaneous error and time delta.
        /// Completely allocation-free.
        /// </summary>
        /// <param name="error">Current error (Setpoint - Measured)</param>
        /// <param name="dt">Time step in seconds since last update</param>
        /// <returns>Saturated control output clamped to [MinOutput, MaxOutput]</returns>
        public float Calculate(float error, float dt)
        {
            if (dt <= 0f)
            {
                return lastOutput;
            }

            // 1. Deadband suppression: eliminates jitter and mechanical hunting around zero error
            float effectiveError = error;
            if (deadband > 0f && Mathf.Abs(effectiveError) <= deadband)
            {
                effectiveError = 0f;
            }

            // 2. Proportional Term: P = Kp * e
            proportionalTerm = kp * effectiveError;

            // 3. Integral Term with Anti-Windup
            // Conditional integration: if output is already saturated in the direction of error,
            // or if integration is explicitly frozen (e.g. during COASTING), suspend integration.
            bool suspendIntegration = freezeIntegration;
            if (enableAntiWindup && isSaturated && !suspendIntegration)
            {
                if (lastOutput >= maxOutput && effectiveError > 0f)
                {
                    suspendIntegration = true;
                }
                else if (lastOutput <= minOutput && effectiveError < 0f)
                {
                    suspendIntegration = true;
                }
            }

            if (!suspendIntegration && ki > 0f)
            {
                integral += effectiveError * dt;

                // Clamp accumulator if explicit integral limit is enabled
                if (enableIntegralLimit && maxIntegral > 0f)
                {
                    integral = Mathf.Clamp(integral, -maxIntegral, maxIntegral);
                }
            }
            integralTerm = ki * integral;

            // 4. Derivative Term with First-Order Low-Pass Filter
            // D_raw = (e - e_prev) / dt
            // Low-pass filter: D_filtered = D_prev + alpha * (D_raw - D_prev), where alpha = dt / (tau + dt)
            if (isFirstUpdate)
            {
                filteredDerivative = 0f;
                isFirstUpdate = false;
            }
            else
            {
                float rawDerivative = (effectiveError - lastError) / dt;
                if (derivativeFilterTimeConstant > 0f)
                {
                    float alpha = dt / (derivativeFilterTimeConstant + dt);
                    filteredDerivative = Mathf.Lerp(filteredDerivative, rawDerivative, alpha);
                }
                else
                {
                    filteredDerivative = rawDerivative;
                }
            }
            derivativeTerm = kd * filteredDerivative;
            lastError = effectiveError;

            // 5. Total Output & Saturation Clamping
            float rawOutput = proportionalTerm + integralTerm + derivativeTerm;
            float clampedOutput = Mathf.Clamp(rawOutput, minOutput, maxOutput);

            isSaturated = (rawOutput != clampedOutput);
            lastOutput = clampedOutput;

            return clampedOutput;
        }

        /// <summary>
        /// Computes PID control output given setpoint and measured values directly.
        /// </summary>
        /// <param name="setpoint">Target setpoint</param>
        /// <param name="measured">Current measured process variable</param>
        /// <param name="dt">Time step in seconds</param>
        /// <returns>Saturated control output</returns>
        public float Calculate(float setpoint, float measured, float dt)
        {
            return Calculate(setpoint - measured, dt);
        }
    }
}
