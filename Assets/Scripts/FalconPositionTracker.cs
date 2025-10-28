using UnityEngine;

/// <summary>
/// Makes the Falcon grip follow the position of a Unity GameObject using PID control.
/// Attach this script to the FalconController GameObject and assign a target object to track.
/// </summary>
public class FalconPositionTracker : MonoBehaviour
{
    [Header("Target Tracking")]
    [Tooltip("The GameObject that the Falcon grip should follow")]
    public GameObject targetObject;

    [Tooltip("Enable position tracking mode")]
    public bool enableTracking = false;

    [Tooltip("Use fixed position instead of following GameObject (for testing)")]
    public bool useFixedPosition = false;

    [Tooltip("Fixed position to track (when useFixedPosition is enabled)")]
    public Vector3 fixedPosition = new Vector3(0f, 0.04f, 0f);

    [Tooltip("Scale factor for position (Unity units to Falcon meters)")]
    public float positionScale = 1.0f;

    [Tooltip("Z-axis inversion (Falcon coordinate system adjustment)")]
    public bool invertZ = true;

    [Header("Unity-side Smoothing")]
    [Tooltip("Smooth target position changes on Unity side (0.0 = no smoothing, 0.5 = heavy smoothing)")]
    [Range(0f, 0.9f)]
    public float unitySmoothingFactor = 0.5f;  // Heavy smoothing to prevent jerky GameObject movement

    [Header("PID Control Parameters")]
    [Tooltip("Proportional gain (lower = softer, higher = stiffer)")]
    [Range(1f, 200f)]
    public float pidKp = 100f;  // Same as mouse cursor version

    [Tooltip("Integral gain (lower = less windup)")]
    [Range(0f, 20f)]
    public float pidKi = 5f;  // Same as mouse cursor version

    [Tooltip("Derivative gain (higher = more damping)")]
    [Range(0f, 30f)]
    public float pidKd = 8f;  // Same as mouse cursor version

    [Tooltip("Low-pass filter coefficient for C++ side interpolation")]
    [Range(0.01f, 1f)]
    public float filterAlpha = 0.1f;  // Same as mouse cursor version

    [Tooltip("Maximum force per axis in Newtons")]
    [Range(0.5f, 9.0f)]
    public float maxForce = 2.0f;  // Same as mouse cursor version

    [Header("Debug")]
    [Tooltip("Show debug information")]
    public bool showDebug = false;

    private bool wasTrackingEnabled = false;
    private float lastKp, lastKi, lastKd, lastFilterAlpha, lastMaxForce;
    private Vector3 smoothedTargetPos = Vector3.zero;  // Unity-side smoothed position
    private bool smoothedInitialized = false;

    void Start()
    {
        // Initialize PID parameters on start
        UpdatePIDParameters();
    }

    void Update()
    {
        // Check if PID parameters changed
        if (pidKp != lastKp || pidKi != lastKi || pidKd != lastKd ||
            filterAlpha != lastFilterAlpha || maxForce != lastMaxForce)
        {
            UpdatePIDParameters();
        }

        // Check if tracking mode changed
        if (enableTracking != wasTrackingEnabled)
        {
            FalconBridge.EnablePositionControl(enableTracking);
            wasTrackingEnabled = enableTracking;

            if (showDebug)
            {
                Debug.Log($"Falcon position tracking: {(enableTracking ? "ENABLED" : "DISABLED")}");
            }
        }
    }

    void FixedUpdate()
    {
        // If tracking is enabled and we have a target, update the target position
        // Using FixedUpdate for consistent timing (50Hz instead of variable framerate)
        if (enableTracking && FalconBridge.IsCalibrated())
        {
            // Get raw target position
            Vector3 rawTargetPos;

            if (useFixedPosition)
            {
                // Use fixed position (for testing - no smoothing needed)
                rawTargetPos = fixedPosition;
            }
            else if (targetObject != null)
            {
                // Get position from GameObject
                rawTargetPos = targetObject.transform.position * positionScale;
            }
            else
            {
                // No target - skip this update
                return;
            }

            // Apply Z-axis inversion if needed (Falcon coordinate system difference)
            if (invertZ)
            {
                rawTargetPos.z = -rawTargetPos.z;
            }

            // Initialize smoothed position on first run
            if (!smoothedInitialized)
            {
                smoothedTargetPos = rawTargetPos;
                smoothedInitialized = true;
            }

            // Apply Unity-side smoothing (Lerp)
            // smoothedPos = smoothedPos + (1 - smoothingFactor) * (rawPos - smoothedPos)
            // When smoothingFactor = 0: no smoothing (instant follow)
            // When smoothingFactor = 0.5: heavy smoothing
            smoothedTargetPos = Vector3.Lerp(smoothedTargetPos, rawTargetPos, 1.0f - unitySmoothingFactor);

            // Send smoothed target position to Falcon (C++ will further interpolate at 1kHz)
            FalconBridge.SetTargetPosition(smoothedTargetPos);

            // Debug visualization
            if (showDebug && Time.frameCount % 60 == 0) // Log once per second at 60fps
            {
                Debug.Log($"Target Object: {targetObject.name} at {targetObject.transform.position:F3}");
                Debug.Log($"Raw Target: {rawTargetPos:F3}, Smoothed: {smoothedTargetPos:F3}");
            }
        }
    }

    void OnDisable()
    {
        // Disable position control when this component is disabled
        if (enableTracking)
        {
            FalconBridge.EnablePositionControl(false);
            enableTracking = false;
            wasTrackingEnabled = false;
        }

        // Reset smoothing state
        smoothedInitialized = false;
    }

    void OnDrawGizmos()
    {
        if (targetObject != null && enableTracking)
        {
            // Draw a line from Falcon current position to target
            Vector3 falconPos;
            if (FalconBridge.GetToolPosition(out falconPos))
            {
                Vector3 targetPos = targetObject.transform.position * positionScale;
                if (invertZ)
                {
                    targetPos.z = -targetPos.z;
                }

                Gizmos.color = Color.cyan;
                Gizmos.DrawLine(falconPos, targetPos);

                // Draw target position indicator
                Gizmos.color = Color.yellow;
                Gizmos.DrawWireSphere(targetPos, 0.01f);
            }
        }
    }

    private void UpdatePIDParameters()
    {
        FalconBridge.SetPIDParameters(pidKp, pidKi, pidKd, filterAlpha, maxForce);

        lastKp = pidKp;
        lastKi = pidKi;
        lastKd = pidKd;
        lastFilterAlpha = filterAlpha;
        lastMaxForce = maxForce;

        if (showDebug)
        {
            Debug.Log($"PID parameters updated: Kp={pidKp}, Ki={pidKi}, Kd={pidKd}, alpha={filterAlpha}, maxForce={maxForce}");
        }
    }
}
