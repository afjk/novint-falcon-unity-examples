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

    [Tooltip("Scale factor for position (Unity units to Falcon meters)")]
    public float positionScale = 1.0f;

    [Tooltip("Z-axis inversion (Falcon coordinate system adjustment)")]
    public bool invertZ = true;

    [Header("Debug")]
    [Tooltip("Show debug information")]
    public bool showDebug = false;

    private bool wasTrackingEnabled = false;

    void Update()
    {
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

        // If tracking is enabled and we have a target, update the target position
        if (enableTracking && targetObject != null && FalconBridge.IsCalibrated())
        {
            // Get target position in Falcon coordinate system
            Vector3 targetPos = targetObject.transform.position * positionScale;

            // Apply Z-axis inversion if needed (Falcon coordinate system difference)
            if (invertZ)
            {
                targetPos.z = -targetPos.z;
            }

            // Send target position to Falcon
            FalconBridge.SetTargetPosition(targetPos);

            // Debug visualization
            if (showDebug && Time.frameCount % 60 == 0) // Log once per second at 60fps
            {
                Debug.Log($"Target Object: {targetObject.name} at {targetObject.transform.position:F3}");
                Debug.Log($"Falcon Target: {targetPos:F3}");
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
}
