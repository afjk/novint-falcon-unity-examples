using UnityEngine;

/// <summary>
/// Example controller for the Novint Falcon haptic device.
/// This script demonstrates basic usage of the FalconBridge plugin.
/// </summary>
public class FalconController : MonoBehaviour
{
    [Header("Falcon Settings")]
    [Tooltip("Enable debug logging")]
    public bool debugLog = true;

    [Tooltip("Visualize the Falcon tool position")]
    public GameObject toolCursor;

    [Tooltip("Scale factor for Falcon position (meters to Unity units)")]
    public float positionScale = 1.0f;

    [Header("Haptic Feedback")]
    [Tooltip("Enable haptic feedback")]
    public bool enableHaptics = true;

    [Tooltip("Test plane Y position for haptic feedback")]
    public float testPlaneY = 0.0f;

    private bool isInitialized = false;
    private Vector3 currentPosition;

    void Start()
    {
        // Initialize the Falcon device
        try
        {
            if (FalconBridge.InitFalcon())
            {
                isInitialized = true;
                if (debugLog)
                {
                    Debug.Log("Falcon device initialized successfully");
                    Debug.Log("Starting calibration - move Falcon outward then straight inward");
                }
                // Start calibration automatically
                FalconBridge.StartCalibration();
            }
            else
            {
                Debug.LogError("Failed to initialize Falcon device. Make sure it's connected and drivers are installed.");
            }
        }
        catch (System.DllNotFoundException)
        {
            Debug.LogError("FalconBridge plugin not found. Please build the native plugin first. See Assets/Plugins/FalconBridge/BUILD.md for instructions.");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Error initializing Falcon device: {e.Message}");
        }

        // Create tool cursor if not assigned
        if (toolCursor == null)
        {
            toolCursor = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            toolCursor.transform.localScale = Vector3.one * 0.05f;
            toolCursor.name = "FalconToolCursor";

            // Make it visually distinct
            var renderer = toolCursor.GetComponent<Renderer>();
            if (renderer != null)
            {
                renderer.material.color = Color.red;
            }
        }
    }

    void Update()
    {
        if (!isInitialized)
            return;

        // Check calibration status
        bool isCalibrated = FalconBridge.IsCalibrated();

        // Get the current tool position
        if (FalconBridge.GetToolPosition(out currentPosition))
        {
            // Update cursor position
            if (toolCursor != null)
            {
                toolCursor.transform.position = currentPosition * positionScale;

                // Change cursor color based on calibration status
                var renderer = toolCursor.GetComponent<Renderer>();
                if (renderer != null)
                {
                    renderer.material.color = isCalibrated ? Color.green : Color.red;
                }
            }

            // Apply haptic feedback only if calibrated
            if (enableHaptics && isCalibrated)
            {
                UpdateHapticFeedback();
            }

            // Debug output
            if (debugLog && Time.frameCount % 60 == 0) // Log once per second at 60 fps
            {
                string calStatus = isCalibrated ? "CALIBRATED" : "NOT CALIBRATED";
                Debug.Log($"[{calStatus}] Falcon Position: {currentPosition:F4}");
            }
        }
        else
        {
            if (debugLog)
            {
                Debug.LogWarning("Failed to get Falcon position");
            }
        }
    }

    void UpdateHapticFeedback()
    {
        // Simple plane collision example
        // Apply force when the tool goes below the test plane
        float penetrationDepth = testPlaneY - currentPosition.y;

        if (penetrationDepth > 0)
        {
            // Tool is below the plane - apply upward force
            Vector3 normal = Vector3.up;
            FalconBridge.SetContact(normal, penetrationDepth);

            if (debugLog && Time.frameCount % 60 == 0)
            {
                Debug.Log($"Contact detected - Depth: {penetrationDepth:F4}m");
            }
        }
        else
        {
            // No contact - clear forces
            FalconBridge.ClearContact();
        }
    }

    void OnDestroy()
    {
        // Clean shutdown
        if (isInitialized)
        {
            FalconBridge.ClearContact();
            FalconBridge.ShutdownFalcon();
            if (debugLog)
            {
                Debug.Log("Falcon device shutdown");
            }
        }
    }

    void OnApplicationQuit()
    {
        // Ensure clean shutdown on application exit
        if (isInitialized)
        {
            FalconBridge.ClearContact();
            FalconBridge.ShutdownFalcon();
        }
    }

    // Visualize the test plane in the Scene view
    void OnDrawGizmos()
    {
        Gizmos.color = new Color(0, 1, 0, 0.3f);
        Gizmos.DrawCube(new Vector3(0, testPlaneY, 0), new Vector3(0.2f, 0.001f, 0.2f));

        Gizmos.color = Color.green;
        Gizmos.DrawWireCube(new Vector3(0, testPlaneY, 0), new Vector3(0.2f, 0.001f, 0.2f));
    }
}
