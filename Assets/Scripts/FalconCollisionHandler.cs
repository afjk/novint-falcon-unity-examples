using UnityEngine;

/// <summary>
/// Advanced Falcon haptic controller with 3D object collision detection.
/// Provides haptic feedback when the Falcon tool intersects with Unity colliders.
/// </summary>
[RequireComponent(typeof(SphereCollider))]
public class FalconCollisionHandler : MonoBehaviour
{
    [Header("Falcon Settings")]
    [Tooltip("Enable debug logging")]
    public bool debugLog = false;

    [Tooltip("Scale factor for Falcon position (meters to Unity units)")]
    public float positionScale = 1.0f;

    [Tooltip("Radius of the tool collision sphere")]
    public float toolRadius = 0.02f;

    [Header("Haptic Parameters")]
    [Tooltip("Custom spring stiffness multiplier (applied on top of native k=3000)")]
    [Range(0.1f, 2.0f)]
    public float stiffnessMultiplier = 1.0f;

    [Tooltip("Minimum penetration depth to trigger haptics (meters)")]
    public float minDepthThreshold = 0.0001f;

    [Header("Visual Feedback")]
    [Tooltip("Cursor object to visualize tool position")]
    public GameObject toolCursor;

    [Tooltip("Change cursor color on contact")]
    public bool visualizeContact = true;

    [Tooltip("Contact color")]
    public Color contactColor = Color.red;

    [Tooltip("No contact color")]
    public Color noContactColor = Color.green;

    private bool isInitialized = false;
    private Vector3 currentPosition;
    private SphereCollider toolCollider;
    private Renderer cursorRenderer;
    private bool inContact = false;

    void Start()
    {
        // Initialize the Falcon device
        if (FalconBridge.InitFalcon())
        {
            isInitialized = true;
            Debug.Log("Falcon device initialized successfully");
        }
        else
        {
            Debug.LogError("Failed to initialize Falcon device");
            enabled = false;
            return;
        }

        // Setup tool collider
        toolCollider = GetComponent<SphereCollider>();
        toolCollider.radius = toolRadius;
        toolCollider.isTrigger = true;

        // Setup rigidbody for collision detection
        var rb = gameObject.AddComponent<Rigidbody>();
        rb.isKinematic = true;
        rb.useGravity = false;

        // Create or setup tool cursor
        if (toolCursor == null)
        {
            toolCursor = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            toolCursor.transform.localScale = Vector3.one * toolRadius * 2;
            toolCursor.name = "FalconToolCursor";

            // Remove collider from cursor (we use the one on this GameObject)
            Destroy(toolCursor.GetComponent<Collider>());
        }

        cursorRenderer = toolCursor.GetComponent<Renderer>();
        if (cursorRenderer != null && visualizeContact)
        {
            cursorRenderer.material.color = noContactColor;
        }
    }

    void Update()
    {
        if (!isInitialized)
            return;

        // Get the current tool position from Falcon
        if (FalconBridge.GetToolPosition(out currentPosition))
        {
            // Update this GameObject's position (for collision detection)
            transform.position = currentPosition * positionScale;

            // Update cursor position
            if (toolCursor != null)
            {
                toolCursor.transform.position = transform.position;
            }
        }

        // Reset contact flag (will be set by collision detection)
        if (inContact)
        {
            inContact = false;
        }
        else
        {
            // No collision this frame - clear forces
            FalconBridge.ClearContact();
            UpdateCursorColor(false);
        }
    }

    void OnTriggerStay(Collider other)
    {
        // Calculate penetration depth and normal
        Vector3 closestPoint = other.ClosestPoint(transform.position);
        Vector3 penetrationVector = transform.position - closestPoint;
        float penetrationDepth = penetrationVector.magnitude;

        // Only apply haptics if penetration is above threshold
        if (penetrationDepth > minDepthThreshold)
        {
            // Calculate surface normal (pointing away from the object)
            Vector3 normal = penetrationVector.normalized;

            // Apply depth multiplier if needed
            float adjustedDepth = penetrationDepth * stiffnessMultiplier;

            // Set contact force
            FalconBridge.SetContact(normal, adjustedDepth);

            inContact = true;
            UpdateCursorColor(true);

            if (debugLog && Time.frameCount % 30 == 0)
            {
                Debug.Log($"Contact with {other.gameObject.name} - Depth: {penetrationDepth:F4}m, Normal: {normal:F3}");
            }
        }
    }

    void OnTriggerExit(Collider other)
    {
        // Clear contact when exiting trigger
        if (!inContact)
        {
            FalconBridge.ClearContact();
            UpdateCursorColor(false);
        }
    }

    void UpdateCursorColor(bool contact)
    {
        if (cursorRenderer != null && visualizeContact)
        {
            cursorRenderer.material.color = contact ? contactColor : noContactColor;
        }
    }

    void OnDestroy()
    {
        Shutdown();
    }

    void OnApplicationQuit()
    {
        Shutdown();
    }

    void Shutdown()
    {
        if (isInitialized)
        {
            FalconBridge.ClearContact();
            FalconBridge.ShutdownFalcon();
            isInitialized = false;
            Debug.Log("Falcon device shutdown");
        }
    }

    // Visualize the tool collision sphere in the Scene view
    void OnDrawGizmos()
    {
        Gizmos.color = inContact ? contactColor : noContactColor;
        Gizmos.DrawWireSphere(transform.position, toolRadius);
    }

    void OnDrawGizmosSelected()
    {
        // Draw a more visible sphere when selected
        Gizmos.color = new Color(1, 1, 0, 0.3f);
        Gizmos.DrawSphere(transform.position, toolRadius);
    }
}
