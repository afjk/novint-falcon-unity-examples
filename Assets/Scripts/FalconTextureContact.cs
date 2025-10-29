using UnityEngine;

/// <summary>
/// Generates a rough "sandpaper" feel when the Falcon tool touches MeshColliders.
/// Attach alongside FalconController on a GameObject that manages the Falcon device.
/// </summary>
[RequireComponent(typeof(FalconController))]
public class FalconTextureContact : MonoBehaviour
{
    [Header("Contact Probe")]
    [Tooltip("Probe radius in Unity units for detecting surface contact")]
    [Range(0.002f, 0.05f)]
    public float probeRadius = 0.01f;

    [Tooltip("Physics layers that provide rough texture haptics")]
    public LayerMask texturedLayers = ~0;

    [Tooltip("Scale converting Falcon meters to Unity units (match FalconController)")]
    public float positionScale = 1.0f;

    [Tooltip("Invert Z axis when mapping Falcon -> Unity coordinates")]
    public bool invertZ = true;

    [Header("Texture Parameters")]
    [Tooltip("Blend factor for tangential jitter applied to the surface normal")]
    [Range(0f, 0.4f)]
    public float normalJitterStrength = 0.08f;

    [Tooltip("Temporal frequency (Hz) for normal jitter modulation")]
    [Range(10f, 200f)]
    public float normalJitterFrequency = 80f;

    [Tooltip("Spatial frequency for the rough pattern (higher = denser bumps)")]
    [Range(3f, 80f)]
    public float spatialFrequency = 18f;

    [Tooltip("Depth modulation strength (adds pulsing in/out along the normal)")]
    [Range(0f, 0.4f)]
    public float depthNoiseStrength = 0.15f;

    [Tooltip("Temporal frequency (Hz) for depth modulation")]
    [Range(10f, 180f)]
    public float depthNoiseFrequency = 120f;

    [Tooltip("Amplitude of high-frequency pulses along the normal (meters)")]
    [Range(0f, 0.0004f)]
    public float normalPulseAmplitude = 0.0f;

    [Tooltip("Pulse frequency (Hz) for the normal vibration")]
    [Range(20f, 220f)]
    public float normalPulseFrequency = 120f;

    [Header("Stabilisation")]
    [Tooltip("Clamp for penetration depth passed to the Falcon (meters)")]
    [Range(0.0005f, 0.006f)]
    public float maxDepthMeters = 0.003f;

    [Tooltip("Time constant in seconds for normal smoothing (0 = no smoothing)")]
    [Range(0f, 0.2f)]
    public float normalSmoothingTime = 0.03f;

    [Tooltip("Time constant in seconds for depth smoothing (0 = no smoothing)")]
    [Range(0f, 0.2f)]
   public float depthSmoothingTime = 0.04f;

    [Header("Native Force Response")]
    [Tooltip("Spring stiffness used by the native 1 kHz loop (N/m)")]
    [Range(200f, 6000f)]
    public float nativeSpring = 3000f;

    [Tooltip("Damping gain used by the native 1 kHz loop (N·s/m)")]
    [Range(0f, 6f)]
    public float nativeDamping = 0.6f;

    [Header("Debug")]
    public bool showDebugGizmos = false;

    private SphereCollider probeCollider;
    private Transform probeTransform;
    private readonly Collider[] overlapCache = new Collider[16];
    private Vector3 debugContactPoint;
    private Vector3 debugContactNormal;
    private bool hadContactLastFrame;
    private Vector3 smoothedNormal = Vector3.up;
    private float smoothedDepth;
    private bool hasSmoothedContact;

    void Awake()
    {
        GameObject probeGO = new GameObject("FalconTextureProbe");
        probeGO.hideFlags = HideFlags.HideAndDontSave;
        int ignoreLayer = LayerMask.NameToLayer("Ignore Raycast");
        if (ignoreLayer >= 0)
        {
            probeGO.layer = ignoreLayer;
        }
        probeGO.transform.SetParent(transform, false);

        probeCollider = probeGO.AddComponent<SphereCollider>();
        probeCollider.isTrigger = true;
        probeCollider.radius = probeRadius;
        probeTransform = probeGO.transform;
    }

    void OnEnable()
    {
        ApplyNativeResponseSettings();
    }

    void OnValidate()
    {
        if (probeCollider != null)
        {
            probeCollider.radius = probeRadius;
        }

        if (Application.isPlaying)
        {
            ApplyNativeResponseSettings();
        }
    }

    void OnDestroy()
    {
        if (probeTransform != null)
        {
            Destroy(probeTransform.gameObject);
        }
    }

    void FixedUpdate()
    {
        if (!FalconBridge.IsCalibrated())
        {
            ClearContactIfNeeded();
            return;
        }

        if (!FalconBridge.GetToolPosition(out Vector3 toolMeters))
        {
            ClearContactIfNeeded();
            return;
        }

        Vector3 toolUnity = new Vector3(toolMeters.x, toolMeters.y, invertZ ? -toolMeters.z : toolMeters.z) * positionScale;
        probeTransform.position = toolUnity;
        probeCollider.radius = probeRadius;

        int hitCount = Physics.OverlapSphereNonAlloc(toolUnity, probeRadius, overlapCache, texturedLayers, QueryTriggerInteraction.Ignore);
        if (hitCount == 0)
        {
            ClearContactIfNeeded();
            return;
        }

        bool hasContact = false;
        Vector3 bestNormal = Vector3.zero;
        float bestDepth = 0f;
        Vector3 bestPoint = Vector3.zero;
        Collider bestCollider = null;

        for (int i = 0; i < hitCount; i++)
        {
            Collider other = overlapCache[i];
            if (other == null || other == probeCollider)
                continue;

            if (Physics.ComputePenetration(
                    probeCollider,
                    toolUnity,
                    probeTransform.rotation,
                    other,
                    other.transform.position,
                    other.transform.rotation,
                    out Vector3 separationNormal,
                    out float separationDistance))
            {
                if (separationDistance > bestDepth)
                {
                    Vector3 outwardNormal = separationNormal.normalized;
                    bestDepth = separationDistance;
                    bestNormal = outwardNormal;
                    float contactOffset = Mathf.Clamp(probeRadius - separationDistance, 0f, probeRadius);
                    bestPoint = toolUnity - outwardNormal * contactOffset;
                    bestCollider = other;
                    hasContact = true;
                }
            }
        }

        if (!hasContact)
        {
            ClearContactIfNeeded();
            return;
        }

        ApplyRoughContact(toolUnity, bestPoint, bestNormal.normalized, bestDepth);
    }

    private void ApplyRoughContact(Vector3 toolUnity, Vector3 contactPoint, Vector3 surfaceNormal, float penetrationDepth)
    {
        float time = Time.time;
        float scaleSafe = Mathf.Max(positionScale, 1e-5f);

        Vector3 tangent = Vector3.Cross(surfaceNormal, Vector3.up);
        if (tangent.sqrMagnitude < 1e-6f)
        {
            tangent = Vector3.Cross(surfaceNormal, Vector3.right);
        }
        tangent.Normalize();
        Vector3 bitangent = Vector3.Cross(surfaceNormal, tangent);

        float jitterParam = normalJitterStrength;
        Vector3 jitter = Vector3.zero;
        if (jitterParam > 0f)
        {
            float jitterPhase = time * normalJitterFrequency;
            float noise1 = Mathf.PerlinNoise(contactPoint.x * spatialFrequency + jitterPhase, contactPoint.y * spatialFrequency) * 2f - 1f;
            float noise2 = Mathf.PerlinNoise(contactPoint.z * spatialFrequency, contactPoint.x * spatialFrequency + jitterPhase * 0.79f) * 2f - 1f;
            float depthInfluence = Mathf.Clamp01(penetrationDepth / Mathf.Max(probeRadius * 0.6f, 1e-4f));
            jitter = (tangent * noise1 + bitangent * noise2) * (jitterParam * depthInfluence);
        }

        Vector3 texturedNormal = (surfaceNormal + jitter).normalized;

        float depthMeters = Mathf.Max(0f, penetrationDepth / scaleSafe);
        if (depthMeters < 0f)
        {
            depthMeters = 0f;
        }

        if (depthNoiseStrength > 0f)
        {
            float depthPhase = time * depthNoiseFrequency;
            float depthNoise = Mathf.PerlinNoise(contactPoint.x * spatialFrequency + depthPhase, contactPoint.z * spatialFrequency + 37.1f) * 2f - 1f;
            float depthScale = 1f + depthNoiseStrength * depthNoise;
            depthMeters *= Mathf.Clamp(depthScale, 0.4f, 1.6f);
        }

        if (normalPulseAmplitude > 0f)
        {
            float pulse = Mathf.Sin(time * normalPulseFrequency * Mathf.PI * 2f) * normalPulseAmplitude;
            depthMeters = Mathf.Max(0f, depthMeters + pulse);
        }

        depthMeters = Mathf.Clamp(depthMeters, 0f, maxDepthMeters);

        float dt = Time.deltaTime > 0f ? Time.deltaTime : Time.fixedDeltaTime;
        float normalBlend = GetBlendFactor(dt, normalSmoothingTime);
        float depthBlend = GetBlendFactor(dt, depthSmoothingTime);
        if (!hasSmoothedContact)
        {
            smoothedNormal = texturedNormal;
            smoothedDepth = depthMeters;
            hasSmoothedContact = true;
        }
        else
        {
            smoothedNormal = Vector3.Slerp(smoothedNormal, texturedNormal, normalBlend);
            smoothedNormal.Normalize();
            smoothedDepth = Mathf.Lerp(smoothedDepth, depthMeters, depthBlend);
        }

        if (smoothedDepth <= 0f)
        {
            ClearContactIfNeeded();
            return;
        }

        Vector3 outputNormal = -smoothedNormal;
        Vector3 deviceNormal = outputNormal;
        if (invertZ)
        {
            deviceNormal.z = -deviceNormal.z;
        }
        FalconBridge.SetContact(deviceNormal, smoothedDepth);
        hadContactLastFrame = true;

        if (showDebugGizmos)
        {
            debugContactPoint = contactPoint;
            debugContactNormal = new Vector3(-deviceNormal.x, -deviceNormal.y, deviceNormal.z);
        }
    }

    private void ClearContactIfNeeded()
    {
        if (hadContactLastFrame)
        {
            FalconBridge.ClearContact();
            hadContactLastFrame = false;
        }
        hasSmoothedContact = false;
        smoothedDepth = 0f;
    }

    private float GetBlendFactor(float deltaTime, float timeConstant)
    {
        if (timeConstant <= 0f)
            return 1f;
        float safeTime = Mathf.Max(timeConstant, 1e-4f);
        return 1f - Mathf.Exp(-deltaTime / safeTime);
    }

    private void ApplyNativeResponseSettings()
    {
        FalconBridge.SetContactStiffness(Mathf.Clamp(nativeSpring, 200f, 6000f));
        FalconBridge.SetContactDamping(Mathf.Clamp(nativeDamping, 0f, 6f));
    }

    void OnDisable()
    {
        ClearContactIfNeeded();
    }

    void OnDrawGizmos()
    {
        if (!showDebugGizmos)
            return;

        if (probeTransform != null)
        {
            Gizmos.color = Color.cyan;
            Gizmos.DrawWireSphere(probeTransform.position, probeRadius);
        }

        if (hadContactLastFrame)
        {
            Gizmos.color = Color.magenta;
            Gizmos.DrawSphere(debugContactPoint, probeRadius * 0.25f);
            Gizmos.DrawLine(debugContactPoint, debugContactPoint + debugContactNormal * 0.03f);
        }
    }
}
