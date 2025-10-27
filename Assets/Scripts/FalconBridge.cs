using System;
using System.Runtime.InteropServices;
using UnityEngine;

/// <summary>
/// Unity C# wrapper for the FalconBridge native plugin.
/// Provides access to Novint Falcon haptic device functionality.
/// </summary>
public static class FalconBridge
{
    // Platform-specific library name
#if UNITY_STANDALONE_OSX || UNITY_EDITOR_OSX
    private const string LIBRARY_NAME = "FalconBridge";
#elif UNITY_STANDALONE_WIN || UNITY_EDITOR_WIN
    private const string LIBRARY_NAME = "FalconBridge";
#elif UNITY_STANDALONE_LINUX || UNITY_EDITOR_LINUX
    private const string LIBRARY_NAME = "FalconBridge";
#else
    private const string LIBRARY_NAME = "FalconBridge";
#endif

    /// <summary>
    /// Initialize the Falcon device and start the haptic feedback loop.
    /// </summary>
    /// <returns>True if initialization successful, false otherwise</returns>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool InitFalcon();

    /// <summary>
    /// Shutdown the Falcon device and stop the haptic feedback loop.
    /// </summary>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void ShutdownFalcon();

    /// <summary>
    /// Get the current position of the Falcon tool (end effector).
    /// </summary>
    /// <param name="x">X position in meters</param>
    /// <param name="y">Y position in meters</param>
    /// <param name="z">Z position in meters</param>
    /// <returns>True if position retrieved successfully, false otherwise</returns>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool GetToolPose(out float x, out float y, out float z);

    /// <summary>
    /// Set contact parameters for haptic feedback.
    /// Force is calculated as: F = -k * depth * normal (k = 3000 N/m, max = 3N)
    /// </summary>
    /// <param name="nx">Normal vector X component</param>
    /// <param name="ny">Normal vector Y component</param>
    /// <param name="nz">Normal vector Z component</param>
    /// <param name="depth">Penetration depth in meters</param>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetContact(float nx, float ny, float nz, float depth);

    /// <summary>
    /// Check if the Falcon device is calibrated (homed).
    /// </summary>
    /// <returns>True if calibrated, false otherwise</returns>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool IsCalibrated();

    /// <summary>
    /// Start the calibration process. Move the Falcon fully outward then straight inward.
    /// The device LED will turn red during calibration and green when complete.
    /// </summary>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void StartCalibration();

    /// <summary>
    /// Convenience method to get tool position as a Vector3.
    /// </summary>
    /// <param name="position">Output position vector</param>
    /// <returns>True if position retrieved successfully, false otherwise</returns>
    public static bool GetToolPosition(out Vector3 position)
    {
        float x, y, z;
        bool success = GetToolPose(out x, out y, out z);
        position = new Vector3(x, y, z);
        return success;
    }

    /// <summary>
    /// Convenience method to set contact using Vector3 normal.
    /// </summary>
    /// <param name="normal">Surface normal vector</param>
    /// <param name="depth">Penetration depth in meters</param>
    public static void SetContact(Vector3 normal, float depth)
    {
        SetContact(normal.x, normal.y, normal.z, depth);
    }

    /// <summary>
    /// Clear contact (set depth to zero).
    /// </summary>
    public static void ClearContact()
    {
        SetContact(0f, 0f, 0f, 0f);
    }
}
