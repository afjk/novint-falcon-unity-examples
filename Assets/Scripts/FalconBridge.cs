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
    /// Set the native spring constant (N/m) used for contact forces.
    /// </summary>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetContactStiffness(float stiffness);

    /// <summary>
    /// Set the native damping gain (N·s/m) applied along the contact normal.
    /// </summary>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetContactDamping(float damping);

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
    /// Set target position for position control mode.
    /// The Falcon grip will be pulled towards this position using PID control.
    /// </summary>
    /// <param name="x">Target X position in meters</param>
    /// <param name="y">Target Y position in meters</param>
    /// <param name="z">Target Z position in meters</param>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetTargetPosition(float x, float y, float z);

    /// <summary>
    /// Enable or disable position control mode.
    /// When enabled, the grip tracks the target position set by SetTargetPosition.
    /// When disabled, normal contact-based haptic feedback is used.
    /// </summary>
    /// <param name="enable">True to enable position control, false for contact mode</param>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void EnablePositionControl(bool enable);

    /// <summary>
    /// Set PID control parameters for position control.
    /// Adjust these to tune the tracking behavior and reduce vibration.
    /// </summary>
    /// <param name="kp">Proportional gain (lower = softer, higher = stiffer)</param>
    /// <param name="ki">Integral gain (lower = less windup)</param>
    /// <param name="kd">Derivative gain (higher = more damping, reduces oscillation)</param>
    /// <param name="filterAlpha">Low-pass filter coefficient (0.01-1.0, lower = smoother)</param>
    /// <param name="maxForce">Maximum force per axis in Newtons (0.5-3.0)</param>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetPIDParameters(float kp, float ki, float kd, float filterAlpha, float maxForce);

    /// <summary>
    /// Set LED status on the Falcon device.
    /// Multiple LEDs can be enabled by OR-ing the values together.
    /// Note: During calibration, LED status is overridden (red during calibration, green when complete).
    /// </summary>
    /// <param name="ledMask">LED mask (0x1=BLUE, 0x2=GREEN, 0x4=AMBER, 0x8=RED)</param>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern void SetLEDStatus(int ledMask);

    /// <summary>
    /// Get the current state of the Falcon's buttons.
    /// </summary>
    /// <param name="button1">Button 1 state (PLUS/Right button)</param>
    /// <param name="button2">Button 2 state (FORWARD button)</param>
    /// <param name="button3">Button 3 state (CENTER button)</param>
    /// <param name="button4">Button 4 state (MINUS/Left button)</param>
    /// <returns>True if button states retrieved successfully, false otherwise</returns>
    [DllImport(LIBRARY_NAME, CallingConvention = CallingConvention.Cdecl)]
    public static extern bool GetButtonStates(out bool button1, out bool button2, out bool button3, out bool button4);

    /// <summary>
    /// LED color constants for use with SetLEDStatus.
    /// </summary>
    public static class LED
    {
        public const int BLUE = 0x1;
        public const int GREEN = 0x2;
        public const int AMBER = 0x4;
        public const int RED = 0x8;
        public const int OFF = 0x0;
    }

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

    /// <summary>
    /// Convenience method to set target position using Vector3.
    /// </summary>
    /// <param name="targetPosition">Target position vector in meters</param>
    public static void SetTargetPosition(Vector3 targetPosition)
    {
        SetTargetPosition(targetPosition.x, targetPosition.y, targetPosition.z);
    }
}
