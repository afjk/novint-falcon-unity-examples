#ifndef FALCONBRIDGE_H
#define FALCONBRIDGE_H

// Platform-specific export macro
#ifdef _WIN32
    #define FALCON_API extern "C" __declspec(dllexport)
#else
    #define FALCON_API extern "C" __attribute__((visibility("default")))
#endif

/**
 * Initialize the Falcon device and start the haptic feedback loop.
 * @return true if initialization successful, false otherwise
 */
FALCON_API bool InitFalcon();

/**
 * Shutdown the Falcon device and stop the haptic feedback loop.
 */
FALCON_API void ShutdownFalcon();

/**
 * Get the current position of the Falcon tool (end effector).
 * @param x Pointer to store X position (meters)
 * @param y Pointer to store Y position (meters)
 * @param z Pointer to store Z position (meters)
 * @return true if position retrieved successfully, false otherwise
 */
FALCON_API bool GetToolPose(float* x, float* y, float* z);

/**
 * Set contact parameters for haptic feedback.
 * @param nx Normal vector X component
 * @param ny Normal vector Y component
 * @param nz Normal vector Z component
 * @param depth Penetration depth (meters)
 */
FALCON_API void SetContact(float nx, float ny, float nz, float depth);

/**
 * Check if the Falcon device is calibrated (homed).
 * @return true if calibrated, false otherwise
 */
FALCON_API bool IsCalibrated();

/**
 * Start the calibration process. The haptic thread will handle the calibration.
 * User must move the Falcon fully outward and then straight inward.
 */
FALCON_API void StartCalibration();

/**
 * Set target position for position control mode.
 * The Falcon grip will be pulled towards this position using PID control.
 * @param x Target X position (meters)
 * @param y Target Y position (meters)
 * @param z Target Z position (meters)
 */
FALCON_API void SetTargetPosition(float x, float y, float z);

/**
 * Enable or disable position control mode.
 * When enabled, the grip tracks the target position set by SetTargetPosition.
 * When disabled, normal contact-based haptic feedback is used.
 * @param enable true to enable position control, false for contact mode
 */
FALCON_API void EnablePositionControl(bool enable);

#endif // FALCONBRIDGE_H
