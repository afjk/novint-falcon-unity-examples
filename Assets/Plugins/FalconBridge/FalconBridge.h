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

#endif // FALCONBRIDGE_H
