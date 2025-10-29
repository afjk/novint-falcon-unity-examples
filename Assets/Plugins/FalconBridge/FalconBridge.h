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
 * Adjust the native spring constant used for contact forces.
 * @param stiffness Spring stiffness (N/m)
 */
FALCON_API void SetContactStiffness(float stiffness);

/**
 * Adjust the native damping term used for contact forces.
 * @param damping Damping gain (N·s/m)
 */
FALCON_API void SetContactDamping(float damping);

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

/**
 * Set PID control parameters for position control.
 * Adjust these to tune the tracking behavior and reduce vibration.
 * @param kp Proportional gain (lower = softer, higher = stiffer)
 * @param ki Integral gain (lower = less windup)
 * @param kd Derivative gain (higher = more damping, reduces oscillation)
 * @param filterAlpha Low-pass filter coefficient (0.01-1.0, lower = smoother)
 * @param maxForce Maximum force per axis in Newtons (0.5-3.0)
 */
FALCON_API void SetPIDParameters(float kp, float ki, float kd, float filterAlpha, float maxForce);

/**
 * Set LED status on the Falcon device.
 * Multiple LEDs can be enabled by OR-ing the values together.
 * @param ledMask LED mask (0x1=BLUE, 0x2=GREEN, 0x4=AMBER, 0x8=RED)
 */
FALCON_API void SetLEDStatus(int ledMask);

/**
 * Get the current state of the Falcon's buttons.
 * @param button1 Pointer to store button 1 state (true if pressed)
 * @param button2 Pointer to store button 2 state (true if pressed)
 * @param button3 Pointer to store button 3 state (true if pressed)
 * @param button4 Pointer to store button 4 state (true if pressed)
 * @return true if button states retrieved successfully, false otherwise
 */
FALCON_API bool GetButtonStates(bool* button1, bool* button2, bool* button3, bool* button4);

#endif // FALCONBRIDGE_H
