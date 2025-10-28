#include "FalconBridge.h"
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/grip/FalconGripFourButton.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <cmath>
#include <array>
#include <mutex>
#include <algorithm>

using namespace libnifalcon;

// Global state
static FalconDevice* g_falcon = nullptr;
static std::atomic<bool> g_running(false);
static std::thread g_hapticThread;
static std::mutex g_falconMutex;  // Protect device access

// Contact parameters (thread-safe)
static std::atomic<float> g_nx(0.0f);
static std::atomic<float> g_ny(0.0f);
static std::atomic<float> g_nz(0.0f);
static std::atomic<float> g_depth(0.0f);

// Force model parameters
static const float K = 3000.0f;  // Spring stiffness
static const float FMAX = 3.0f;  // Maximum force (Newtons)

// IO loop failure tracking
static const int kIOLoopFailureLimit = 200;
static std::atomic<int> g_consecutiveFailures(0);
static std::atomic<int> g_successfulLoops(0);
static constexpr bool kEnableHapticLoopLogging = false;  // Disable per-iteration logging by default

// Calibration state
static std::atomic<bool> g_calibrating(false);
static std::atomic<bool> g_calibrated(false);

// Current position cache
static std::mutex g_posMutex;
static std::array<double, 3> g_currentPos = {0.0, 0.0, 0.0};

// Button state cache
static std::mutex g_buttonMutex;
static std::atomic<bool> g_button1(false);
static std::atomic<bool> g_button2(false);
static std::atomic<bool> g_button3(false);
static std::atomic<bool> g_button4(false);

// LED control
static std::atomic<int> g_ledMask(0);

// Position control mode
static std::atomic<bool> g_positionControlEnabled(false);
static std::atomic<float> g_targetX(0.0f);
static std::atomic<float> g_targetY(0.0f);
static std::atomic<float> g_targetZ(0.0f);
static std::atomic<double> g_targetStartX(0.0);
static std::atomic<double> g_targetStartY(0.0);
static std::atomic<double> g_targetStartZ(0.0);
static std::atomic<double> g_targetBlendDuration(0.02);
static std::atomic<long long> g_targetBlendStartNs(0);
static std::atomic<bool> g_targetInitialized(false);

// PID control parameters (adjustable at runtime)
static std::atomic<double> PID_Kp(100.0);         // Proportional gain
static std::atomic<double> PID_Ki(5.0);           // Integral gain
static std::atomic<double> PID_Kd(8.0);           // Derivative gain
static const double PID_integralLimit = 0.005;    // Integral limit (fixed)
static std::atomic<double> PID_alpha(0.1);        // Low-pass filter coefficient
static std::atomic<double> PID_maxForce(2.0);     // Maximum force per axis

// PID control state variables
static std::array<double, 3> g_lastPidPos = {0.0, 0.0, 0.0};
static double g_xErrorIntegral = 0.0;
static double g_yErrorIntegral = 0.0;
static double g_zErrorIntegral = 0.0;
static double g_filteredTargetX = 0.0;
static double g_filteredTargetY = 0.0;
static double g_filteredTargetZ = 0.0;

/**
 * Haptic feedback loop running at 1kHz
 */
static void HapticLoop()
{
    static auto lastLoopTime = std::chrono::steady_clock::now();
    constexpr double kNominalDt = 1.0 / 1000.0;  // 1 kHz nominal timestep

    while (g_running)
    {
        try
        {
            // Check if device is valid
            if (g_falcon == nullptr)
            {
                std::cerr << "Device is null in haptic loop" << std::endl;
                g_running.store(false);
                break;
            }

            // Step 1: Run IO loop FIRST to read device state from USB
            // Note: runIOLoop() can occasionally return false - this is normal
            if (!g_falcon->runIOLoop())
            {
                int failures = g_consecutiveFailures.fetch_add(1) + 1;
                if (failures > kIOLoopFailureLimit)
                {
                    std::cerr << "IO loop failed " << failures << " times consecutively" << std::endl;
                    std::cerr << "  Device error code: " << g_falcon->getErrorCode() << std::endl;
                    g_running.store(false);
                    break;
                }
                else if (failures % 50 == 1)  // Log every 50 failures
                {
                    std::cerr << "IO loop failures: " << failures << std::endl;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }

            // Reset failure counter on success
            int prevFailures = g_consecutiveFailures.exchange(0);
            if (prevFailures > 0)
            {
                std::cout << "IO loop recovered after " << prevFailures << " failures" << std::endl;
            }

            // Measure loop delta time to keep PID timing consistent even if cadence drifts
            auto now = std::chrono::steady_clock::now();
            double deltaSeconds = std::chrono::duration<double>(now - lastLoopTime).count();
            lastLoopTime = now;
            if (deltaSeconds <= 0.0)
            {
                deltaSeconds = kNominalDt;
            }
            long long nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

            // Use nominal timestep for control math (reduces noise from scheduling jitter)
            constexpr double dt = kNominalDt;
            constexpr double invDt = 1000.0;  // 1 / 0.001

            // Step 2: Get current position (after successful IO loop)
            std::array<double, 3> pos = g_falcon->getPosition();

            // Debug: Log position every 100 successful loops (more frequent)
            int loopCount = g_successfulLoops.fetch_add(1) + 1;
            if (kEnableHapticLoopLogging && loopCount % 100 == 0)
            {
                std::cout << "C++ Position [" << loopCount << "]: ("
                          << pos[0] << ", " << pos[1] << ", " << pos[2] << ")"
                          << " magnitude: " << std::sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2])
                          << std::endl;
            }

            // runIOLoop() already sets the device cadence (~1 kHz), so avoid extra sleeps here

            // Cache position for GetToolPose
            {
                std::lock_guard<std::mutex> lock(g_posMutex);
                g_currentPos = pos;
            }

            // Step 3: Handle calibration or normal operation
            auto firmware = g_falcon->getFalconFirmware();
            if (g_calibrating.load() && firmware != nullptr)
            {
                // Calibration mode
                firmware->setHomingMode(true);
                firmware->setLEDStatus(0x8);  // RED_LED = 0x8

                if (firmware->isHomed())
                {
                    std::cout << "Calibration complete!" << std::endl;
                    g_calibrating.store(false);
                    g_calibrated.store(true);
                    firmware->setHomingMode(false);
                    firmware->setLEDStatus(0x2);  // GREEN_LED = 0x2
                }
            }
            else if (firmware != nullptr)
            {
                // Normal operation - set user-controlled LED status
                firmware->setLEDStatus(g_ledMask.load());

                // Read button states from firmware grip info
                // Cast to FalconFirmwareNovintSDK to access getGripInfo()
                auto firmwareShared = g_falcon->getFalconFirmware();
                auto novintFirmware = std::dynamic_pointer_cast<FalconFirmwareNovintSDK>(firmwareShared);
                if (novintFirmware != nullptr)
                {
                    const uint8_t* gripInfo = novintFirmware->getGripInfo();
                    if (gripInfo != nullptr)
                    {
                        uint8_t buttons = *gripInfo;
                        g_button1.store((buttons & 0x01) != 0);  // PLUS_BUTTON
                        g_button2.store((buttons & 0x02) != 0);  // FORWARD_BUTTON
                        g_button3.store((buttons & 0x04) != 0);  // CENTER_BUTTON
                        g_button4.store((buttons & 0x08) != 0);  // MINUS_BUTTON
                    }
                }
            }

            if (g_calibrated.load())
            {
                std::array<double, 3> force;

                if (g_positionControlEnabled.load())
                {
                    // Position control mode: PID control to track target position

                    // Interpolate target between Unity updates for smoother motion
                    long long blendStartNs = g_targetBlendStartNs.load();
                    double blendDuration = g_targetBlendDuration.load();
                    double elapsedSinceUpdate = 0.0;
                    if (blendStartNs > 0)
                    {
                        elapsedSinceUpdate = (nowNs - blendStartNs) * 1e-9;
                        if (elapsedSinceUpdate < 0.0)
                        {
                            elapsedSinceUpdate = 0.0;
                        }
                    }

                    double startX = g_targetStartX.load();
                    double startY = g_targetStartY.load();
                    double startZ = g_targetStartZ.load();
                    double goalX = static_cast<double>(g_targetX.load());
                    double goalY = static_cast<double>(g_targetY.load());
                    double goalZ = static_cast<double>(g_targetZ.load());

                    double progress = 1.0;
                    if (blendDuration > 0.0005)
                    {
                        progress = elapsedSinceUpdate / blendDuration;
                        if (progress < 0.0) progress = 0.0;
                        if (progress > 1.0) progress = 1.0;
                    }

                    double targetX = startX + (goalX - startX) * progress;
                    double targetY = startY + (goalY - startY) * progress;
                    double targetZ = startZ + (goalZ - startZ) * progress;

                    double alpha = PID_alpha.load();
                    if (alpha < 0.0) alpha = 0.0;
                    if (alpha > 1.0) alpha = 1.0;
                    g_filteredTargetX = (1.0 - alpha) * g_filteredTargetX + alpha * targetX;
                    g_filteredTargetY = (1.0 - alpha) * g_filteredTargetY + alpha * targetY;
                    g_filteredTargetZ = (1.0 - alpha) * g_filteredTargetZ + alpha * targetZ;

                    // Load PID parameters
                    double kp = PID_Kp.load();
                    double ki = PID_Ki.load();
                    double kd = PID_Kd.load();

                    // X-axis PID control
                    double xError = g_filteredTargetX - pos[0];
                    double xVelocity = (pos[0] - g_lastPidPos[0]) * invDt;  // velocity in m/s
                    g_xErrorIntegral += xError * dt;
                    if (g_xErrorIntegral > PID_integralLimit) g_xErrorIntegral = PID_integralLimit;
                    if (g_xErrorIntegral < -PID_integralLimit) g_xErrorIntegral = -PID_integralLimit;
                    double xForce = kp * xError + ki * g_xErrorIntegral - kd * xVelocity;

                    // Y-axis PID control
                    double yError = g_filteredTargetY - pos[1];
                    double yVelocity = (pos[1] - g_lastPidPos[1]) * invDt;
                    g_yErrorIntegral += yError * dt;
                    if (g_yErrorIntegral > PID_integralLimit) g_yErrorIntegral = PID_integralLimit;
                    if (g_yErrorIntegral < -PID_integralLimit) g_yErrorIntegral = -PID_integralLimit;
                    double yForce = kp * yError + ki * g_yErrorIntegral - kd * yVelocity;

                    // Z-axis PID control
                    double zError = g_filteredTargetZ - pos[2];
                    double zVelocity = (pos[2] - g_lastPidPos[2]) * invDt;
                    g_zErrorIntegral += zError * dt;
                    if (g_zErrorIntegral > PID_integralLimit) g_zErrorIntegral = PID_integralLimit;
                    if (g_zErrorIntegral < -PID_integralLimit) g_zErrorIntegral = -PID_integralLimit;
                    double zForce = kp * zError + ki * g_zErrorIntegral - kd * zVelocity;

                    // Clamp forces to safe maximum
                    double maxForce = PID_maxForce.load();
                    xForce = std::max(-maxForce, std::min(maxForce, xForce));
                    yForce = std::max(-maxForce, std::min(maxForce, yForce));
                    zForce = std::max(-maxForce, std::min(maxForce, zForce));

                    force = {xForce, yForce, zForce};

                    // Update last position for derivative calculation
                    g_lastPidPos = pos;
                }
                else
                {
                    // Contact-based haptic feedback mode
                    float n[3] = {
                        g_nx.load(),
                        g_ny.load(),
                        g_nz.load()
                    };
                    float d = g_depth.load();

                    // Calculate force: F = -k * depth * n
                    force = {
                        -K * d * n[0],
                        -K * d * n[1],
                        -K * d * n[2]
                    };

                    // Clamp force to maximum magnitude
                    double forceMagnitude = std::sqrt(
                        force[0] * force[0] +
                        force[1] * force[1] +
                        force[2] * force[2]
                    );

                    if (forceMagnitude > FMAX && forceMagnitude > 0.0)
                    {
                        double scale = FMAX / forceMagnitude;
                        force[0] *= scale;
                        force[1] *= scale;
                        force[2] *= scale;
                    }
                }

                // Set force (will be sent on next IO loop)
                g_falcon->setForce(force);
            }
            else
            {
                // Not calibrated yet - set zero force
                std::array<double, 3> force = {0.0, 0.0, 0.0};
                g_falcon->setForce(force);
            }

        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception in haptic loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        catch (...)
        {
            std::cerr << "Unknown exception in haptic loop" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
}

FALCON_API bool InitFalcon()
{
    try
    {
        // If already running, force shutdown first
        if (g_running.load())
        {
            std::cout << "Falcon already initialized, shutting down first..." << std::endl;
            ShutdownFalcon();
            // Wait a bit to ensure complete shutdown
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        // Create a NEW device object for fresh USB state
        std::cout << "Creating new device instance..." << std::endl;
        std::lock_guard<std::mutex> lock(g_falconMutex);

        if (g_falcon != nullptr)
        {
            delete g_falcon;
            g_falcon = nullptr;
        }

        g_falcon = new FalconDevice();
        std::cout << "New device instance created" << std::endl;

        // Set firmware type BEFORE opening device (critical!)
        std::cout << "Setting firmware type..." << std::endl;
        g_falcon->setFalconFirmware<FalconFirmwareNovintSDK>();

        // Open the first Falcon device
        std::cout << "Opening Falcon device..." << std::endl;
        if (!g_falcon->open(0))
        {
            std::cerr << "Failed to open Falcon device" << std::endl;
            std::cerr << "  Error code: " << g_falcon->getErrorCode() << std::endl;
            auto comm = g_falcon->getFalconComm();
            if (comm != nullptr)
            {
                std::cerr << "  Comm error code: " << comm->getErrorCode() << std::endl;
            }
            delete g_falcon;
            g_falcon = nullptr;
            return false;
        }
        std::cout << "Device opened successfully" << std::endl;

        // Load firmware if needed
        std::cout << "Checking firmware..." << std::endl;
        if (!g_falcon->isFirmwareLoaded())
        {
            std::cout << "Loading firmware..." << std::endl;

            // Try loading firmware (skip_checksum = false for better reliability)
            for (int i = 0; i < 10; ++i)
            {
                std::cout << "Firmware load attempt " << (i+1) << " / 10" << std::endl;
                if (g_falcon->getFalconFirmware()->loadFirmware(false,
                    NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
                    const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
                {
                    std::cout << "Firmware loaded successfully" << std::endl;
                    break;
                }
                std::cerr << "Firmware load attempt " << (i+1) << " failed" << std::endl;
                std::cerr << "  Device error code: " << g_falcon->getErrorCode() << std::endl;
                auto firmware = g_falcon->getFalconFirmware();
                if (firmware != nullptr)
                {
                    std::cerr << "  Firmware error code: " << firmware->getErrorCode() << std::endl;
                }
            }

            if (!g_falcon->isFirmwareLoaded())
            {
                std::cerr << "Failed to load firmware after retries" << std::endl;
                g_falcon->close();
                delete g_falcon;
                g_falcon = nullptr;
                return false;
            }
        }

        std::cout << "Firmware ready" << std::endl;

        // Set kinematic model for position conversion
        std::cout << "Setting kinematic model..." << std::endl;
        g_falcon->setFalconKinematic<FalconKinematicStamper>();

        // Set grip model for button input
        std::cout << "Setting grip model..." << std::endl;
        g_falcon->setFalconGrip<FalconGripFourButton>();

        // Start haptic thread
        std::cout << "Starting haptic thread..." << std::endl;
        g_running.store(true);
        g_hapticThread = std::thread(HapticLoop);

        std::cout << "Falcon initialized successfully" << std::endl;
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception in InitFalcon: " << e.what() << std::endl;
        if (g_falcon != nullptr)
        {
            delete g_falcon;
            g_falcon = nullptr;
        }
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in InitFalcon" << std::endl;
        if (g_falcon != nullptr)
        {
            delete g_falcon;
            g_falcon = nullptr;
        }
        return false;
    }
}

FALCON_API void ShutdownFalcon()
{
    try
    {
        std::cout << "Shutting down Falcon..." << std::endl;

        // Stop haptic thread FIRST
        g_running.store(false);

        // Wait for thread to finish
        if (g_hapticThread.joinable())
        {
            std::cout << "Waiting for haptic thread to stop..." << std::endl;
            g_hapticThread.join();
            std::cout << "Haptic thread stopped" << std::endl;
        }

        // Close device and delete object
        if (g_falcon != nullptr)
        {
            std::lock_guard<std::mutex> lock(g_falconMutex);

            // Clear all forces before closing
            std::array<double, 3> zeroForce = {0.0, 0.0, 0.0};
            g_falcon->setForce(zeroForce);

            // Run a few IO loops to ensure zero force is sent
            for (int i = 0; i < 3; ++i)
            {
                g_falcon->runIOLoop();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }

            // Close device
            std::cout << "Closing device..." << std::endl;
            g_falcon->close();

            // Delete the device object
            std::cout << "Deleting device object..." << std::endl;
            delete g_falcon;
            g_falcon = nullptr;
            std::cout << "Device object deleted" << std::endl;
        }

        // Reset all state variables
        g_nx.store(0.0f);
        g_ny.store(0.0f);
        g_nz.store(0.0f);
        g_depth.store(0.0f);
        g_consecutiveFailures.store(0);
        g_successfulLoops.store(0);
        g_calibrating.store(false);
        g_calibrated.store(false);
        g_targetStartX.store(0.0);
        g_targetStartY.store(0.0);
        g_targetStartZ.store(0.0);
        g_targetBlendDuration.store(0.02);
        g_targetBlendStartNs.store(0);
        g_targetInitialized.store(false);
        g_filteredTargetX = 0.0;
        g_filteredTargetY = 0.0;
        g_filteredTargetZ = 0.0;

        // Reset position cache
        {
            std::lock_guard<std::mutex> lock(g_posMutex);
            g_currentPos = {0.0, 0.0, 0.0};
        }

        std::cout << "Falcon shutdown complete" << std::endl;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception in ShutdownFalcon: " << e.what() << std::endl;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in ShutdownFalcon" << std::endl;
    }
}

FALCON_API bool GetToolPose(float* x, float* y, float* z)
{
    if (!g_running.load())
    {
        return false;
    }

    try
    {
        std::lock_guard<std::mutex> lock(g_posMutex);
        *x = static_cast<float>(g_currentPos[0]);
        *y = static_cast<float>(g_currentPos[1]);
        *z = static_cast<float>(g_currentPos[2]);
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception in GetToolPose: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in GetToolPose" << std::endl;
        return false;
    }
}

FALCON_API void SetContact(float nx, float ny, float nz, float depth)
{
    g_nx.store(nx);
    g_ny.store(ny);
    g_nz.store(nz);
    g_depth.store(depth);
}

FALCON_API bool IsCalibrated()
{
    return g_calibrated.load();
}

FALCON_API void StartCalibration()
{
    std::cout << "Starting calibration - move Falcon outward then straight inward" << std::endl;
    g_calibrating.store(true);
    g_calibrated.store(false);
}

FALCON_API void SetTargetPosition(float x, float y, float z)
{
    auto now = std::chrono::steady_clock::now();
    long long nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

    double newX = static_cast<double>(x);
    double newY = static_cast<double>(y);
    double newZ = static_cast<double>(z);

    double prevGoalX = static_cast<double>(g_targetX.load());
    double prevGoalY = static_cast<double>(g_targetY.load());
    double prevGoalZ = static_cast<double>(g_targetZ.load());

    long long prevStartNs = g_targetBlendStartNs.load();
    double prevDuration = g_targetBlendDuration.load();

    if (!g_targetInitialized.load())
    {
        g_targetStartX.store(newX);
        g_targetStartY.store(newY);
        g_targetStartZ.store(newZ);
        g_targetBlendDuration.store(0.0);
        g_targetBlendStartNs.store(nowNs);
        g_targetInitialized.store(true);
    }
    else
    {
        double elapsed = prevStartNs > 0 ? (nowNs - prevStartNs) * 1e-9 : prevDuration;
        if (elapsed < 0.001 || elapsed > 0.2)
        {
            elapsed = 0.02;  // default to 20 ms when timing is out of bounds
        }

        g_targetStartX.store(prevGoalX);
        g_targetStartY.store(prevGoalY);
        g_targetStartZ.store(prevGoalZ);
        g_targetBlendDuration.store(elapsed);
        g_targetBlendStartNs.store(nowNs);
    }

    g_targetX.store(x);
    g_targetY.store(y);
    g_targetZ.store(z);
}

FALCON_API void EnablePositionControl(bool enable)
{
    if (enable && !g_positionControlEnabled.load())
    {
        // Reset PID state when enabling position control
        g_xErrorIntegral = 0.0;
        g_yErrorIntegral = 0.0;
        g_zErrorIntegral = 0.0;

        auto now = std::chrono::steady_clock::now();
        long long nowNs = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

        double currentGoalX = static_cast<double>(g_targetX.load());
        double currentGoalY = static_cast<double>(g_targetY.load());
        double currentGoalZ = static_cast<double>(g_targetZ.load());

        g_targetStartX.store(currentGoalX);
        g_targetStartY.store(currentGoalY);
        g_targetStartZ.store(currentGoalZ);
        g_targetBlendDuration.store(0.0);
        g_targetBlendStartNs.store(nowNs);
        g_targetInitialized.store(true);

        // Initialize filtered targets to current target
        g_filteredTargetX = currentGoalX;
        g_filteredTargetY = currentGoalY;
        g_filteredTargetZ = currentGoalZ;

        // Initialize last position
        {
            std::lock_guard<std::mutex> lock(g_posMutex);
            g_lastPidPos = g_currentPos;
        }

        std::cout << "Position control enabled" << std::endl;
    }
    else if (!enable && g_positionControlEnabled.load())
    {
        std::cout << "Position control disabled" << std::endl;
    }

    g_positionControlEnabled.store(enable);

    if (!enable)
    {
        g_targetInitialized.store(false);
    }
}

FALCON_API void SetPIDParameters(float kp, float ki, float kd, float filterAlpha, float maxForce)
{
    PID_Kp.store(static_cast<double>(kp));
    PID_Ki.store(static_cast<double>(ki));
    PID_Kd.store(static_cast<double>(kd));
    PID_alpha.store(static_cast<double>(filterAlpha));
    PID_maxForce.store(static_cast<double>(maxForce));

    std::cout << "PID parameters updated: Kp=" << kp << ", Ki=" << ki << ", Kd=" << kd
              << ", alpha=" << filterAlpha << ", maxForce=" << maxForce << std::endl;
}

FALCON_API void SetLEDStatus(int ledMask)
{
    // Store LED mask - will be applied in haptic loop
    // Note: During calibration, LED status is overridden
    g_ledMask.store(ledMask);
}

FALCON_API bool GetButtonStates(bool* button1, bool* button2, bool* button3, bool* button4)
{
    if (!g_running.load())
    {
        return false;
    }

    try
    {
        *button1 = g_button1.load();
        *button2 = g_button2.load();
        *button3 = g_button3.load();
        *button4 = g_button4.load();
        return true;
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception in GetButtonStates: " << e.what() << std::endl;
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in GetButtonStates" << std::endl;
        return false;
    }
}
