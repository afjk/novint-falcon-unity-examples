#include "FalconBridge.h"
#include <falcon/core/FalconDevice.h>
#include <falcon/firmware/FalconFirmwareNovintSDK.h>
#include <falcon/kinematic/FalconKinematicStamper.h>
#include <falcon/util/FalconFirmwareBinaryNvent.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <iostream>
#include <cmath>
#include <array>
#include <mutex>

using namespace libnifalcon;

// Global state
static FalconDevice g_falcon;
static std::atomic<bool> g_running(false);
static std::thread g_hapticThread;

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

// Calibration state
static std::atomic<bool> g_calibrating(false);
static std::atomic<bool> g_calibrated(false);

// Current position cache
static std::mutex g_posMutex;
static std::array<double, 3> g_currentPos = {0.0, 0.0, 0.0};

/**
 * Haptic feedback loop running at 1kHz
 */
static void HapticLoop()
{
    while (g_running)
    {
        try
        {
            // Step 1: Run IO loop FIRST to read device state from USB
            // Note: runIOLoop() can occasionally return false - this is normal
            if (!g_falcon.runIOLoop())
            {
                int failures = g_consecutiveFailures.fetch_add(1) + 1;
                if (failures > kIOLoopFailureLimit)
                {
                    std::cerr << "IO loop failed " << failures << " times consecutively" << std::endl;
                    std::cerr << "  Device error code: " << g_falcon.getErrorCode() << std::endl;
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

            // Step 2: Get current position (after successful IO loop)
            std::array<double, 3> pos = g_falcon.getPosition();

            // Debug: Log position every 100 successful loops (more frequent)
            int loopCount = g_successfulLoops.fetch_add(1) + 1;
            if (loopCount % 100 == 0)
            {
                std::cout << "C++ Position [" << loopCount << "]: ("
                          << pos[0] << ", " << pos[1] << ", " << pos[2] << ")"
                          << " magnitude: " << std::sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2])
                          << std::endl;
            }

            // Cache position for GetToolPose
            {
                std::lock_guard<std::mutex> lock(g_posMutex);
                g_currentPos = pos;
            }

            // Step 3: Handle calibration or normal operation
            auto firmware = g_falcon.getFalconFirmware();
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
            else if (g_calibrated.load())
            {
                // Normal operation: Read contact parameters
                float n[3] = {
                    g_nx.load(),
                    g_ny.load(),
                    g_nz.load()
                };
                float d = g_depth.load();

                // Calculate force: F = -k * depth * n
                std::array<double, 3> force = {
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

                // Set force (will be sent on next IO loop)
                g_falcon.setForce(force);
            }
            else
            {
                // Not calibrated yet - set zero force
                std::array<double, 3> force = {0.0, 0.0, 0.0};
                g_falcon.setForce(force);
            }

            // Run at 1kHz
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
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
        // Check if already running
        if (g_running.load())
        {
            std::cerr << "Falcon already initialized" << std::endl;
            return false;
        }

        // Set firmware type BEFORE opening device (critical!)
        std::cout << "Setting firmware type..." << std::endl;
        g_falcon.setFalconFirmware<FalconFirmwareNovintSDK>();

        // Open the first Falcon device
        std::cout << "Opening Falcon device..." << std::endl;
        if (!g_falcon.open(0))
        {
            std::cerr << "Failed to open Falcon device" << std::endl;
            std::cerr << "  Error code: " << g_falcon.getErrorCode() << std::endl;
            auto comm = g_falcon.getFalconComm();
            if (comm != nullptr)
            {
                std::cerr << "  Comm error code: " << comm->getErrorCode() << std::endl;
            }
            return false;
        }
        std::cout << "Device opened successfully" << std::endl;

        // Load firmware if needed
        std::cout << "Checking firmware..." << std::endl;
        if (!g_falcon.isFirmwareLoaded())
        {
            std::cout << "Loading firmware..." << std::endl;

            // Try loading firmware (skip_checksum = false for better reliability)
            for (int i = 0; i < 10; ++i)
            {
                std::cout << "Firmware load attempt " << (i+1) << " / 10" << std::endl;
                if (g_falcon.getFalconFirmware()->loadFirmware(false,
                    NOVINT_FALCON_NVENT_FIRMWARE_SIZE,
                    const_cast<uint8_t*>(NOVINT_FALCON_NVENT_FIRMWARE)))
                {
                    std::cout << "Firmware loaded successfully" << std::endl;
                    break;
                }
                std::cerr << "Firmware load attempt " << (i+1) << " failed" << std::endl;
                std::cerr << "  Device error code: " << g_falcon.getErrorCode() << std::endl;
                auto firmware = g_falcon.getFalconFirmware();
                if (firmware != nullptr)
                {
                    std::cerr << "  Firmware error code: " << firmware->getErrorCode() << std::endl;
                }
            }

            if (!g_falcon.isFirmwareLoaded())
            {
                std::cerr << "Failed to load firmware after retries" << std::endl;
                g_falcon.close();
                return false;
            }
        }

        std::cout << "Firmware ready" << std::endl;

        // Set kinematic model for position conversion
        std::cout << "Setting kinematic model..." << std::endl;
        g_falcon.setFalconKinematic<FalconKinematicStamper>();

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
        return false;
    }
    catch (...)
    {
        std::cerr << "Unknown exception in InitFalcon" << std::endl;
        return false;
    }
}

FALCON_API void ShutdownFalcon()
{
    try
    {
        std::cout << "Shutting down Falcon..." << std::endl;

        // Stop haptic thread
        g_running.store(false);
        if (g_hapticThread.joinable())
        {
            g_hapticThread.join();
        }

        // Reset contact parameters
        g_nx.store(0.0f);
        g_ny.store(0.0f);
        g_nz.store(0.0f);
        g_depth.store(0.0f);
        g_consecutiveFailures.store(0);
        g_successfulLoops.store(0);
        g_calibrating.store(false);
        g_calibrated.store(false);

        // Close device
        g_falcon.close();

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
