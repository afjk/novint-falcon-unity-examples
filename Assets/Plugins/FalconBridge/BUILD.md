# FalconBridge Build Instructions

This document describes how to build the FalconBridge native plugin for Unity on macOS.

## Prerequisites

### 1. Install libnifalcon

You need to install libnifalcon and its dependencies. The easiest way is using Homebrew:

```bash
# Install dependencies
brew install cmake pkg-config libusb

# Clone and build libnifalcon
git clone https://github.com/libnifalcon/libnifalcon.git
cd libnifalcon
mkdir build && cd build
cmake ..
make
sudo make install
```

Alternatively, if you have libnifalcon installed in a custom location, you'll need to adjust the CMakeLists.txt file to point to the correct paths.

### 2. Install Build Tools

Make sure you have CMake and a C++ compiler installed:

```bash
# Check if CMake is installed
cmake --version

# Install Xcode Command Line Tools if needed
xcode-select --install
```

## Building the Plugin

### Step 1: Navigate to the FalconBridge directory

```bash
cd Assets/Plugins/FalconBridge
```

### Step 2: Create a build directory

```bash
mkdir build
cd build
```

### Step 3: Configure CMake

```bash
cmake ..
```

If libnifalcon is installed in a non-standard location, you may need to specify the paths:

```bash
cmake .. \
  -DNIFALCON_LIB=/path/to/libnifalcon.dylib \
  -DNIFALCON_INCLUDE=/path/to/include
```

### Step 4: Build the plugin

```bash
make
```

This will create `FalconBridge.bundle` in the `Assets/Plugins/macOS` directory.

### Step 5: Verify the build

```bash
ls -la ../macOS/FalconBridge.bundle
```

You should see the bundle file created with a recent timestamp.

## Troubleshooting

### libnifalcon not found

If CMake can't find libnifalcon, try:

1. Check if libnifalcon is installed:
   ```bash
   pkg-config --libs --cflags libnifalcon
   ```

2. Set the PKG_CONFIG_PATH:
   ```bash
   export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH
   cmake ..
   ```

3. Manually specify paths in CMakeLists.txt or via command line arguments.

### USB Permission Issues

On macOS, you may need to grant USB access permissions:

1. System Preferences → Security & Privacy → Privacy → Input Monitoring
2. Add Unity or Terminal to the allowed applications

### Build Errors

If you encounter build errors:

1. Make sure you have the latest version of Xcode Command Line Tools
2. Verify that libnifalcon is properly installed
3. Check that all dependencies are met (libusb, etc.)

## Testing the Plugin

### Step 1: Return to Unity

After building, return to Unity. The Unity Editor should automatically detect the new plugin.

### Step 2: Check Unity Console

Open the Unity Console (Window → General → Console) and look for any plugin loading errors.

### Step 3: Run a test script

Create a simple test script in Unity:

```csharp
using UnityEngine;

public class FalconTest : MonoBehaviour
{
    void Start()
    {
        if (FalconBridge.InitFalcon())
        {
            Debug.Log("Falcon initialized successfully!");
        }
        else
        {
            Debug.LogError("Failed to initialize Falcon");
        }
    }

    void Update()
    {
        Vector3 pos;
        if (FalconBridge.GetToolPosition(out pos))
        {
            Debug.Log($"Falcon position: {pos}");
        }

        // Example: Apply force when touching a virtual plane at y=0
        if (pos.y < 0)
        {
            FalconBridge.SetContact(Vector3.up, -pos.y);
        }
        else
        {
            FalconBridge.ClearContact();
        }
    }

    void OnDestroy()
    {
        FalconBridge.ShutdownFalcon();
        Debug.Log("Falcon shutdown");
    }
}
```

### Step 4: Connect Falcon Device

1. Connect your Novint Falcon device via USB
2. Make sure it's powered on
3. Run the Unity scene with the test script

## Expected Behavior

When everything is working correctly:

1. `InitFalcon()` returns `true`
2. Console shows "Falcon initialized successfully!"
3. `GetToolPosition()` returns the current tool position in meters
4. `SetContact()` applies force feedback to the device
5. `ShutdownFalcon()` cleanly stops the haptic loop

## Force Model

The plugin uses a simple spring-based force model:

```
F = -k * depth * normal
```

Where:
- `k = 3000 N/m` (spring stiffness)
- `depth` = penetration depth in meters
- `normal` = surface normal vector (should be normalized)
- Maximum force output is clamped to ±3N

## Architecture

The plugin consists of:

1. **FalconBridge.cpp/h** - Native C++ implementation
   - Device initialization and control
   - 1kHz haptic feedback loop running in a separate thread
   - Thread-safe contact parameter updates

2. **FalconBridge.cs** - Unity C# wrapper
   - P/Invoke declarations for native functions
   - Convenience methods for Vector3 operations

3. **FalconBridge.bundle** - Compiled native plugin
   - Located in Assets/Plugins/macOS/
   - Automatically loaded by Unity on macOS

## References

- libnifalcon: https://github.com/libnifalcon/libnifalcon
- Novint Falcon examples: https://github.com/afjk/novint-falcon-examples
