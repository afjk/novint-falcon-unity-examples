# Novint Falcon Unity Examples

Unity integration for the Novint Falcon haptic device using libnifalcon.

## Overview

This project provides a native C++ plugin (FalconBridge) that enables Unity to control the Novint Falcon haptic device. The plugin uses libnifalcon for device communication and provides a simple C# API for Unity developers.

## Features

- **Native C++ Plugin**: High-performance haptic feedback loop running at 1kHz
- **Thread-Safe**: Separate haptic thread for consistent force output
- **Simple API**: Easy-to-use C# wrapper with Unity Vector3 support
- **Force Model**: Spring-based force feedback (F = -k × depth × normal, k=3000 N/m, max ±3N)
- **Example Scripts**: Multiple demonstration scripts showing different usage patterns

## Project Structure

```
Assets/
├── Plugins/
│   ├── FalconBridge/
│   │   ├── FalconBridge.h        # C++ header
│   │   ├── FalconBridge.cpp      # C++ implementation
│   │   ├── CMakeLists.txt        # Build configuration
│   │   └── BUILD.md              # Build instructions
│   └── macOS/
│       └── FalconBridge.bundle   # Compiled plugin (created after build)
└── Scripts/
    ├── FalconBridge.cs           # C# wrapper
    ├── FalconController.cs       # Basic example
    ├── FalconCollisionHandler.cs # Advanced collision example
    └── FalconRecordPlayback.cs   # Record/replay sample with LED cues
```

## Quick Start

### Prerequisites

1. **macOS** (Linux and Windows support planned)
2. **Novint Falcon device**
3. **Unity 2020.3 or later**
4. **libnifalcon** ([installation instructions](https://github.com/libnifalcon/libnifalcon))
5. **CMake 3.10+**

### Building the Plugin

See [Assets/Plugins/FalconBridge/BUILD.md](Assets/Plugins/FalconBridge/BUILD.md) for detailed build instructions.

Quick build:
```bash
cd Assets/Plugins/FalconBridge
mkdir build && cd build
cmake ..
make
```

This creates `FalconBridge.bundle` in `Assets/Plugins/macOS/`.

### Basic Usage

#### 1. Simple Plane Example

Add `FalconController.cs` to any GameObject:

```csharp
using UnityEngine;

public class Example : MonoBehaviour
{
    void Start()
    {
        // Initialize Falcon
        if (!FalconBridge.InitFalcon())
        {
            Debug.LogError("Failed to initialize Falcon");
            return;
        }
    }

    void Update()
    {
        // Get position
        Vector3 pos;
        if (FalconBridge.GetToolPosition(out pos))
        {
            // Apply force when below y=0
            if (pos.y < 0)
            {
                FalconBridge.SetContact(Vector3.up, -pos.y);
            }
            else
            {
                FalconBridge.ClearContact();
            }
        }
    }

    void OnDestroy()
    {
        FalconBridge.ShutdownFalcon();
    }
}
```

#### 2. 3D Object Collision Example

For collision-based haptics, use `FalconCollisionHandler.cs`:

1. Create an empty GameObject
2. Add `FalconCollisionHandler` component
3. Create objects with colliders in the scene
4. Run the scene - you'll feel the objects when the Falcon tool touches them!

## API Reference

### FalconBridge (C# API)

#### Initialization
```csharp
bool InitFalcon()
```
Initialize the Falcon device. Returns `true` on success.

#### Shutdown
```csharp
void ShutdownFalcon()
```
Shutdown the device and stop the haptic loop.

#### Get Position
```csharp
bool GetToolPosition(out Vector3 position)
```
Get the current tool position in meters. Returns `true` on success.

#### Set Contact
```csharp
void SetContact(Vector3 normal, float depth)
void SetContact(float nx, float ny, float nz, float depth)
```
Set contact parameters for haptic feedback:
- `normal`: Surface normal vector (should be normalized)
- `depth`: Penetration depth in meters

Force calculation: **F = -3000 × depth × normal** (clamped to ±3N)

#### Clear Contact
```csharp
void ClearContact()
```
Stop all haptic forces.

## Example Scripts

### FalconController.cs
Basic controller demonstrating:
- Device initialization
- Position tracking
- Simple plane collision
- Visual cursor
- Debug logging

### FalconCollisionHandler.cs
Advanced controller with:
- 3D collider integration
- Automatic normal calculation
- Stiffness adjustment
- Visual feedback
- Contact detection

### FalconRecordPlayback.cs
Recording and playback workflow that:
- Captures Falcon grip motion while button **3** is held
- Stops recording when button 3 is released
- Plays back the captured path (looping) when button **1** is pressed
- Stops playback when button 1 is pressed again
- Drives the Falcon LEDs (recording = red, playback = blue, idle = green)
- Keeps forces disabled whenever playback is not active so the arm stays limp during recording/idle

Attach it alongside `FalconController` (with a small sphere assigned to `playbackCursor`) to reproduce recorded trajectories.

## Sample Scenes

- **SampleScene.unity** – original plane/collision demo
- **FalconRecordPlaybackScene.unity** – ready-to-run scene wired for the record/replay flow:
  1. Select `Assets/Scenes/FalconRecordPlaybackScene.unity`.
  2. Ensure the Falcon is calibrated (LED green) and press Play.
  3. Hold button 3 to record a path, release to stop.
  4. Press button 1 to start looping playback (LED turns blue), press again to stop (LED returns green).

## Force Model

The haptic force is calculated using a simple spring model:

```
F = -k × depth × n
```

Where:
- **k = 3000 N/m** (spring stiffness, fixed)
- **depth** = penetration depth in meters
- **n** = surface normal (unit vector)
- **F** = output force (clamped to ±3N)

This provides realistic contact feedback for virtual objects.

## Troubleshooting

### Plugin Not Loading
1. Check Unity Console for errors
2. Verify `FalconBridge.bundle` exists in `Assets/Plugins/macOS/`
3. Check plugin import settings (should be macOS-only)

### Device Not Found
1. Ensure Falcon is connected and powered on
2. Check USB permissions (System Preferences → Security & Privacy)
3. Verify libnifalcon installation: `pkg-config --libs libnifalcon`

### No Force Output
1. Check `InitFalcon()` returns `true`
2. Verify firmware loaded successfully (check logs)
3. Ensure `SetContact()` is called each frame during contact
4. Test with a simple example first

### Build Errors
See [BUILD.md](Assets/Plugins/FalconBridge/BUILD.md) for detailed troubleshooting.

## Platform Support

| Platform | Status |
|----------|--------|
| macOS    | ✅ Supported |
| Windows  | 🚧 Planned |
| Linux    | 🚧 Planned |

## Performance

- **Haptic Loop**: 1000 Hz (1ms update rate)
- **Position Update**: Every Unity frame
- **Latency**: < 2ms (device + processing)

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

## References

- [libnifalcon](https://github.com/libnifalcon/libnifalcon) - Novint Falcon driver library
- [novint-falcon-examples](https://github.com/afjk/novint-falcon-examples) - C++ examples

## License

See LICENSE file for details.

## Acknowledgments

- libnifalcon developers
- Novint Technologies (original Falcon hardware)
