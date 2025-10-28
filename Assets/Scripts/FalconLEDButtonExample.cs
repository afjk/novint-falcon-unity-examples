using UnityEngine;

/// <summary>
/// Example script demonstrating Falcon LED and button functionality.
/// Attach this to a GameObject to test LED control and button input.
/// </summary>
public class FalconLEDButtonExample : MonoBehaviour
{
    [Header("LED Settings")]
    [Tooltip("Current LED color mask")]
    public int currentLED = FalconBridge.LED.GREEN;

    [Header("Button State (Read-only)")]
    [Tooltip("Button 1 - PLUS/Right button")]
    public bool button1Pressed = false;

    [Tooltip("Button 2 - FORWARD button")]
    public bool button2Pressed = false;

    [Tooltip("Button 3 - CENTER button")]
    public bool button3Pressed = false;

    [Tooltip("Button 4 - MINUS/Left button")]
    public bool button4Pressed = false;

    [Header("LED Cycle Settings")]
    [Tooltip("Automatically cycle through LED colors")]
    public bool cycleLEDs = false;

    [Tooltip("Time between LED color changes (seconds)")]
    public float cycleDuration = 1.0f;

    private float lastCycleTime = 0f;
    private int[] ledColors = new int[]
    {
        FalconBridge.LED.BLUE,
        FalconBridge.LED.GREEN,
        FalconBridge.LED.AMBER,
        FalconBridge.LED.RED,
        FalconBridge.LED.BLUE | FalconBridge.LED.GREEN,  // Cyan
        FalconBridge.LED.RED | FalconBridge.LED.BLUE,    // Magenta
        FalconBridge.LED.RED | FalconBridge.LED.GREEN,   // Yellow
        FalconBridge.LED.OFF
    };
    private int currentLEDIndex = 0;

    void Start()
    {
        // Set initial LED color
        FalconBridge.SetLEDStatus(currentLED);
    }

    void Update()
    {
        // Read button states
        bool success = FalconBridge.GetButtonStates(
            out button1Pressed,
            out button2Pressed,
            out button3Pressed,
            out button4Pressed
        );

        if (!success)
        {
            // Falcon not initialized or not running
            return;
        }

        // Example: Change LED color based on button presses
        if (button1Pressed)
        {
            FalconBridge.SetLEDStatus(FalconBridge.LED.BLUE);
        }
        else if (button2Pressed)
        {
            FalconBridge.SetLEDStatus(FalconBridge.LED.GREEN);
        }
        else if (button3Pressed)
        {
            FalconBridge.SetLEDStatus(FalconBridge.LED.AMBER);
        }
        else if (button4Pressed)
        {
            FalconBridge.SetLEDStatus(FalconBridge.LED.RED);
        }

        // LED cycling mode
        if (cycleLEDs)
        {
            if (Time.time - lastCycleTime >= cycleDuration)
            {
                currentLEDIndex = (currentLEDIndex + 1) % ledColors.Length;
                currentLED = ledColors[currentLEDIndex];
                FalconBridge.SetLEDStatus(currentLED);
                lastCycleTime = Time.time;

                Debug.Log($"LED changed to: {GetLEDName(currentLED)}");
            }
        }
    }

    void OnGUI()
    {
        // Display button states on screen
        GUILayout.BeginArea(new Rect(10, 10, 300, 200));
        GUILayout.Label("Falcon Button States:", GUI.skin.box);
        GUILayout.Label($"Button 1 (PLUS): {(button1Pressed ? "PRESSED" : "Released")}");
        GUILayout.Label($"Button 2 (FORWARD): {(button2Pressed ? "PRESSED" : "Released")}");
        GUILayout.Label($"Button 3 (CENTER): {(button3Pressed ? "PRESSED" : "Released")}");
        GUILayout.Label($"Button 4 (MINUS): {(button4Pressed ? "PRESSED" : "Released")}");
        GUILayout.Label($"Current LED: {GetLEDName(currentLED)}");

        GUILayout.Space(10);

        // Manual LED control buttons
        if (GUILayout.Button("Set LED: Blue"))
        {
            currentLED = FalconBridge.LED.BLUE;
            FalconBridge.SetLEDStatus(currentLED);
        }
        if (GUILayout.Button("Set LED: Green"))
        {
            currentLED = FalconBridge.LED.GREEN;
            FalconBridge.SetLEDStatus(currentLED);
        }
        if (GUILayout.Button("Set LED: Amber"))
        {
            currentLED = FalconBridge.LED.AMBER;
            FalconBridge.SetLEDStatus(currentLED);
        }
        if (GUILayout.Button("Set LED: Red"))
        {
            currentLED = FalconBridge.LED.RED;
            FalconBridge.SetLEDStatus(currentLED);
        }
        if (GUILayout.Button("Set LED: Off"))
        {
            currentLED = FalconBridge.LED.OFF;
            FalconBridge.SetLEDStatus(currentLED);
        }

        GUILayout.EndArea();
    }

    private string GetLEDName(int ledMask)
    {
        if (ledMask == FalconBridge.LED.OFF) return "OFF";
        if (ledMask == FalconBridge.LED.BLUE) return "BLUE";
        if (ledMask == FalconBridge.LED.GREEN) return "GREEN";
        if (ledMask == FalconBridge.LED.AMBER) return "AMBER";
        if (ledMask == FalconBridge.LED.RED) return "RED";
        if (ledMask == (FalconBridge.LED.BLUE | FalconBridge.LED.GREEN)) return "CYAN";
        if (ledMask == (FalconBridge.LED.RED | FalconBridge.LED.BLUE)) return "MAGENTA";
        if (ledMask == (FalconBridge.LED.RED | FalconBridge.LED.GREEN)) return "YELLOW";
        return $"Custom (0x{ledMask:X})";
    }
}
