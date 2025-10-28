using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Records and replays the Novint Falcon grip trajectory using the device buttons.
/// Controls:
/// - Hold button 3 (CENTER) to start recording; release to stop. LED turns red while recording.
/// - Press button 1 (PLUS/Right) to start playback; press again to stop. LED turns blue while playing.
/// The recorded path loops continuously during playback.
///
/// This script assumes the Falcon device is already initialised elsewhere (e.g. FalconController).
/// Attach it to a GameObject to monitor recording/playback status and optionally visualise the replay.
/// </summary>
public class FalconRecordPlayback : MonoBehaviour
{
    [Header("Visualisation")]
    [Tooltip("Optional transform used to show the currently replayed position in the scene.")]
    public Transform playbackCursor;

    [Tooltip("Scale factor applied when mapping Falcon metres to Unity units for the playback cursor.")]
    public float playbackCursorScale = 1.0f;

    [Header("Recording")]
    [Tooltip("Minimum time (seconds) between recorded samples when movement is minimal.")]
    public float recordMinInterval = 0.01f;

    [Tooltip("Minimum distance change (metres) between samples while recording to reduce noise.")]
    public float recordPositionEpsilon = 0.0005f;

    [Header("Integration")]
    [Tooltip("Automatically disable FalconController haptics whenever playback is not active.")]
    public bool manageFalconControllerHaptics = true;

    private enum FalconState
    {
        Idle,
        Recording,
        Playing
    }

    [Serializable]
    private struct Sample
    {
        public float time;
        public Vector3 position;

        public Sample(float time, Vector3 position)
        {
            this.time = time;
            this.position = position;
        }
    }

    private readonly List<Sample> samples = new List<Sample>();
    private FalconState state = FalconState.Idle;
    private float recordStartTime;
    private float totalDuration;
    private float playbackTimer;
    private Vector3 lastRecordedPosition;
    private float lastSampleAbsoluteTime;
    private bool button1Prev;
    private bool button3Prev;
    private FalconController cachedController;
    private bool controllerOriginalHaptics;
    private bool controllerCaptured;
    private bool forceActive;

    void OnEnable()
    {
        CacheFalconController();
        if (cachedController != null && manageFalconControllerHaptics)
        {
            controllerOriginalHaptics = cachedController.enableHaptics;
            controllerCaptured = true;
            cachedController.enableHaptics = false;
        }

        FalconBridge.SetLEDStatus(FalconBridge.LED.GREEN);
        SetPlaybackForceActive(false);
    }

    void OnDisable()
    {
        StopPlayback();
        if (controllerCaptured && cachedController != null && manageFalconControllerHaptics)
        {
            cachedController.enableHaptics = controllerOriginalHaptics;
        }
        controllerCaptured = false;
        FalconBridge.SetLEDStatus(FalconBridge.LED.GREEN);
        FalconBridge.ClearContact();
        FalconBridge.EnablePositionControl(false);
        forceActive = false;
    }

    void Update()
    {
        if (!FalconBridge.IsCalibrated())
        {
            return;
        }

        Vector3 currentPosition;
        if (!FalconBridge.GetToolPosition(out currentPosition))
        {
            return;
        }

        bool button1, button3;
        if (!FalconBridge.GetButtonStates(out button1, out _, out button3, out _))
        {
            return;
        }

        HandleRecordingInput(button3, currentPosition);
        HandlePlaybackInput(button1);

        if (state != FalconState.Playing)
        {
            SetPlaybackForceActive(false);
        }

        if (state == FalconState.Recording)
        {
            AppendSample(Time.time, currentPosition);
        }
        else if (state == FalconState.Playing)
        {
            UpdatePlayback(Time.deltaTime);
        }

        button1Prev = button1;
        button3Prev = button3;
    }

    private void HandleRecordingInput(bool button3, Vector3 currentPosition)
    {
        if (button3 && !button3Prev)
        {
            StartRecording(currentPosition);
        }
        else if (!button3 && button3Prev && state == FalconState.Recording)
        {
            StopRecording();
        }
    }

    private void HandlePlaybackInput(bool button1)
    {
        if (!button1 || button1Prev)
        {
            return;
        }

        if (state == FalconState.Playing)
        {
            StopPlayback();
        }
        else if (samples.Count > 0)
        {
            StartPlayback();
        }
        else
        {
            Debug.LogWarning("FalconRecordPlayback: No samples recorded yet.");
        }
    }

    private void StartRecording(Vector3 currentPosition)
    {
        StopPlayback();

        samples.Clear();
        recordStartTime = Time.time;
        lastSampleAbsoluteTime = recordStartTime;
        lastRecordedPosition = currentPosition;
        AppendSample(recordStartTime, currentPosition);

        state = FalconState.Recording;
        FalconBridge.SetLEDStatus(FalconBridge.LED.RED);
        SetPlaybackForceActive(false);
        FalconBridge.ClearContact();
        Debug.Log("FalconRecordPlayback: Recording started.");
    }

    private void StopRecording()
    {
        if (state != FalconState.Recording)
        {
            return;
        }

        totalDuration = samples.Count > 0 ? samples[samples.Count - 1].time : 0f;
        state = FalconState.Idle;
        FalconBridge.SetLEDStatus(FalconBridge.LED.GREEN);
        SetPlaybackForceActive(false);
        FalconBridge.ClearContact();
        Debug.LogFormat("FalconRecordPlayback: Recording stopped. Captured {0} samples over {1:F2}s.", samples.Count, totalDuration);
    }

    private void StartPlayback()
    {
        if (samples.Count == 0)
        {
            return;
        }

        playbackTimer = 0f;
        state = FalconState.Playing;
        SetPlaybackForceActive(true);
        FalconBridge.SetLEDStatus(FalconBridge.LED.BLUE);
        Debug.Log("FalconRecordPlayback: Playback started.");
    }

    private void StopPlayback()
    {
        if (state != FalconState.Playing)
        {
            SetPlaybackForceActive(false);
            FalconBridge.ClearContact();
            return;
        }

        state = FalconState.Idle;
        playbackTimer = 0f;
        SetPlaybackForceActive(false);
        FalconBridge.ClearContact();
        FalconBridge.SetLEDStatus(FalconBridge.LED.GREEN);
        Debug.Log("FalconRecordPlayback: Playback stopped.");
    }

    private void AppendSample(float timestamp, Vector3 position)
    {
        float elapsedSinceLast = timestamp - lastSampleAbsoluteTime;
        bool belowDistanceThreshold = Vector3.Distance(position, lastRecordedPosition) < recordPositionEpsilon;
        if (samples.Count > 0 && belowDistanceThreshold && elapsedSinceLast < recordMinInterval)
        {
            return;
        }

        float relativeTime = timestamp - recordStartTime;
        samples.Add(new Sample(relativeTime, position));
        lastRecordedPosition = position;
        lastSampleAbsoluteTime = timestamp;
    }

    private void UpdatePlayback(float deltaTime)
    {
        if (samples.Count == 0)
        {
            StopPlayback();
            return;
        }

        if (samples.Count == 1 || totalDuration <= 0f)
        {
            Vector3 onlyPosition = samples[0].position;
            FalconBridge.SetTargetPosition(onlyPosition);
            UpdatePlaybackCursor(onlyPosition);
            return;
        }

        playbackTimer = (playbackTimer + deltaTime) % totalDuration;

        Sample prevSample = samples[samples.Count - 1];
        Sample nextSample = samples[0];

        for (int i = 0; i < samples.Count; i++)
        {
            nextSample = samples[i];
            if (nextSample.time >= playbackTimer)
            {
                prevSample = (i == 0) ? samples[samples.Count - 1] : samples[i - 1];
                break;
            }

            if (i == samples.Count - 1)
            {
                prevSample = samples[samples.Count - 1];
                nextSample = samples[0];
            }
        }

        float deltaTimeSamples = nextSample.time - prevSample.time;
        float lerpT;
        if (Mathf.Approximately(deltaTimeSamples, 0f))
        {
            lerpT = 0f;
        }
        else
        {
            float startTime = prevSample.time;
            float endTime = (prevSample.time <= nextSample.time) ? nextSample.time : prevSample.time + totalDuration;
            float targetTime = playbackTimer;
            if (targetTime < prevSample.time)
            {
                targetTime += totalDuration;
            }

            lerpT = Mathf.InverseLerp(startTime, endTime, targetTime);
        }

        Vector3 targetPosition = Vector3.Lerp(prevSample.position, nextSample.position, lerpT);
        FalconBridge.SetTargetPosition(targetPosition);
        UpdatePlaybackCursor(targetPosition);
    }

    private void UpdatePlaybackCursor(Vector3 targetPosition)
    {
        if (playbackCursor == null)
        {
            return;
        }

        Vector3 unityPosition = new Vector3(targetPosition.x, targetPosition.y, -targetPosition.z);
        playbackCursor.position = unityPosition * playbackCursorScale;
    }

    void OnGUI()
    {
        GUILayout.BeginArea(new Rect(10f, 220f, 350f, 140f), GUI.skin.box);
        GUILayout.Label(string.Format("Falcon Record/Playback: {0}", state));
        GUILayout.Label(string.Format("Samples: {0}", samples.Count));
        if (state == FalconState.Playing)
        {
            GUILayout.Label(string.Format("Playback Time: {0:F2}s / {1:F2}s", playbackTimer, totalDuration));
        }
        else
        {
            GUILayout.Label(string.Format("Total Duration: {0:F2}s", totalDuration));
        }
        GUILayout.Label("Controls: Hold Button3 to record, press Button1 to toggle playback.");
        GUILayout.EndArea();
    }

    private void CacheFalconController()
    {
        if (cachedController == null)
        {
            cachedController = GetComponent<FalconController>();
        }
    }

    private void SetPlaybackForceActive(bool enablePositionControl)
    {
        if (forceActive == enablePositionControl)
        {
            if (!enablePositionControl)
            {
                FalconBridge.ClearContact();
            }
            return;
        }

        forceActive = enablePositionControl;
        FalconBridge.EnablePositionControl(enablePositionControl);
        if (!enablePositionControl)
        {
            FalconBridge.ClearContact();
        }
    }
}
