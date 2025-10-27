using UnityEngine;

public class FalconGripTracker : MonoBehaviour
{
    private Vector3 currentPosition;

    // Update is called once per frame
    void Update()
    {
        bool isCalibrated = FalconBridge.IsCalibrated();

        // Get the current tool position
        if (FalconBridge.GetToolPosition(out currentPosition))
        {
            gameObject.transform.position = new Vector3(currentPosition.x,currentPosition.y,-currentPosition.z);
        }
    }
}
