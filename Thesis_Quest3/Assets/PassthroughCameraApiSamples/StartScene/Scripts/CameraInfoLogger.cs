using UnityEngine;
using UnityEngine.InputSystem; // REQUIRED for New Input System

public class CameraInfoLogger : MonoBehaviour
{
    void Update()
    {
        // Check if the Space key was pressed (New Input System syntax)
        if (Keyboard.current != null && Keyboard.current.spaceKey.wasPressedThisFrame)
        {
            Camera cam = GetComponent<Camera>();

            if (cam != null)
            {
                float v_fov = cam.fieldOfView;
                float h_fov = Camera.VerticalToHorizontalFieldOfView(v_fov, cam.aspect);

                Debug.Log("---------------------------------------");
                Debug.Log("?? CAMERA INTRINSICS (Truth Data)");
                Debug.Log("Vertical FOV: " + v_fov);
                Debug.Log("Horizontal FOV (Approx): " + h_fov);
                Debug.Log("Aspect Ratio: " + cam.aspect);
                Debug.Log("Pixel Width: " + cam.pixelWidth);
                Debug.Log("Pixel Height: " + cam.pixelHeight);
                Debug.Log("---------------------------------------");
            }
            else
            {
                Debug.LogError("? No Camera component found on this object!");
            }
        }
    }
}