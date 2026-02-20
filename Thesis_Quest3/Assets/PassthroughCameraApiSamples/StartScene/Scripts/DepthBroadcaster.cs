using System.Net.Sockets;
using System.Text;
using Meta.XR;
// We need this namespace to talk to the script in your screenshot
using Meta.XR.EnvironmentDepth;
using UnityEngine;

public class DepthBroadcaster : MonoBehaviour
{
    [Header("Network Settings")]
    public string targetIP = "192.168.0.103"; // <-- YOUR LAPTOP IP
    public int targetPort = 5006;

    [Header("Meta Depth Settings")]
    // DRAG the GameObject with 'EnvironmentRaycastManager' here in the Inspector!
    public EnvironmentRaycastManager metaRaycaster;

    private UdpClient udpClient;
    private float timer;

    void Start()
    {
        udpClient = new UdpClient();
    }

    void Update()
    {
        timer += Time.deltaTime;
        if (timer < 0.033f) return; // limit to 30fps
        timer = 0;

        float z_distance = 0.0f;

        // Safety check: Did you drag the script in?
        if (metaRaycaster != null)
        {
            // Create a ray pointing forward from THIS object (CenterEyeAnchor)
            Ray ray = new Ray(transform.position, transform.forward);

            // Use Meta's special raycast struct
            EnvironmentRaycastHit hitInfo;

            // Perform the raycast
            if (metaRaycaster.Raycast(ray, out hitInfo, 3.0f)) // Max 3 meters
            {
                // Calculate real distance
                z_distance = Vector3.Distance(transform.position, hitInfo.point);
            }
        }

        // Send to Python
        if (z_distance > 0.1f)
        {
            try
            {
                string msg = z_distance.ToString("F3");
                byte[] data = Encoding.UTF8.GetBytes(msg);
                udpClient.Send(data, data.Length, targetIP, targetPort);
            }
            catch { }
        }
    }
}