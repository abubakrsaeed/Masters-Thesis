using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	public class CalibrateOnB : MonoBehaviour
	{
		[SerializeField] private ToolCalibrator m_calibrator;

		private void Update()
		{
			// Oculus/Meta controllers: Button.Two corresponds to B
			if (OVRInput.GetDown(OVRInput.Button.Two) && m_calibrator != null)
			{
				m_calibrator.CalibrateRuntime();
			}
		}
	}
}


