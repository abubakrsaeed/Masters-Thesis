using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	public class PoseLogger : MonoBehaviour
	{
		[SerializeField] private ToolPoseEstimator m_estimator;
		[SerializeField, Tooltip("Seconds between logs to avoid spamming logcat.")] private float m_logIntervalSeconds = 0.25f;
		[SerializeField] private bool m_logPositionRotation = true;
		[SerializeField] private bool m_logMatrix = false;
		[SerializeField] private bool m_logRmsError = true;

		private float m_nextLogTime;

		private void Update()
		{
			if (m_estimator == null || !m_estimator.HasPose)
			{
				return;
			}

			if (Time.unscaledTime < m_nextLogTime)
			{
				return;
			}
			m_nextLogTime = Time.unscaledTime + Mathf.Max(0.02f, m_logIntervalSeconds);

			var M = m_estimator.WorldFromTool;
			if (m_logPositionRotation)
			{
				var pos = new Vector3(M.m03, M.m13, M.m23);
				var rot = M.rotation.eulerAngles;
				Debug.Log($"[ToolPose] pos=({pos.x:F3},{pos.y:F3},{pos.z:F3}) eul=({rot.x:F1},{rot.y:F1},{rot.z:F1})");
			}
			if (m_logRmsError)
			{
				Debug.Log($"[ToolPose RMS] {m_estimator.LastRmsError:F4} m");
			}
			if (m_logMatrix)
			{
				Debug.Log($"[ToolPose Matrix]\n{M}");
			}
		}
	}
}


