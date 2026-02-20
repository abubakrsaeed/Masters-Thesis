using System.Text;
using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	public class DetectionCenterLogger : MonoBehaviour
	{
		[SerializeField] private SentisInferenceUiManager m_uiInference;
		[SerializeField] private string m_targetClassName = "sphere";
		[SerializeField, Tooltip("Seconds between logs.")] private float m_interval = 0.5f;
		[SerializeField, Tooltip("Log pairwise distances as well as positions.")] private bool m_logDistances = true;
		[SerializeField, Tooltip("Limit how many detections to log.")] private int m_maxLog = 8;

		private float m_next;

		private void Update()
		{
			if (m_uiInference == null) return;
			if (Time.unscaledTime < m_next) return;
			m_next = Time.unscaledTime + Mathf.Max(0.05f, m_interval);

			var pts = m_uiInference.BoxDrawn;
			if (pts == null || pts.Count == 0) return;

			var sb = new StringBuilder();
			sb.Append("[DetCenters] ");
			int count = 0;
			for (int i = 0; i < pts.Count && count < m_maxLog; i++)
			{
				if (pts[i].WorldPos.HasValue && pts[i].ClassName == m_targetClassName)
				{
					var p = pts[i].WorldPos.Value;
					sb.Append($"#{i}=({p.x:F3},{p.y:F3},{p.z:F3}) ");
					count++;
				}
			}
			if (count == 0) return;

			if (m_logDistances)
			{
				sb.Append(" | dists: ");
				count = 0;
				for (int i = 0; i < pts.Count && count < m_maxLog; i++)
				{
					if (!pts[i].WorldPos.HasValue || pts[i].ClassName != m_targetClassName) continue;
					for (int j = i + 1; j < pts.Count && count < m_maxLog; j++)
					{
						if (!pts[j].WorldPos.HasValue || pts[j].ClassName != m_targetClassName) continue;
						float d = Vector3.Distance(pts[i].WorldPos.Value, pts[j].WorldPos.Value);
						sb.Append($"({i},{j})={d:F3} ");
						count++;
					}
				}
			}

			Debug.Log(sb.ToString());
		}
	}
}


