using System.Collections.Generic;
using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	public class ToolOverlayVisualizer : MonoBehaviour
	{
		[SerializeField] private SentisInferenceUiManager m_uiInference;
		[SerializeField] private ToolPoseEstimator m_estimator;
		[SerializeField] private string m_targetClassName = "sphere";
		[SerializeField, Range(0.005f, 0.1f)] private float m_markerSize = 0.02f;
		[SerializeField, Range(1, 64)] private int m_maxMarkers = 16;
		[SerializeField] private bool m_showObserved = true;
		[SerializeField] private bool m_showModel = true;
		[SerializeField] private Color m_observedColor = Color.green;
		[SerializeField] private Color m_modelColor = Color.cyan;

		private readonly List<GameObject> m_observedPool = new();
		private readonly List<GameObject> m_modelPool = new();

		private void Update()
		{
			if (m_uiInference == null) return;
			if (m_showObserved) UpdateObserved();
			else SetActiveAll(m_observedPool, false);
			if (m_showModel) UpdateModel();
			else SetActiveAll(m_modelPool, false);
		}

		private void UpdateObserved()
		{
			var positions = new List<Vector3>();
			foreach (var b in m_uiInference.BoxDrawn)
			{
				if (b.WorldPos.HasValue && b.ClassName == m_targetClassName)
				{
					positions.Add(b.WorldPos.Value);
					if (positions.Count >= m_maxMarkers) break;
				}
			}
			EnsurePoolSize(m_observedPool, positions.Count, m_observedColor);
			for (int i = 0; i < m_observedPool.Count; i++)
			{
				bool active = i < positions.Count;
				var go = m_observedPool[i];
				go.SetActive(active);
				if (active)
				{
					go.transform.SetPositionAndRotation(positions[i], Quaternion.identity);
					go.transform.localScale = Vector3.one * m_markerSize;
				}
			}
		}

		private void UpdateModel()
		{
			if (m_estimator == null || !m_estimator.HasPose) { SetActiveAll(m_modelPool, false); return; }
			var tool = GetComponent<ToolDefinition>() ?? m_estimator.GetComponent<ToolDefinition>();
			if (tool == null || tool.ModelPointsLocal == null) { SetActiveAll(m_modelPool, false); return; }

			int count = Mathf.Min(tool.ModelPointsLocal.Length, m_maxMarkers);
			EnsurePoolSize(m_modelPool, count, m_modelColor);
			for (int i = 0; i < m_modelPool.Count; i++)
			{
				bool active = i < count;
				var go = m_modelPool[i];
				go.SetActive(active);
				if (active)
				{
					Vector3 w = m_estimator.WorldFromTool.MultiplyPoint3x4(tool.ModelPointsLocal[i]);
					go.transform.SetPositionAndRotation(w, Quaternion.identity);
					go.transform.localScale = Vector3.one * (m_markerSize * 0.9f);
				}
			}
		}

		private void EnsurePoolSize(List<GameObject> pool, int needed, Color color)
		{
			for (int i = pool.Count; i < needed; i++)
			{
				var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
				s.name = pool == m_observedPool ? "ObservedMarker" : "ModelMarker";
				s.transform.SetParent(transform, false);
				var mr = s.GetComponent<Renderer>();
				if (mr != null)
				{
					mr.material = new Material(Shader.Find("Standard")) { color = color };
				}
				pool.Add(s);
			}
		}

		private static void SetActiveAll(List<GameObject> pool, bool active)
		{
			for (int i = 0; i < pool.Count; i++) if (pool[i] != null) pool[i].SetActive(active);
		}
	}
}


