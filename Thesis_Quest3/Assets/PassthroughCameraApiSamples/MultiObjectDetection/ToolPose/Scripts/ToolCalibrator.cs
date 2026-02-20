using System.Collections.Generic;
using System.Text;
using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	/// <summary>
	/// One-click calibration: captures current detected sphere world positions and derives a local tool model.
	/// Use the context menu from the component in Play mode: "Calibrate From Current Detections".
	/// The computed model is written to ToolDefinition.ModelPointsLocal and also copied to clipboard.
	/// </summary>
	public class ToolCalibrator : MonoBehaviour
	{
		[SerializeField] private ToolDefinition m_tool;
		[SerializeField] private SentisInferenceUiManager m_uiInference;
		[SerializeField] private string m_targetClassOverride = "";
		[Tooltip("If true, tries to calibrate once on Start (requires >=3 visible spheres).")]
		[SerializeField] private bool m_calibrateOnStart = false;

		private void Start()
		{
			if (m_calibrateOnStart)
			{
				CalibrateRuntime();
			}
		}

		[ContextMenu("Calibrate From Current Detections")]
		private void CalibrateFromDetections()
		{
			if (m_tool == null || m_uiInference == null)
			{
				Debug.LogError($"{nameof(ToolCalibrator)}: Missing references.");
				return;
			}
			string className = string.IsNullOrEmpty(m_targetClassOverride) ? m_tool.TargetClassName : m_targetClassOverride;

			var points = new List<Vector3>();
			foreach (var b in m_uiInference.BoxDrawn)
			{
				if (b.WorldPos.HasValue && b.ClassName == className)
				{
					points.Add(b.WorldPos.Value);
				}
			}
			if (points.Count < 3)
			{
				Debug.LogError($"{nameof(ToolCalibrator)}: Need at least 3 visible spheres to calibrate. Found {points.Count}.");
				return;
			}

			// Choose p0 as any point, p1 as farthest from p0 (defines +X), p2 maximizing area (defines XY plane)
			var p0 = points[0];
			int idx1 = 0;
			float bestD = -1f;
			for (int i = 0; i < points.Count; i++)
			{
				float d = (points[i] - p0).sqrMagnitude;
				if (d > bestD) { bestD = d; idx1 = i; }
			}
			var p1 = points[idx1];

			int idx2 = -1;
			float bestArea = -1f;
			for (int i = 0; i < points.Count; i++)
			{
				if (i == idx1) continue;
				float area = Vector3.Cross(points[i] - p0, p1 - p0).magnitude;
				if (area > bestArea) { bestArea = area; idx2 = i; }
			}
			var p2 = points[idx2];

			// Build orthonormal basis
			var ex = (p1 - p0).normalized;
			var v2 = p2 - p0;
			var e2tmp = v2 - Vector3.Dot(v2, ex) * ex;
			var ey = e2tmp.sqrMagnitude > 1e-12f ? e2tmp.normalized : Vector3.up;
			var ez = Vector3.Cross(ex, ey).normalized;
			// Ensure right-handed
			ey = Vector3.Cross(ez, ex).normalized;

			// Convert world points to local tool frame: local = [ex ey ez]^T * (w - p0)
			var locals = new Vector3[points.Count];
			for (int i = 0; i < points.Count; i++)
			{
				var w = points[i] - p0;
				locals[i] = new Vector3(Vector3.Dot(w, ex), Vector3.Dot(w, ey), Vector3.Dot(w, ez));
			}

			m_tool.ModelPointsLocal = locals;
			// Require at least 3 points for 6DoF solve; allow tracking when fewer than all spheres are visible.
			m_tool.MinVisible = 3;

			// Log and copy for persistence
			var sb = new StringBuilder();
			sb.AppendLine("Calibrated ModelPointsLocal (meters):");
			for (int i = 0; i < locals.Length; i++)
			{
				sb.AppendLine($"{i}: new Vector3({locals[i].x:F4}f, {locals[i].y:F4}f, {locals[i].z:F4}f),");
			}
#if UNITY_EDITOR
			GUIUtility.systemCopyBuffer = sb.ToString();
#endif
			Debug.Log($"{nameof(ToolCalibrator)}: Calibration complete. Copied ModelPointsLocal to clipboard.\n{sb}");
		}

		/// <summary>
		/// Public entry point to run calibration at runtime (e.g., from a UI Button on Meta Quest).
		/// </summary>
		public void CalibrateRuntime()
		{
			CalibrateFromDetections();
		}
	}
}


