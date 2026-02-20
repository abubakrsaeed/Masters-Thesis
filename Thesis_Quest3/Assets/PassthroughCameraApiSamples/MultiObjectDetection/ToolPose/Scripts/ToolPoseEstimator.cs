using System.Collections.Generic;
using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	/// <summary>
	/// Estimates the tool pose each frame by matching detected sphere world positions to model points.
	/// Uses a lightweight point-to-point ICP to establish correspondences and then solves with RigidPoseSolver.
	/// </summary>
	public class ToolPoseEstimator : MonoBehaviour
	{
		[SerializeField] private ToolDefinition m_tool;
		[SerializeField] private SentisInferenceUiManager m_uiInference;
		[SerializeField] private Transform m_poseTarget;

		[Tooltip("Max ICP iterations per frame.")]
		[Range(1, 128)] public int MaxIterations = 20;
		[Tooltip("Convergence epsilon (meters).")]
		[Range(1e-5f, 1e-2f)] public float Epsilon = 1e-4f;
		[SerializeField, Tooltip("Static rotation offset applied to the solved pose (Euler, degrees).")]
		private Vector3 m_rotationOffsetEuler = Vector3.zero;
		[SerializeField, Tooltip("If true, applies every new solve immediately (no rejection/low-pass). Use to debug stuck overlay.")]
		private bool m_disableStabilization = false;
		[SerializeField, Tooltip("RMS threshold to reject noisy solves (meters). Higher = less likely to freeze.")]
		private float m_rmsReject = 0.2f;
		[SerializeField, Tooltip("Log pose application/rejection for debugging.")]
		private bool m_logDebug = false;
		[SerializeField, Tooltip("Local-space position offset applied after solve (meters). Useful to nudge overlay up/down/sideways.")]
		private Vector3 m_localPositionOffset = Vector3.zero;
		[SerializeField, Tooltip("Enable simple Kalman filtering on position (per-axis) for additional smoothing.")]
		private bool m_useKalmanFilter = false;
		[SerializeField, Tooltip("Kalman process noise (Q) per axis. Higher = more responsive, lower = smoother.")]
		private float m_kalmanProcessNoise = 1e-4f;
		[SerializeField, Tooltip("Kalman measurement noise (R) per axis. Higher = smoother, lower = more responsive.")]
		private float m_kalmanMeasurementNoise = 1e-2f;
		[SerializeField, Tooltip("Clamp maximum on-screen pixel delta per frame for the pose target (<=0 disables).")]
		private float m_maxScreenDeltaPixels = 0f;
		[Header("Attached 3D Object")]
		[SerializeField, Tooltip("Prefab to attach to the solved tool pose (e.g., cube). Parent is the pose target.")]
		private GameObject m_toolAttachmentPrefab;
		[SerializeField, Tooltip("Optional custom anchor. If null, the attachment is parented to the pose target.")]
		private Transform m_attachmentAnchor;
		[SerializeField, Tooltip("Local position offset for the attachment relative to its parent.")]
		private Vector3 m_attachmentLocalPosition = Vector3.zero;
		[SerializeField, Tooltip("Local rotation offset (Euler) for the attachment relative to its parent.")]
		private Vector3 m_attachmentLocalEuler = Vector3.zero;
		[SerializeField, Tooltip("Local scale for the attachment.")]
		private Vector3 m_attachmentLocalScale = Vector3.one;

		public bool HasPose { get; private set; }
		public Matrix4x4 WorldFromTool { get; private set; } = Matrix4x4.identity;
		public float LastRmsError { get; private set; } = float.PositiveInfinity;

		private Matrix4x4 m_prevPose = Matrix4x4.identity;
		private bool m_hasPrev;
		private Vector3 m_kalmanState;
		private Vector3 m_kalmanError = Vector3.one;
		private bool m_kalmanInitialized;
		private GameObject m_attachmentInstance;

		private void Reset()
		{
			m_poseTarget = transform;
		}

		private void Awake()
		{
			EnsureAttachment();
		}

		private void Update()
		{
			if (m_tool == null || m_uiInference == null || m_tool.ModelPointsLocal == null || m_tool.ModelPointsLocal.Length == 0)
			{
				HasPose = false;
				return;
			}

			var observed = GatherObservedPoints(m_tool.TargetClassName);
			if (observed.Count < Mathf.Max(1, m_tool.MinVisible))
			{
				HasPose = false;
				return;
			}

			if (TryEstimatePose(observed, m_tool.ModelPointsLocal, out var pose, out var rms))
			{
				if (!m_disableStabilization)
				{
					// Reject noisy updates and hold last pose if available
					if (m_hasPrev && rms > m_rmsReject)
					{
						if (m_logDebug) Debug.Log($"[ToolPose] Rejected RMS {rms:F4} > {m_rmsReject:F4}, holding last pose");
						pose = m_prevPose;
						rms = LastRmsError;
					}
					// Low-pass
					if (m_hasPrev)
					{
						var posPrev = m_prevPose.GetColumn(3);
						var rotPrev = m_prevPose.rotation;
						var posNew = pose.GetColumn(3);
						var rotNew = pose.rotation;
						var pos = Vector3.Lerp(posNew, posPrev, m_tool.LowpassPosition);
						var rot = Quaternion.Slerp(rotNew, rotPrev, m_tool.LowpassRotation);
						pose = Matrix4x4.TRS(pos, rot, Vector3.one);
					}
				}
				// Apply static rotation offset for visualization/alignment
				if (m_rotationOffsetEuler != Vector3.zero)
				{
					var rotOffset = Quaternion.Euler(m_rotationOffsetEuler);
					var pos = pose.GetPosition();
					var rot = rotOffset * pose.rotation;
					pose = Matrix4x4.TRS(pos, rot, Vector3.one);
				}

				// Apply local-space position offset (after rotation) to nudge overlay placement
				if (m_localPositionOffset != Vector3.zero)
				{
					var pos = pose.GetPosition() + pose.rotation * m_localPositionOffset;
					pose = Matrix4x4.TRS(pos, pose.rotation, Vector3.one);
				}

				// Optional Kalman filter on position for additional smoothing
				if (m_useKalmanFilter)
				{
					var filteredPos = ApplyKalman(pose.GetPosition());
					pose = Matrix4x4.TRS(filteredPos, pose.rotation, Vector3.one);
				}

				// Clamp on-screen pixel delta to avoid large jumps per frame (visual stabilization only)
				if (m_maxScreenDeltaPixels > 0f && m_hasPrev)
				{
					var clampedPos = ClampScreenDelta(pose.GetPosition(), m_prevPose, m_maxScreenDeltaPixels);
					pose = Matrix4x4.TRS(clampedPos, pose.rotation, Vector3.one);
				}
				m_prevPose = pose;
				m_hasPrev = true;

				WorldFromTool = pose;
				LastRmsError = rms;
				HasPose = true;
				EnsureAttachment();
				if (m_attachmentInstance != null)
				{
					m_attachmentInstance.SetActive(true);
				}

				if (m_poseTarget != null)
				{
					m_poseTarget.SetPositionAndRotation(pose.GetPosition(), pose.rotation);
				}
				if (m_logDebug)
				{
					Debug.Log($"[ToolPose] Applied pose RMS={rms:F4} pos={pose.GetPosition()} rot={pose.rotation.eulerAngles}");
				}
			}
			else
			{
				HasPose = false;
				if (m_attachmentInstance != null)
				{
					m_attachmentInstance.SetActive(false);
				}
				if (m_logDebug) Debug.Log("[ToolPose] Solve failed (insufficient matches)");
			}
		}

		private List<Vector3> GatherObservedPoints(string className)
		{
			var points = new List<Vector3>();
			foreach (var b in m_uiInference.BoxDrawn)
			{
				if (b.WorldPos.HasValue && b.ClassName == className)
				{
					points.Add(b.WorldPos.Value);
				}
			}
			return points;
		}

		private bool TryEstimatePose(List<Vector3> observedWorld, Vector3[] modelLocal, out Matrix4x4 pose, out float rms)
		{
			pose = Matrix4x4.identity;
			rms = float.PositiveInfinity;
			int mCount = modelLocal.Length;
			int oCount = observedWorld.Count;
			if (mCount == 0 || oCount == 0)
			{
				return false;
			}

			// Distance-map search to stabilize correspondences (small n so brute force is fine)
			if (!FindBestAssignment(modelLocal, observedWorld, Mathf.Max(1, m_tool.MinVisible), out var bestModelIdx, out var bestObsIdx))
			{
				return false;
			}

			var matchedModel = new List<Vector3>(bestModelIdx.Count);
			var matchedObserved = new List<Vector3>(bestModelIdx.Count);
			for (int i = 0; i < bestModelIdx.Count; i++)
			{
				matchedModel.Add(modelLocal[bestModelIdx[i]]);
				matchedObserved.Add(observedWorld[bestObsIdx[i]]);
			}

			bool okSolve = RigidPoseSolver.TrySolve(matchedModel, matchedObserved, out var Tsolve, out var rmsSolve);
			if (!okSolve)
			{
				return false;
			}
			pose = Tsolve;
			rms = rmsSolve;
			return true;
		}

		private Vector3 ApplyKalman(Vector3 measurement)
		{
			if (!m_kalmanInitialized)
			{
				m_kalmanState = measurement;
				m_kalmanError = Vector3.one;
				m_kalmanInitialized = true;
				return measurement;
			}

			m_kalmanState.x = Kalman1D(measurement.x, ref m_kalmanError.x);
			m_kalmanState.y = Kalman1D(measurement.y, ref m_kalmanError.y);
			m_kalmanState.z = Kalman1D(measurement.z, ref m_kalmanError.z);
			return m_kalmanState;
		}

		private float Kalman1D(float z, ref float p)
		{
			// Predict
			float pPri = p + m_kalmanProcessNoise;
			// Update
			float k = pPri / (pPri + m_kalmanMeasurementNoise);
			float xPost = m_kalmanState.x + k * (z - m_kalmanState.x);
			p = (1f - k) * pPri;
			return xPost;
		}

		private Vector3 ClampScreenDelta(Vector3 currentPos, Matrix4x4 prevPose, float maxPixels)
		{
			var cam = Camera.main;
			if (cam == null) return currentPos;
			var prevPos = prevPose.GetPosition();
			var prevScreen = cam.WorldToScreenPoint(prevPos);
			var currScreen = cam.WorldToScreenPoint(currentPos);
			if (prevScreen.z <= 0f || currScreen.z <= 0f) return currentPos;

			var delta = currScreen - prevScreen;
			var mag = delta.magnitude;
			if (mag <= maxPixels || mag <= Mathf.Epsilon) return currentPos;

			float scale = maxPixels / mag;
			var clampedScreen = prevScreen + delta * scale;
			// Preserve depth by reusing current screen Z
			clampedScreen.z = currScreen.z;
			return cam.ScreenToWorldPoint(clampedScreen);
		}

		private void EnsureAttachment()
		{
			if (m_toolAttachmentPrefab == null || m_attachmentInstance != null) return;
			var parent = m_attachmentAnchor != null ? m_attachmentAnchor : m_poseTarget != null ? m_poseTarget : transform;
			m_attachmentInstance = Instantiate(m_toolAttachmentPrefab, parent);
			m_attachmentInstance.transform.localPosition = m_attachmentLocalPosition;
			m_attachmentInstance.transform.localRotation = Quaternion.Euler(m_attachmentLocalEuler);
			m_attachmentInstance.transform.localScale = m_attachmentLocalScale;
			m_attachmentInstance.SetActive(false);
		}

		// Brute-force best mapping using pairwise distance error
		private bool FindBestAssignment(Vector3[] model, List<Vector3> observed, int minVisible, out List<int> bestModelIdx, out List<int> bestObsIdx)
		{
			bestModelIdx = null;
			bestObsIdx = null;
			int mCount = model.Length;
			int oCount = observed.Count;
			if (mCount == 0 || oCount == 0) return false;
			int maxUse = Mathf.Min(mCount, oCount);
			float bestCost = float.PositiveInfinity;

			// Precompute model distances
			float[,] md = new float[mCount, mCount];
			for (int i = 0; i < mCount; i++)
				for (int j = i + 1; j < mCount; j++)
				{
					md[i, j] = md[j, i] = Vector3.Distance(model[i], model[j]);
				}

			// Precompute observed distances
			float[,] od = new float[oCount, oCount];
			for (int i = 0; i < oCount; i++)
				for (int j = i + 1; j < oCount; j++)
				{
					od[i, j] = od[j, i] = Vector3.Distance(observed[i], observed[j]);
				}

			// Enumerate k points used
			for (int k = minVisible; k <= maxUse; k++)
			{
				// Combinations of model indices of size k
				var mComb = new List<List<int>>();
				BuildCombinations(mCount, k, 0, new List<int>(), mComb);
				foreach (var mSubset in mComb)
				{
					// Permutations of observed indices of size k
					var oPerms = new List<List<int>>();
					BuildPermutations(oCount, k, new List<int>(), new bool[oCount], oPerms);
					foreach (var oPerm in oPerms)
					{
						float cost = 0f;
						for (int a = 0; a < k; a++)
						{
							for (int b = a + 1; b < k; b++)
							{
								float diff = Mathf.Abs(od[oPerm[a], oPerm[b]] - md[mSubset[a], mSubset[b]]);
								cost += diff;
							}
						}
						if (cost < bestCost)
						{
							bestCost = cost;
							bestModelIdx = new List<int>(mSubset);
							bestObsIdx = new List<int>(oPerm);
						}
					}
				}
			}

			return bestModelIdx != null && bestModelIdx.Count >= minVisible;
		}

		private void BuildCombinations(int n, int k, int start, List<int> current, List<List<int>> output)
		{
			if (current.Count == k)
			{
				output.Add(new List<int>(current));
				return;
			}
			for (int i = start; i <= n - (k - current.Count); i++)
			{
				current.Add(i);
				BuildCombinations(n, k, i + 1, current, output);
				current.RemoveAt(current.Count - 1);
			}
		}

		private void BuildPermutations(int n, int k, List<int> current, bool[] used, List<List<int>> output)
		{
			if (current.Count == k)
			{
				output.Add(new List<int>(current));
				return;
			}
			for (int i = 0; i < n; i++)
			{
				if (used[i]) continue;
				used[i] = true;
				current.Add(i);
				BuildPermutations(n, k, current, used, output);
				current.RemoveAt(current.Count - 1);
				used[i] = false;
			}
		}
	}

	internal static class MatrixExtensions
	{
		public static Vector3 GetPosition(this Matrix4x4 m) => new Vector3(m.m03, m.m13, m.m23);
	}
}


