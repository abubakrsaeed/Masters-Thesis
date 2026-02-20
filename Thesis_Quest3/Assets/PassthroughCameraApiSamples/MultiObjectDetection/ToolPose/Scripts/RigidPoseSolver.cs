using System.Collections.Generic;
using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	/// <summary>
	/// Computes the rigid transform (R,t) that best aligns model points to observed points.
	/// Uses Davenport/Horn quaternion method with power iteration (no SVD dependency).
	/// Returns a Matrix4x4 with rotation and translation (scale = 1).
	/// </summary>
	public static class RigidPoseSolver
	{
		public static bool TrySolve(IReadOnlyList<Vector3> modelLocal, IReadOnlyList<Vector3> observedWorld, out Matrix4x4 poseWorldFromModel, out float rmsError)
		{
			poseWorldFromModel = Matrix4x4.identity;
			rmsError = 0f;
			int n = modelLocal.Count;
			if (n != observedWorld.Count || n < 1)
			{
				return false;
			}

			if (n == 1)
			{
				var tOnly = observedWorld[0] - modelLocal[0];
				poseWorldFromModel = Matrix4x4.TRS(tOnly, Quaternion.identity, Vector3.one);
				return true;
			}

			if (n == 2)
			{
				// Two points: solve translation and the rotation that aligns the segment directions.
				var m0 = modelLocal[0];
				var m1 = modelLocal[1];
				var o0 = observedWorld[0];
				var o1 = observedWorld[1];
				var vm = (m1 - m0).normalized;
				var vo = (o1 - o0).normalized;
				Quaternion rot = Quaternion.FromToRotation(vm, vo);
				Vector3 t = o0 - (rot * m0);
				poseWorldFromModel = Matrix4x4.TRS(t, rot, Vector3.one);
				rmsError = ((poseWorldFromModel.MultiplyPoint3x4(m0) - o0).magnitude + (poseWorldFromModel.MultiplyPoint3x4(m1) - o1).magnitude) * 0.5f;
				return true;
			}

			// N >= 3
			Vector3 centroidModel = Vector3.zero;
			Vector3 centroidObserved = Vector3.zero;
			for (int i = 0; i < n; i++)
			{
				centroidModel += modelLocal[i];
				centroidObserved += observedWorld[i];
			}
			centroidModel /= n;
			centroidObserved /= n;

			// Covariance H = sum m' * o'^T
			float h00 = 0, h01 = 0, h02 = 0;
			float h10 = 0, h11 = 0, h12 = 0;
			float h20 = 0, h21 = 0, h22 = 0;
			for (int i = 0; i < n; i++)
			{
				var m = modelLocal[i] - centroidModel;
				var o = observedWorld[i] - centroidObserved;
				h00 += m.x * o.x; h01 += m.x * o.y; h02 += m.x * o.z;
				h10 += m.y * o.x; h11 += m.y * o.y; h12 += m.y * o.z;
				h20 += m.z * o.x; h21 += m.z * o.y; h22 += m.z * o.z;
			}

			float trace = h00 + h11 + h22;
			float k00 = trace;
			float k01 = h21 - h12, k02 = h02 - h20, k03 = h10 - h01;
			float k11 = h00 - h11 - h22, k12 = h01 + h10,   k13 = h02 + h20;
			float k22 = -h00 + h11 - h22, k23 = h12 + h21;
			float k33 = -h00 - h11 + h22;

			// Power iteration to get dominant eigenvector (w,x,y,z)
			Vector4 q = new Vector4(1, 0, 0, 0);
			for (int it = 0; it < 32; it++)
			{
				Vector4 nq = new Vector4(
					k00 * q.x + k01 * q.y + k02 * q.z + k03 * q.w,
					k01 * q.x + k11 * q.y + k12 * q.z + k13 * q.w,
					k02 * q.x + k12 * q.y + k22 * q.z + k23 * q.w,
					k03 * q.x + k13 * q.y + k23 * q.z + k33 * q.w
				);
				float norm = Mathf.Sqrt(nq.x * nq.x + nq.y * nq.y + nq.z * nq.z + nq.w * nq.w);
				if (norm < 1e-9f) break;
				nq /= norm;
				if ((nq - q).sqrMagnitude < 1e-12f) { q = nq; break; }
				q = nq;
			}
			Quaternion rotWorldFromModel = new Quaternion(q.y, q.z, q.w, q.x);
			Vector3 tWorldFromModel = centroidObserved - rotWorldFromModel * centroidModel;
			poseWorldFromModel = Matrix4x4.TRS(tWorldFromModel, rotWorldFromModel, Vector3.one);

			float se = 0f;
			for (int i = 0; i < n; i++)
			{
				Vector3 predicted = poseWorldFromModel.MultiplyPoint3x4(modelLocal[i]);
				se += (predicted - observedWorld[i]).sqrMagnitude;
			}
			rmsError = Mathf.Sqrt(se / n);
			return true;
		}
	}
}


