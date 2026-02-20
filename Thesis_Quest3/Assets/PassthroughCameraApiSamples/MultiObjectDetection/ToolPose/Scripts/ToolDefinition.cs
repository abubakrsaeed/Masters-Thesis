using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection.ToolPose
{
	/// <summary>
	/// Defines a tool by its sphere centers in local coordinates.
	/// </summary>
	public class ToolDefinition : MonoBehaviour
	{
		[Tooltip("Identifier for this tool instance.")]
		public string Identifier = "Tool";

		[Tooltip("Class name of detections to use (e.g., 'sphere').")]
		public string TargetClassName = "sphere";

		[Tooltip("Sphere centers in the tool's local frame (meters).")]
		public Vector3[] ModelPointsLocal = new Vector3[3];

		[Tooltip("Minimum visible spheres needed to solve pose.")]
		[Range(1, 32)]
		public int MinVisible = 3;

		[Tooltip("Greedy matching distance tolerance (meters).")]
		[Range(0.001f, 0.2f)]
		public float MatchToleranceMeters = 0.03f;

		[Tooltip("Low-pass factor for position (0 = no filter, 1 = frozen).")]
		[Range(0, 1)]
		public float LowpassPosition = 0.2f;

		[Tooltip("Low-pass factor for rotation (0 = no filter, 1 = frozen).")]
		[Range(0, 1)]
		public float LowpassRotation = 0.2f;
	}
}


