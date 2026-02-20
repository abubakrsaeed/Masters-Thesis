using UnityEngine;
using PassthroughCameraSamples.MultiObjectDetection.ToolPose;

namespace PassthroughCameraSamples.MultiObjectDetection
{
    /// <summary>
    /// Logs performance/pose metrics at a fixed interval to the Unity Console.
    /// FPS | LAT(ms) | YOLO(ms) | JITTER | POS_X(m) | POS_Z(m) | MOVED_X(mm) | MOVED_Z(mm)
    /// </summary>
    public class PerformanceLogger : MonoBehaviour
    {
        [Header("Sources")]
        [SerializeField] private SentisInferenceRunManager m_runManager;
        [SerializeField] private ToolPoseEstimator m_toolPose;
        [SerializeField, Tooltip("Transform to read pose from; defaults to toolPose.poseTarget or this transform.")]
        private Transform m_poseTarget;

        [Header("Logging")]
        [SerializeField, Tooltip("How often to log (seconds).")]
        private float m_logIntervalSeconds = 0.5f;
        [SerializeField, Tooltip("If true, write to Unity console.")]
        private bool m_logToConsole = true;

        private float m_nextLogTime;
        private Vector3 m_prevPos;
        private bool m_hasPrevPos;

        private Vector3 GetPosePosition()
        {
            if (m_poseTarget != null)
            {
                return m_poseTarget.position;
            }
            if (m_toolPose != null && m_toolPose.HasPose)
            {
                return m_toolPose.WorldFromTool.GetPosition();
            }
            return Vector3.zero;
        }

        private void Start()
        {
            if (m_poseTarget == null && m_toolPose != null)
            {
                m_poseTarget = m_toolPose.transform;
            }
        }

        private void Update()
        {
            if (Time.unscaledTime < m_nextLogTime)
                return;
            m_nextLogTime = Time.unscaledTime + m_logIntervalSeconds;

            float fps = Time.unscaledDeltaTime > 1e-4f ? 1f / Time.unscaledDeltaTime : 0f;
            float latMs = m_runManager != null ? m_runManager.LastEndToEndMs : 0f;
            float yoloMs = m_runManager != null ? m_runManager.LastInferenceMs : 0f;
            float jitter = m_toolPose != null ? m_toolPose.LastRmsError : 0f;

            Vector3 pos = GetPosePosition();
            var cam = Camera.main;
            float depth = 0f;
            float dist = 0f;
            if (cam != null)
            {
                var toTarget = pos - cam.transform.position;
                dist = toTarget.magnitude;
                depth = Vector3.Dot(cam.transform.forward, toTarget);
            }
            Vector3 delta = m_hasPrevPos ? (pos - m_prevPos) : Vector3.zero;
            m_prevPos = pos;
            m_hasPrevPos = true;

            float movedXmm = delta.x * 1000f;
            float movedZmm = delta.z * 1000f;

            if (m_logToConsole)
            {
                Debug.Log($"FPS {fps:0.0} | LAT(ms) {latMs:0.0} | YOLO(ms) {yoloMs:0.0} | JITTER {jitter:0.000} | POS_X(m) {pos.x:0.000} | POS_Z(m) {pos.z:0.000} | DEPTH(m) {depth:0.000} | DIST(m) {dist:0.000} | MOVED_X(mm) {movedXmm:0.0} | MOVED_Z(mm) {movedZmm:0.0}");
            }
        }
    }
}
