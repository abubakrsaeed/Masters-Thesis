// Copyright (c) Meta Platforms, Inc. and affiliates.

using Meta.XR.Samples;
using UnityEngine;

namespace PassthroughCameraSamples.MultiObjectDetection
{
    [MetaCodeSample("PassthroughCameraApiSamples-MultiObjectDetection")]
    public class DetectionSpawnMarkerAnim : MonoBehaviour
    {
        [SerializeField] private Vector3 m_anglesSpeed = new(20.0f, 40.0f, 60.0f);
        [SerializeField] private Transform m_model;
        [SerializeField] private TextMesh m_textModel;
        [SerializeField] private Transform m_textEntity;

        private Vector3 m_angles;
        private OVRCameraRig m_camera;
		private string m_className;

        private void Update()
        {
            m_angles.x = AddAngle(m_angles.x, m_anglesSpeed.x * Time.deltaTime);
            m_angles.y = AddAngle(m_angles.y, m_anglesSpeed.y * Time.deltaTime);
            m_angles.z = AddAngle(m_angles.z, m_anglesSpeed.z * Time.deltaTime);

            m_model.rotation = Quaternion.Euler(m_angles);

            if (!m_camera)
            {
                m_camera = FindFirstObjectByType<OVRCameraRig>();
            }
            else
            {
                m_textEntity.gameObject.transform.LookAt(m_camera.centerEyeAnchor);
				// Update label with real-time depth (distance from camera to this marker)
				var depthMeters = Vector3.Distance(m_camera.centerEyeAnchor.position, transform.position);
				m_textModel.text = string.IsNullOrEmpty(m_className) ? $"{depthMeters:0.00}m" : $"{m_className} {depthMeters:0.00}m";
            }
        }

        private float AddAngle(float value, float toAdd)
        {
            value += toAdd;
            if (value > 360.0f)
            {
                value -= 360.0f;
            }

            if (value < 0.0f)
            {
                value = 360.0f - value;
            }

            return value;
        }

        public void SetYoloClassName(string name)
        {
			m_className = name;
			m_textModel.text = name;
        }

        public string GetYoloClassName()
        {
			return string.IsNullOrEmpty(m_className) ? m_textModel.text : m_className;
        }
    }
}
