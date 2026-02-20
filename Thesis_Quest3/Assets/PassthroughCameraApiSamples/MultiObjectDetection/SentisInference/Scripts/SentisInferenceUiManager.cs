// Copyright (c) Meta Platforms, Inc. and affiliates.

using System.Collections.Generic;
using Meta.XR;
using Meta.XR.Samples;
using Unity.InferenceEngine;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.UI;

namespace PassthroughCameraSamples.MultiObjectDetection
{
    [MetaCodeSample("PassthroughCameraApiSamples-MultiObjectDetection")]
    public class SentisInferenceUiManager : MonoBehaviour
    {
        [Header("Placement configureation")]
        [SerializeField] private EnvironmentRayCastSampleManager m_environmentRaycast;
        [SerializeField] private PassthroughCameraAccess m_cameraAccess;

        [Header("UI display references")]
        [SerializeField] private SentisObjectDetectedUiManager m_detectionCanvas;
        [SerializeField] private RawImage m_displayImage;
        [SerializeField] private Sprite m_boxTexture;
        [SerializeField] private Color m_boxColor;
        [SerializeField] private Font m_font;
        [SerializeField] private Color m_fontColor;
        [SerializeField] private int m_fontSize = 80;
        [SerializeField, Tooltip("Maximum number of detections to process/draw per frame (<=0 means no cap).")]
        private int m_maxDetections = 4;
        [SerializeField, Tooltip("If true, face each box toward the camera to avoid edge-on view.")]
        private bool m_faceCamera = true;
        [SerializeField, Tooltip("If true, also draw a 2D sphere overlay at each box center (no 3D placement).")]
        private bool m_draw2dSphereOverlay = false;
        [SerializeField, Tooltip("Sprite used for the 2D sphere overlay. If null, falls back to built-in UISprite.")]
        private Sprite m_sphereOverlaySprite;
        [SerializeField, Tooltip("Overlay size in pixels (width,height).")]
        private Vector2 m_sphereOverlaySize = new(48f, 48f);
        [SerializeField, Tooltip("Color for the 2D sphere overlay.")]
        private Color m_sphereOverlayColor = Color.white;
        [SerializeField, Tooltip("If set, only draw overlays for this class name (leave empty for all).")]
        private string m_sphereOverlayClassFilter = "";
        [SerializeField, Tooltip("If true, draw a small 3D marker at each box center (child of the canvas).")]
        private bool m_draw3dMarkerOverlay = false;
        [SerializeField, Tooltip("Prefab for 3D marker; if null, a simple sphere will be generated.")]
        private GameObject m_marker3dPrefab;
        [SerializeField, Tooltip("Local scale for the 3D marker.")]
        private Vector3 m_marker3dScale = new(0.03f, 0.03f, 0.03f);
        [SerializeField, Tooltip("Tint color for the generated marker (if prefab has a Renderer).")]
        private Color m_marker3dColor = Color.red;
		[SerializeField, Tooltip("If true, estimate center from box width + intrinsics (pinhole), bypassing depth raycast.")]
		private bool m_useProjectiveCenter = false;
		[SerializeField, Tooltip("Scale factor applied to projective depth (z); >1 increases estimated distance.")]
		private float m_projectiveScale = 1f;
		[SerializeField] private bool m_applySphereCenterOffset = false;
		[SerializeField, Tooltip("Sphere radius in meters; used for center estimation/offset (can be negative to flip direction).")]
		private float m_sphereRadiusMeters = 0f;
        [Space(10)]
        public UnityEvent<int> OnObjectsDetected;

        public List<BoundingBox> BoxDrawn = new();

        private string[] m_labels;
        private List<GameObject> m_boxPool = new();
        private List<GameObject> m_overlayPool = new();
        private List<GameObject> m_marker3dPool = new();
        private Transform m_displayLocation;

        //bounding box data
        public struct BoundingBox
        {
            public float CenterX;
            public float CenterY;
            public float Width;
            public float Height;
            public string Label;
            public Vector3? WorldPos;
            public string ClassName;
        }

        #region Unity Functions
        private void Start()
        {
            m_displayLocation = m_displayImage.transform;
        }
        #endregion

        #region Detection Functions
        public void OnObjectDetectionError()
        {
            // Clear current boxes
            ClearAnnotations();

            // Set obejct found to 0
            OnObjectsDetected?.Invoke(0);
        }
        #endregion

        #region BoundingBoxes functions
        public void SetLabels(TextAsset labelsAsset)
        {
            //Parse neural net m_labels
            m_labels = labelsAsset.text.Split('\n');
        }

        public void SetDetectionCapture(Texture image)
        {
            m_displayImage.texture = image;
            m_detectionCanvas.CapturePosition();
        }

        public void DrawUIBoxes(Tensor<float> output, Tensor<int> labelIDs, float imageWidth, float imageHeight, Pose cameraPose)
        {
            // Updte canvas position
            m_detectionCanvas.UpdatePosition();
            // Ensure overlay space has no unintended rotation
            m_displayLocation.localRotation = Quaternion.identity;
            m_displayImage.rectTransform.localRotation = Quaternion.identity;

            // Clear current boxes
            ClearAnnotations();

            float displayWidth = m_displayImage.rectTransform.rect.width;
            var displayHeight = m_displayImage.rectTransform.rect.height;

            var boxesFound = output.shape[0];
            if (boxesFound <= 0)
            {
                OnObjectsDetected?.Invoke(0);
                return;
            }
            // Cap how many detections we process/draw to avoid overcrowding
            var maxDetections = m_maxDetections > 0 ? m_maxDetections : boxesFound;
            var maxBoxes = Mathf.Min(boxesFound, maxDetections);

            OnObjectsDetected?.Invoke(maxBoxes);

            //Draw the bounding boxes
            for (var n = 0; n < maxBoxes; n++)
            {
                // Get bounding box center coordinates
                var normalizedCenterX = output[n, 0] / imageWidth;
                var normalizedCenterY = output[n, 1] / imageHeight;
                var centerX = displayWidth * (normalizedCenterX - 0.5f);
                var centerY = displayHeight * (normalizedCenterY - 0.5f);

                // Get object class name
                var classname = m_labels[labelIDs[n]].Replace(" ", "_");

				// Compute 3D center position
				Vector3? worldPos = null;
				float? depthMeters = null;
				var viewportP = new Vector2(normalizedCenterX, 1.0f - normalizedCenterY);

				if (m_useProjectiveCenter && m_sphereRadiusMeters != 0f && m_cameraAccess != null)
				{
					// Estimate depth from box width: d_px ≈ 2R * fx / z  ->  z ≈ 2R * fx / d_px
					var intr = m_cameraAccess.Intrinsics;
					var sensorRes = intr.SensorResolution;
					var principal = intr.PrincipalPoint;
					var focal = intr.FocalLength;

					// Scale intrinsics to the current input resolution (imageWidth/Height)
					float scaleX = imageWidth / sensorRes.x;
					float scaleY = imageHeight / sensorRes.y;
					var focalScaled = new Vector2(focal.x * scaleX, focal.y * scaleY);
					var principalScaled = new Vector2(principal.x * scaleX, principal.y * scaleY);

					// Pixel coords in input space
					float px = viewportP.x * imageWidth;
					float py = viewportP.y * imageHeight;

					float dpx = Mathf.Max(1e-3f, output[n, 2]); // width in input-space pixels
					float z = (2f * m_sphereRadiusMeters * focalScaled.x) / dpx; // meters
					z *= m_projectiveScale;

					var dirCam = new Vector3(
						(px - principalScaled.x) / focalScaled.x,
						(py - principalScaled.y) / focalScaled.y,
						1f
					);
					var pCam = dirCam * z;
					worldPos = cameraPose.position + cameraPose.rotation * pCam;
					depthMeters = z;
				}

				if (!worldPos.HasValue)
				{
					// Use environment depth raycast, then optionally offset along ray to approximate center
					var ray = m_cameraAccess.ViewportPointToRay(viewportP, cameraPose);
					worldPos = m_environmentRaycast.Raycast(ray);
					if (m_applySphereCenterOffset && worldPos.HasValue && m_sphereRadiusMeters != 0f)
					{
						worldPos = worldPos.Value + ray.direction.normalized * m_sphereRadiusMeters;
					}
					if (worldPos.HasValue)
					{
						depthMeters = Vector3.Distance(ray.origin, worldPos.Value);
					}
				}

                // Create a new bounding box
                var box = new BoundingBox
                {
                    CenterX = centerX,
                    CenterY = centerY,
                    ClassName = classname,
                    Width = output[n, 2] * (displayWidth / imageWidth),
                    Height = output[n, 3] * (displayHeight / imageHeight),
					Label = depthMeters.HasValue
						? $"Id: {n} Class: {classname} Center (px): {(int)centerX},{(int)centerY} Center (%): {normalizedCenterX:0.00},{normalizedCenterY:0.00} Depth: {depthMeters.Value:0.00}m"
						: $"Id: {n} Class: {classname} Center (px): {(int)centerX},{(int)centerY} Center (%): {normalizedCenterX:0.00},{normalizedCenterY:0.00}",
                    WorldPos = worldPos,
                };

                // Add to the list of boxes
                BoxDrawn.Add(box);

                // Draw 2D box
                DrawBox(box, n, cameraPose.position);
            }
        }

        private void ClearAnnotations()
        {
            foreach (var box in m_boxPool)
            {
                box?.SetActive(false);
            }
            foreach (var ov in m_overlayPool)
            {
                ov?.SetActive(false);
            }
            foreach (var mk in m_marker3dPool)
            {
                mk?.SetActive(false);
            }
            BoxDrawn.Clear();
        }

        private void DrawBox(BoundingBox box, int id, Vector3 cameraPosition)
        {
            //Create the bounding box graphic or get from pool
            GameObject panel;
            if (id < m_boxPool.Count)
            {
                panel = m_boxPool[id];
                if (panel == null)
                {
                    panel = CreateNewBox(m_boxColor);
                }
                else
                {
                    panel.SetActive(true);
                }
            }
            else
            {
                panel = CreateNewBox(m_boxColor);
            }
            //Set box position (2D, ignore depth)
            panel.transform.localPosition = new Vector3(box.CenterX, -box.CenterY, 0f);
            if (m_faceCamera)
            {
                // Flip direction so the panel's front faces the camera (avoid mirrored text)
                var dir = (panel.transform.position - cameraPosition).normalized;
                if (dir.sqrMagnitude > 1e-6f)
                {
                    panel.transform.rotation = Quaternion.LookRotation(dir, Vector3.up);
                }
                else
                {
                    panel.transform.rotation = Quaternion.identity;
                }
            }
            else
            {
                panel.transform.rotation = Quaternion.identity;
            }
            //Set box size
            var rt = panel.GetComponent<RectTransform>();
            rt.sizeDelta = new Vector2(box.Width, box.Height);
            //Set label text
            var label = panel.GetComponentInChildren<Text>();
            label.text = box.Label;
            label.fontSize = 12;

            // Optional 2D sphere overlay at box center
            if (m_draw2dSphereOverlay)
            {
                if (string.IsNullOrEmpty(m_sphereOverlayClassFilter) || box.ClassName == m_sphereOverlayClassFilter)
                {
                    DrawOverlaySphere(box, id);
                }
            }

            // Optional 3D marker at box center
            if (m_draw3dMarkerOverlay)
            {
                if (string.IsNullOrEmpty(m_sphereOverlayClassFilter) || box.ClassName == m_sphereOverlayClassFilter)
                {
                    DrawOverlayMarker3D(box, id, panel.transform.rotation);
                }
            }
        }

        private void DrawOverlaySphere(BoundingBox box, int id)
        {
            GameObject ov;
            if (id < m_overlayPool.Count)
            {
                ov = m_overlayPool[id];
                if (ov == null)
                {
                    ov = CreateOverlaySphere();
                }
                else
                {
                    ov.SetActive(true);
                }
            }
            else
            {
                ov = CreateOverlaySphere();
            }

            ov.transform.localPosition = new Vector3(box.CenterX, -box.CenterY, 0f);
            ov.transform.rotation = Quaternion.identity;
            var rt = ov.GetComponent<RectTransform>();
            rt.sizeDelta = m_sphereOverlaySize;
            ov.SetActive(true);
        }

        private GameObject CreateOverlaySphere()
        {
            var go = new GameObject("SphereOverlay");
            _ = go.AddComponent<CanvasRenderer>();
            var img = go.AddComponent<Image>();
            // Fallback to built-in UISprite if none assigned
            img.sprite = m_sphereOverlaySprite != null
                ? m_sphereOverlaySprite
                : Resources.GetBuiltinResource<Sprite>("UI/Skin/UISprite.psd");
            img.color = m_sphereOverlayColor;
            img.raycastTarget = false;
            go.transform.SetParent(m_displayLocation, false);
            m_overlayPool.Add(go);
            return go;
        }

        private void DrawOverlayMarker3D(BoundingBox box, int id, Quaternion rotation)
        {
            GameObject mk;
            if (id < m_marker3dPool.Count)
            {
                mk = m_marker3dPool[id];
                if (mk == null)
                {
                    mk = CreateOverlayMarker3D();
                }
                else
                {
                    mk.SetActive(true);
                }
            }
            else
            {
                mk = CreateOverlayMarker3D();
            }

            // Place marker in world space at the box center projected through the display transform
            Vector3 worldPos = m_displayLocation.TransformPoint(new Vector3(box.CenterX, -box.CenterY, 0f));
            mk.transform.SetParent(m_detectionCanvas.transform, false);
            mk.transform.position = worldPos;
            mk.transform.rotation = rotation;
            mk.transform.localScale = m_marker3dScale;
            ApplyMarkerColor(mk);
        }

        private GameObject CreateOverlayMarker3D()
        {
            GameObject go;
            if (m_marker3dPrefab != null)
            {
                go = Instantiate(m_marker3dPrefab);
            }
            else
            {
                go = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                var col = go.GetComponent<Collider>();
                if (col != null) Destroy(col);
                ApplyMarkerColor(go);
            }
            go.transform.SetParent(m_displayLocation, false);
            m_marker3dPool.Add(go);
            return go;
        }

        private void ApplyMarkerColor(GameObject go)
        {
            if (go == null) return;
            var rend = go.GetComponent<Renderer>();
            if (rend == null) return;
            var mat = rend.material;
            if (mat == null) return;
            if (mat.HasProperty("_Color"))
            {
                mat.color = m_marker3dColor;
            }
            else if (mat.HasProperty("_BaseColor"))
            {
                mat.SetColor("_BaseColor", m_marker3dColor);
            }
        }

        private GameObject CreateNewBox(Color color)
        {
            //Create the box and set image
            var panel = new GameObject("ObjectBox");
            _ = panel.AddComponent<CanvasRenderer>();
            var img = panel.AddComponent<Image>();
            img.color = color;
            img.sprite = m_boxTexture;
            img.type = Image.Type.Sliced;
            img.fillCenter = false;
            panel.transform.SetParent(m_displayLocation, false);

            //Create the label
            var text = new GameObject("ObjectLabel");
            _ = text.AddComponent<CanvasRenderer>();
            text.transform.SetParent(panel.transform, false);
            var txt = text.AddComponent<Text>();
            txt.font = m_font;
            txt.color = m_fontColor;
            txt.fontSize = m_fontSize;
            txt.horizontalOverflow = HorizontalWrapMode.Overflow;

            var rt2 = text.GetComponent<RectTransform>();
            rt2.offsetMin = new Vector2(20, rt2.offsetMin.y);
            rt2.offsetMax = new Vector2(0, rt2.offsetMax.y);
            rt2.offsetMin = new Vector2(rt2.offsetMin.x, 0);
            rt2.offsetMax = new Vector2(rt2.offsetMax.x, 30);
            rt2.anchorMin = new Vector2(0, 0);
            rt2.anchorMax = new Vector2(1, 1);

            m_boxPool.Add(panel);
            return panel;
        }
        #endregion
    }
}
