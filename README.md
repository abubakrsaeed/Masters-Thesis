# Enabling Retroreflective Marker Tracking on Meta Quest 3

This repository contains a dual-architecture implementation for purely vision-based surgical tool tracking on Meta Quest 3. It provides two operational modes: a Hybrid PC-Tethered pipeline (Python) and a Standalone On-Device application (Unity).

---

## 🖥️ Implementation 1: Hybrid PC-Tethered Tool Tracking

This implementation captures a Quest or desktop cast window, detects four marker spheres using a YOLO model, and estimates a 6-DoF pose for a custom tool using OpenCV PnP. It renders a stabilized view with projected wireframe spheres and streams live metrics (FPS, latency, jitter, position, and relative movement) to the console.

### 📁 Contents

- **`run_tracker.py`**: Main loop for screen capture, YOLO inference, point association, PnP pose solve, visual overlays, console logging, and hotkeys.
- **`spheres_openvino_model/`**: OpenVINO-exported YOLO weights (preferred for performance). Falls back to `spheres.pt` if OpenVINO files are unavailable.
- **`requirements.txt`**: Python dependencies.

### ⚙️ Prerequisites & Setup

- **OS:** Windows
- **Python:** 3.10+
- **Environment:** A casting window titled `Meta Quest Casting` (or adapt `WINDOW_TITLE` in the script).
- **Hardware:** A marker tool with four spheres positioned as defined in `TOOL_POINTS` (meters) inside `run_tracker.py`.

1. *(Optional)* Create and activate a virtual environment.
2. Install required dependencies:

```bash
pip install -r requirements.txt
```

### 🚀 Running the Tracker

1. Start your Quest stream (or another video source) so a window titled `Meta Quest Casting` is visible.
2. From the project folder, run:

```bash
python run_tracker.py
```

The script attempts to focus the cast window, grab frames via `mss`, and execute YOLO detection. If fewer than four points are detected, it continues scanning.

### 🎮 Controls & Output

- **`q`** — Quit the application.
- **`z`** — Zero the current pose; subsequent console rows include movement in millimeters relative to this baseline.

**Outputs:**

- **Console:** Per-frame, CSV-friendly row displaying FPS, total latency, YOLO inference time, jitter (mm), current X/Z (m), and delta X/Z (mm) after zeroing.
- **Window:** A `Quest Stabilized` window showing cast video with projected wireframe spheres and a status banner (movement values or `Scanning...`).

### Notes

- Camera intrinsics are currently approximated from captured frame size ($f_x = 0.65 \times width$; principal point at center), assuming zero distortion. Adjust these values if you calibrate your capture pipeline.
- Tracking uses smoothed 2D point association and `cv2.solvePnP` (`SOLVEPNP_EPNP`). If tracking drops, the script re-permutes detections to recover.
- UDP IP/port variables are defined in the script but not currently active. You can remove them or extend the script to stream pose data over the network.

<p align="center">
  <img src="Images/vis2.png" width="45%" />
  <img src="Images/vis3.png" width="45%" />
</p>

---

## 🥽 Implementation 2: Standalone On-Device (Meta Quest 3)

This implementation is built as a custom fork/extension of the Meta Passthrough Camera API Samples, modified to support stable surgical tool tracking natively on the headset.

### 🛠️ Customizations in This Fork

- **Stable 2D/3D Overlays:** Implemented YOLO detections with billboarding, pooling, and fallback sprites/meshes.  
	Ref: [Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/SentisInferenceUiManager.cs](Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/SentisInferenceUiManager.cs)

- **Tool Pose Stabilization:** Added temporal matching, RMS error rejection, low-pass filtering, optional per-axis Kalman filtering, screen-space clamping, and local offsets. Includes an attachable prefab that follows the computed pose.  
	Ref: [Assets/PassthroughCameraApiSamples/MultiObjectDetection/ToolPose/Scripts/ToolPoseEstimator.cs](Assets/PassthroughCameraApiSamples/MultiObjectDetection/ToolPose/Scripts/ToolPoseEstimator.cs)

- **Performance Logging:** Added periodic console logs for FPS, end-to-end vs. inference-only latency, pose RMS, position/depth/distance, and per-frame movement deltas.  
	Ref: [Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/PerformanceLogger.cs](Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/PerformanceLogger.cs) and [Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/SentisInferenceRunManager.cs](Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/SentisInferenceRunManager.cs)

- **External YOLO over UDP (Hybrid Mode):** Added a Unity listener that accepts normalized detection JSON to drive in-headset overlays. Pair with the Python sender (capture PC window, run YOLO, stream detections over UDP).  
	Ref: [Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/YoloListener.cs](Assets/PassthroughCameraApiSamples/MultiObjectDetection/SentisInference/Scripts/YoloListener.cs)

- **Attached 3D Markers:** Added a configurable prefab attached to stabilized tool pose, enabled/disabled based on geometric solve validity.  
	Ref: [Assets/PassthroughCameraApiSamples/MultiObjectDetection/ToolPose/Scripts/ToolPoseEstimator.cs](Assets/PassthroughCameraApiSamples/MultiObjectDetection/ToolPose/Scripts/ToolPoseEstimator.cs)

### 💡 Quick Usage Notes

- **Logging Setup:** Assign `PerformanceLogger` in your scene. Set `poseTarget` to the tool pose transform (or leave blank to fall back to `ToolPoseEstimator.WorldFromTool`).
- **Latency Metrics:** In `SentisInferenceRunManager`, `LastInferenceMs` measures raw neural network execution time, while `LastEndToEndMs` measures total capture-to-draw pipeline time.
- **UI Configuration:** In `SentisInferenceUiManager`, set `m_maxDetections` to cap per-frame bounding boxes, and choose 2D sprites or 3D markers.
- **Running External YOLO:** For PC-driven hybrid mode, run the Python sender on PC, configure Quest IP/port, and attach `YoloListener` to a world-space canvas in Unity.

<p align="center">
  <img src="Images/result1.jpg" width="45%" />
  <img src="Images/result1.jpg" width="45%" />
</p>
