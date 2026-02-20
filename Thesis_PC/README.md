# Thesis_PC Tool Tracking

This script captures a Quest/desktop cast window, detects four marker spheres with a YOLO model, and estimates a 6-DoF pose for a custom tool using OpenCV PnP. It renders a stabilised view with projected wireframe spheres and streams live metrics (FPS, latency, jitter, position and relative movement) to the console.

## Contents
- `run_tracker.py` — main loop: screen capture, YOLO inference, point association, PnP pose solve, visual overlay, console logging, and hotkeys.
- `spheres_openvino_model/` — OpenVINO-exported YOLO weights (preferred). Falls back to `spheres.pt` if OpenVINO files are unavailable.
- `requirements.txt` — Python dependencies.

## Prerequisites
- Python 3.10+ on Windows.
- A casting window titled `Meta Quest Casting` (or adapt `WINDOW_TITLE` in the script).
- A marker tool with four spheres positioned as defined in `TOOL_POINTS` (meters) inside `run_tracker.py`.

## Setup
1) (Optional) Create and activate a virtual environment.
2) Install dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Running
1) Start your Quest (or other) stream so a window titled `Meta Quest Casting` is visible.
2) From the project folder, run:
   ```bash
   python run_tracker.py
   ```
3) The script will try to focus the cast window, grab frames via `mss`, and run YOLO detection. If fewer than four points are detected, it keeps scanning.

## Controls
- `q` — quit.
- `z` — zero the current pose; subsequent console rows include movement in mm relative to this baseline.

## Output
- Console: per-frame CSV-friendly row with FPS, total latency, YOLO time, jitter (mm), current X/Z (m), and delta X/Z (mm) after zeroing.
- Window: `Quest Stabilized` shows the cast video with projected wireframe spheres and a status banner (movement values or "Scanning...").

## Notes
- Camera intrinsics are approximated from the captured frame size (fx = 0.65 * width; principal point at center) with zero distortion. Adjust if you calibrate the capture pipeline.
- Tracking uses smoothed 2D point associations and `cv2.solvePnP` (`SOLVEPNP_EPNP`). If tracking drops, the script re-permutes detections to recover.
- UDP IP/port are currently defined but not used; remove or extend to stream pose data as needed.
