import cv2
import numpy as np
import mss
import pygetwindow as gw
from ultralytics import YOLO
import itertools
import socket
import time

# --- CONFIGURATION ---
WINDOW_TITLE = "Meta Quest Casting"
UDP_IP = "127.0.0.1"
UDP_PORT = 5005

# TOOL GEOMETRY (Meters)
TOOL_POINTS = np.array([
    [0.000,    0.000,    0.000],    
    [0.074789, 0.000,    0.000],    
    [0.025217, 0.034311, 0.000],    
    [0.045238, -0.030790, 0.017880]
], dtype=np.float32)

SMOOTH_FACTOR = 0.5 

# --- VISUALS ---
def create_wireframe_sphere(radius, sectors=8):
    points = []
    for i in range(sectors):
        theta = 2 * np.pi * i / sectors
        points.append([radius * np.cos(theta), radius * np.sin(theta), 0])
        points.append([radius * np.cos(theta), 0, radius * np.sin(theta)])
        points.append([0, radius * np.cos(theta), radius * np.sin(theta)])
    return np.array(points, dtype=np.float32)

def draw_sphere(img, rvec, tvec, cam_mat, dist, model, color):
    imgpts, _ = cv2.projectPoints(model, rvec, tvec, cam_mat, dist)
    imgpts = imgpts.reshape(-1, 2).astype(int)
    for pt in imgpts: cv2.circle(img, tuple(pt), 1, color, -1)

# --- SETUP ---
print("🚀 Initializing Axis Logger...")
try: model = YOLO('spheres_openvino_model/')
except: model = YOLO('spheres.pt')

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sct = mss.mss()
try:
    window = gw.getWindowsWithTitle(WINDOW_TITLE)[0]
    window.activate()
except: pass

sphere_model = create_wireframe_sphere(0.0065)

# STATE
is_tracking = False
last_rvec, last_tvec = None, None
cached_2d_points = np.zeros((4, 2), dtype=np.float32)
prev_position = np.array([0.0, 0.0, 0.0])

# AXIS TEST VARIABLES
ref_x = None
ref_z = None

print("✅ TRACKING STARTED.")
print("👉 PRESS 'z' TO ZERO THE METRICS (Start Ruler Test)")
print("-" * 110)
# Header optimized for Excel Copy-Paste
print(f"{'FPS':<5} | {'LAT(ms)':<8} | {'YOLO(ms)':<8} | {'JITTER':<8} | {'POS_X(m)':<10} | {'POS_Z(m)':<10} | {'MOVED_X(mm)':<12} | {'MOVED_Z(mm)':<12}")
print("-" * 110)

while True:
    loop_start_time = time.time()
    try:
        # 1. Capture
        if 'window' in locals():
            monitor = {"top": window.top + 30, "left": window.left, "width": window.width, "height": window.height - 30}
        else:
            monitor = sct.monitors[1] 

        img = np.array(sct.grab(monitor))
        frame = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        H, W, _ = frame.shape
        fx = W * 0.65  
        camera_matrix = np.array([[fx, 0, W/2], [0, fx, H/2], [0, 0, 1]], dtype=np.float32)
        dist_coeffs = np.zeros((4, 1))

        # 2. Detect with Timer
        yolo_start = time.time()
        results = model(frame, stream=True, verbose=False, conf=0.4)
        raw_detected = []
        for r in results:
            for box in r.boxes:
                cx = (box.xyxy[0][0] + box.xyxy[0][2]) / 2
                cy = (box.xyxy[0][1] + box.xyxy[0][3]) / 2
                raw_detected.append([float(cx), float(cy)])
        yolo_ms = (time.time() - yolo_start) * 1000
        
        raw_arr = np.array(raw_detected, dtype=np.float32)
        valid_pose = False
        
        # 3. Logic
        if len(raw_detected) >= 4:
            sorted_2d = np.zeros((4, 2), dtype=np.float32)
            if is_tracking:
                predicted_pts, _ = cv2.projectPoints(TOOL_POINTS, last_rvec, last_tvec, camera_matrix, dist_coeffs)
                predicted_pts = predicted_pts.reshape(-1, 2)
                used_indices = set()
                match_failed = False
                for i in range(4):
                    dists = np.linalg.norm(raw_arr - predicted_pts[i], axis=1)
                    best_idx = -1
                    best_dist = 9999
                    for idx, d in enumerate(dists):
                        if idx not in used_indices and d < best_dist:
                            best_dist = d
                            best_idx = idx
                    if best_dist > 50: 
                        match_failed = True
                        break
                    new_raw_pt = raw_arr[best_idx]
                    old_smooth_pt = cached_2d_points[i]
                    smoothed_pt = old_smooth_pt * (1 - SMOOTH_FACTOR) + new_raw_pt * SMOOTH_FACTOR
                    sorted_2d[i] = smoothed_pt
                    cached_2d_points[i] = smoothed_pt
                    used_indices.add(best_idx)
                
                if not match_failed:
                    success, r, t = cv2.solvePnP(TOOL_POINTS, sorted_2d, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_EPNP)
                    if success:
                        if cv2.norm(sorted_2d, cv2.projectPoints(TOOL_POINTS, r, t, camera_matrix, dist_coeffs)[0].reshape(-1,2), cv2.NORM_L2) < 20:
                            last_rvec, last_tvec = r, t
                            valid_pose = True

            if not valid_pose:
                best_err = 9999
                for p in itertools.permutations(raw_arr[:4]):
                    p_ord = np.ascontiguousarray(p, dtype=np.float32)
                    try:
                        success, r, t = cv2.solvePnP(TOOL_POINTS, p_ord, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_EPNP)
                        if success:
                            err = cv2.norm(p_ord, cv2.projectPoints(TOOL_POINTS, r, t, camera_matrix, dist_coeffs)[0].reshape(-1,2), cv2.NORM_L2)
                            if err < best_err:
                                best_err = err
                                last_rvec, last_tvec = r, t
                                cached_2d_points = p_ord 
                    except: pass
                if best_err < 20:
                    valid_pose = True
                    is_tracking = True

        # 4. LOGS
        current_time = time.time()
        total_latency_ms = (current_time - loop_start_time) * 1000
        fps = 1.0 / (current_time - loop_start_time + 0.00001)

        if valid_pose:
            # Draw
            rmat, _ = cv2.Rodrigues(last_rvec)
            for i in range(4):
                p_local = TOOL_POINTS[i].reshape(3, 1)
                p_cam = np.dot(rmat, p_local) + last_tvec
                draw_sphere(frame, last_rvec, p_cam, camera_matrix, dist_coeffs, sphere_model, (0, 255, 0))
            
            p = last_tvec.flatten()
            current_x = p[0]
            current_z = p[2]
            
            # --- CALCULATE MOVEMENT ---
            moved_x_mm = 0.0
            moved_z_mm = 0.0
            
            if ref_x is not None:
                moved_x_mm = (current_x - ref_x) * 1000.0
                moved_z_mm = (current_z - ref_z) * 1000.0

            # Jitter
            current_pos = np.array([p[0], p[1], p[2]])
            jitter_mm = np.linalg.norm(current_pos - prev_position) * 1000.0 
            prev_position = current_pos

            # PRINT FULL DATA ROW
            print(f"{int(fps):<5} | {total_latency_ms:<8.1f} | {yolo_ms:<8.1f} | {jitter_mm:<8.2f} | {current_x:<10.4f} | {current_z:<10.4f} | {moved_x_mm:<12.2f} | {moved_z_mm:<12.2f}")
            
            status_text = f"X_Move: {moved_x_mm:.1f}mm  Z_Move: {moved_z_mm:.1f}mm"
            cv2.putText(frame, status_text, (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        else:
            is_tracking = False
            print(f"{int(fps):<5} | {total_latency_ms:<8.1f} | {yolo_ms:<8.1f} | {'---':<8} | {'---':<10} | {'---':<10} | {'---':<12} | {'---':<12}")
            cv2.putText(frame, "Scanning...", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        cv2.imshow('Quest Stabilized', frame)
        key = cv2.waitKey(1)
        if key == ord('q'): break
        if key == ord('z'): # ZERO THE AXIS
            ref_x = last_tvec.flatten()[0]
            ref_z = last_tvec.flatten()[2]
            print("\n🔵 ZERO POINT SET! MOVEMENT TRACKING STARTED.\n")
            
    except Exception as e: 
        pass
cv2.destroyAllWindows()