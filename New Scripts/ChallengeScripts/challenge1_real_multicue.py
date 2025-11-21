"""
challenge1_real_multicue.py
Author: Jonathan Lee (for Riley McKinney)

Real Challenge 1 Mission Script
--------------------------------
- UAV takes off from UGV
- UGV moves forward (started separately)
- UAV tracks UGV ArUco marker from downward-facing camera
- UAV performs multi-cue autonomous landing:
    * pixel alignment
    * controlled descent
    * barometric altitude cue
    * vertical speed cue
    * ArUco marker size cue
    * IMU Z-axis variance cue
"""

import time
import collections
import numpy as np
import sys, os


# ==========================================================
# FIX PYTHON PATHS FOR THIS PROJECT STRUCTURE
# ==========================================================

# ChallengeScripts/
#     challenge1_real_multicue.py
# uav/
#     DroneConnect.py
#     aruco_detector.py
#     jetson.py
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.append(project_root)          # So "uav" becomes importable
sys.path.append(os.path.join(project_root, "uav"))
sys.path.append(os.path.join(project_root, "config"))

# Now we can import from /uav/
from uav.DroneConnect import DroneConnect
from uav.jetson import JetsonCamera
from uav.aruco_detector import ArucoDetector


# ==========================================================
# CONFIGURATION
# ==========================================================

TARGET_MARKER_ID = 1
CALIB_FILE = "config/calibration_chessboard.yaml"
MARKER_SIZE_M = 0.254  # 10 inch marker

TAKEOFF_ALT = 3.5
MAX_VEL = 0.6
DESCENT_RATE = 0.12

ALIGN_TOLERANCE_PX = 20
DESCENT_START_TOL_PX = 25
STABILITY_FRAMES = 15

# Multi-cue touchdown
LANDED_ALT_THRESHOLD = 0.25
VERT_SPEED_THRESHOLD = 0.05
MARKER_AREA_MIN = 80000.0
ACCEL_Z_VAR_WINDOW = 20
ACCEL_Z_VAR_THRESHOLD = 0.15

LOOP_DT = 0.05  # 20 Hz


# ==========================================================
# HELPER FUNCTIONS
# ==========================================================

def compute_velocity_from_pixel_error(err_x, err_y):
    k = 0.003  # proportional gain
    vx = np.clip(-err_y * k, -MAX_VEL, MAX_VEL)
    vy = np.clip(-err_x * k, -MAX_VEL, MAX_VEL)
    return vx, vy


def is_alignment_good(err_x, err_y, tol):
    return abs(err_x) < tol and abs(err_y) < tol


def touchdown_detected(alt, vert_speed, marker_area, accel_z_history):
    if alt is None:
        return False

    alt_ok = alt < LANDED_ALT_THRESHOLD
    vs_ok = abs(vert_speed) < VERT_SPEED_THRESHOLD
    marker_ok = marker_area and marker_area > MARKER_AREA_MIN

    if len(accel_z_history) < 5:
        var_ok = False
    else:
        az_var = np.var(np.array(accel_z_history))
        var_ok = az_var < ACCEL_Z_VAR_THRESHOLD

    print(f"[TOUCHDOWN TEST] alt_ok={alt_ok}, vs_ok={vs_ok}, "
          f"marker_ok={marker_ok}, var_ok={var_ok}")

    return alt_ok and vs_ok and marker_ok and var_ok


# ==========================================================
# MAIN MISSION LOGIC
# ==========================================================

def main():
    print("[BOOT] Connecting to Pixhawk through DroneConnect...")
    uav = DroneConnect(connection_string="/dev/ttyACM0", baud=115200)
    uav.wait_ready()

    print("[BOOT] Connecting to Jetson camera...")
    cam = JetsonCamera(resolution=(640, 480), framerate=30)

    print("[BOOT] Initializing ArUco detector...")
    detector = ArucoDetector(calibration_path=CALIB_FILE, marker_size=MARKER_SIZE_M)

    print("[MISSION] Arming UAV...")
    uav.arm()

    print(f"[MISSION] Taking off to {TAKEOFF_ALT}m...")
    uav.takeoff(TAKEOFF_ALT)
    time.sleep(1)

    print("[MISSION] UGV should now be driving forward.")
    print("[MISSION] Starting marker tracking...")

    stable_frames = 0
    landing_phase = False
    accel_z_history = collections.deque(maxlen=ACCEL_Z_VAR_WINDOW)

    try:
        while True:
            frame = cam.read()
            if frame is None:
                print("[WARN] No camera frame; hovering...")
                uav.send_body_velocity(0, 0, 0)
                time.sleep(LOOP_DT)
                continue

            h, w = frame.shape[:2]
            cx_cam = w / 2
            cy_cam = h / 2

            detection = detector.detect_single_marker(frame, TARGET_MARKER_ID)

            if detection is None:
                print("[TRACK] Marker not visible; hover...")
                uav.send_body_velocity(0, 0, 0)
                stable_frames = 0
                accel_z_history.append(uav.get_accel_z())
                time.sleep(LOOP_DT)
                continue

            cx, cy = detection["pixel"]
            area = detection["area"]

            err_x = cx - cx_cam
            err_y = cy - cy_cam

            vx, vy = compute_velocity_from_pixel_error(err_x, err_y)
            vz = 0.0

            if is_alignment_good(err_x, err_y, DESCENT_START_TOL_PX):
                stable_frames += 1
            else:
                stable_frames = 0

            print(f"[TRACK] err_x={err_x:.1f}, err_y={err_y:.1f}, "
                  f"stable={stable_frames}, area={area:.1f}")

            if stable_frames >= STABILITY_FRAMES:
                landing_phase = True

            if landing_phase:
                vz = -DESCENT_RATE

            uav.send_body_velocity(vx, vy, vz)

            # Telemetry for touchdown detection
            alt = uav.get_altitude()
            vs = uav.get_vertical_speed()
            az = uav.get_accel_z()
            accel_z_history.append(az)

            print(f"[TELEM] alt={alt:.2f}, vs={vs:.2f}, az={az:.2f}")

            if landing_phase:
                if touchdown_detected(
                    alt=alt,
                    vert_speed=vs,
                    marker_area=area,
                    accel_z_history=accel_z_history
                ):
                    print("[LAND] TOUCHDOWN CONFIRMED.")
                    break

            time.sleep(LOOP_DT)

    finally:
        print("[MISSION] Stopping motors and disarming...")
        uav.send_body_velocity(0, 0, 0)
        uav.disarm()
        cam.release()
        print("[MISSION COMPLETE] Challenge 1 successful landing.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[ABORT] Mission aborted.")
