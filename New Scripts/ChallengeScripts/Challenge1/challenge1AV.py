"""
challenge1AV.py
Author: Riley McKinney

Challenge 1 - UAV Master Script
--------------------------------
- Connects to UGV over TCP and sends "start" to begin forward driving
- Arms and takes off UAV
- Tracks UGV ArUco marker using downward-facing camera
- Performs multi-cue autonomous landing on moving UGV
- Sends "stop" command to UGV after touchdown or on timeout
- Includes logging, basic reconnect logic, and mission timeout fallback
"""

import time
import collections
import socket
import logging
import numpy as np
import sys, os


# ==========================================================
# PATH SETUP
# ==========================================================

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.append(project_root)
sys.path.append(os.path.join(project_root, "uav"))

from uav.DroneConnect import DroneConnect
from uav.jetson import JetsonCamera
from uav.aruco_detector import ArucoDetector


# ==========================================================
# LOGGING SETUP
# ==========================================================

LOG_FILE = os.path.join(os.path.dirname(__file__), "challenge1AV.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [AV] %(levelname)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("Challenge1AV")


# ==========================================================
# NETWORK / MISSION PARAMETERS
# ==========================================================

UGV_IP = "192.168.4.1"      # TODO: set to actual ESP32/UGV IP
UGV_PORT = 5005
UGV_CONNECT_RETRIES = 5
UGV_RETRY_DELAY = 2.0       # seconds

# Marker / vision
TARGET_MARKER_ID = 1
CALIB_FILE = "config/calibration_chessboard.yaml"
MARKER_SIZE_M = 0.254

# Flight
TAKEOFF_ALT = 3.5
MAX_VEL = 0.6
DESCENT_RATE = 0.12

ALIGN_TOL_PX = 20
DESCENT_TOL_PX = 25
STABILITY_FRAMES = 15

# Touchdown detection
LANDED_ALT = 0.25
VERT_SPEED_TOL = 0.05
MARKER_MIN_AREA = 80000.0
ACCEL_Z_VAR_WINDOW = 20
ACCEL_Z_VAR_TOL = 0.15

LOOP_DT = 0.05

# Mission timeout (e.g., 7 minutes)
MISSION_TIMEOUT_SEC = 7 * 60


# ==========================================================
# UGV TCP HELPERS
# ==========================================================

def connect_to_ugv():
    """Try to connect to UGV with retries; return socket or None."""
    for attempt in range(1, UGV_CONNECT_RETRIES + 1):
        try:
            log.info(f"Connecting to UGV {UGV_IP}:{UGV_PORT} (attempt {attempt})...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((UGV_IP, UGV_PORT))
            s.settimeout(None)
            log.info("UGV connection established.")
            return s
        except Exception as e:
            log.warning(f"UGV connection failed: {e}")
            time.sleep(UGV_RETRY_DELAY)
    log.error("Failed to connect to UGV after multiple attempts.")
    return None


def send_to_ugv(sock, msg: str):
    """Send a line-based command to UGV; failures are logged but not fatal."""
    if sock is None:
        log.warning(f"Cannot send '{msg}': no UGV socket.")
        return
    try:
        sock.sendall((msg + "\n").encode("utf-8"))
        log.info(f"Sent to UGV: {msg}")
    except Exception as e:
        log.warning(f"Failed to send '{msg}' to UGV: {e}")


# ==========================================================
# CONTROL HELPERS
# ==========================================================

def compute_vel(err_x, err_y):
    k = 0.003
    vx = np.clip(-err_y * k, -MAX_VEL, MAX_VEL)
    vy = np.clip(-err_x * k, -MAX_VEL, MAX_VEL)
    return vx, vy


def touchdown_detected(alt, vert_speed, area, accel_hist):
    if alt is None:
        return False

    alt_ok = alt < LANDED_ALT
    vs_ok = abs(vert_speed) < VERT_SPEED_TOL
    area_ok = area is not None and area > MARKER_MIN_AREA

    if len(accel_hist) < 5:
        var_ok = False
    else:
        var_ok = np.var(np.array(accel_hist)) < ACCEL_Z_VAR_TOL

    log.info(f"[TOUCHDOWN CHECK] alt_ok={alt_ok}, vs_ok={vs_ok}, "
             f"area_ok={area_ok}, var_ok={var_ok}")
    return alt_ok and vs_ok and area_ok and var_ok


# ==========================================================
# MAIN
# ==========================================================

def main():
    start_time = time.time()

    # --- UGV connection ---
    ugv_sock = connect_to_ugv()
    if ugv_sock is None:
        log.warning("Proceeding without UGV control (UGV must be started manually).")
    else:
        send_to_ugv(ugv_sock, "start")

    # --- UAV / vision setup ---
    log.info("Connecting to UAV flight controller...")
    uav = DroneConnect(connection_string="/dev/ttyACM0", baud=115200)
    uav.wait_ready()

    log.info("Starting Jetson camera...")
    cam = JetsonCamera(resolution=(640, 480), framerate=30)

    log.info("Initializing ArUco detector...")
    det = ArucoDetector(calibration_path=CALIB_FILE, marker_size=MARKER_SIZE_M)

    log.info("Arming UAV...")
    uav.arm()

    log.info(f"Taking off to {TAKEOFF_ALT} m...")
    uav.takeoff(TAKEOFF_ALT)
    time.sleep(1)

    stable_frames = 0
    landing_phase = False
    accel_z_hist = collections.deque(maxlen=ACCEL_Z_VAR_WINDOW)

    log.info("Entering tracking + landing loop...")

    try:
        while True:
            # Mission timeout fallback
            if time.time() - start_time > MISSION_TIMEOUT_SEC:
                log.error("MISSION TIMEOUT reached! Aborting landing and stopping UGV.")
                break

            frame = cam.read()
            if frame is None:
                log.warning("No camera frame; hovering.")
                uav.send_body_velocity(0, 0, 0)
                accel_z_hist.append(uav.get_accel_z())
                time.sleep(LOOP_DT)
                continue

            h, w = frame.shape[:2]
            cx_cam = w / 2
            cy_cam = h / 2

            d = det.detect_single_marker(frame, TARGET_MARKER_ID)

            if d is None:
                log.info("Marker not visible; hover.")
                uav.send_body_velocity(0, 0, 0)
                accel_z_hist.append(uav.get_accel_z())
                stable_frames = 0
                time.sleep(LOOP_DT)
                continue

            cx, cy = d["pixel"]
            area = d["area"]
            err_x = cx - cx_cam
            err_y = cy - cy_cam

            vx, vy = compute_vel(err_x, err_y)
            vz = 0.0

            if abs(err_x) < DESCENT_TOL_PX and abs(err_y) < DESCENT_TOL_PX:
                stable_frames += 1
            else:
                stable_frames = 0

            if stable_frames >= STABILITY_FRAMES:
                landing_phase = True

            if landing_phase:
                vz = -DESCENT_RATE

            # Send body velocities
            uav.send_body_velocity(vx, vy, vz)

            # Telemetry
            alt = uav.get_altitude()
            vs = uav.get_vertical_speed()
            az = uav.get_accel_z()
            accel_z_hist.append(az)

            log.info(f"[TELEM] alt={alt:.2f}, vs={vs:.2f}, area={area:.1f}, "
                     f"err=({err_x:.1f},{err_y:.1f}), landing={landing_phase}")

            if landing_phase and touchdown_detected(alt, vs, area, accel_z_hist):
                log.info("TOUCHDOWN detected by multi-cue logic.")
                break

            time.sleep(LOOP_DT)

    finally:
        log.info("Sending STOP to UGV...")
        send_to_ugv(ugv_sock, "stop")

        log.info("Disarming UAV and stopping velocity commands...")
        try:
            uav.send_body_velocity(0, 0, 0)
        except Exception as e:
            log.warning(f"Error sending zero velocity: {e}")
        uav.disarm()

        try:
            cam.release()
        except Exception:
            pass

        if ugv_sock is not None:
            try:
                ugv_sock.close()
            except Exception:
                pass

        log.info("MISSION COMPLETE (Challenge 1 AV).")


if __name__ == "__main__":
    main()
