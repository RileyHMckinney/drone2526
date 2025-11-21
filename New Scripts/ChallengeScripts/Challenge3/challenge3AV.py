"""
challenge3AV.py

Challenge 3 - UAV Master Script (BrainChip/Akida Challenge)
-----------------------------------------------------------
This script controls the UAV during Challenge 3.

Key Points:
- Challenge 3 does NOT change UAV behavior from Challenge 2.
- UAV still performs:
    • UGV marker tracking
    • Goal marker detection
    • UGV→Goal vector computation
    • Vector streaming to the UGV
    • Smooth descent toward landing
    • Autonomous landing on the moving UGV

The ONLY difference between Challenge 2 and Challenge 3 is on the UGV side,
where Challenge 3 uses BrainChip/Akida hardware for obstacle avoidance.

Therefore, this script mirrors challenge2AV.py but is provided here
as a clean, clearly named version specifically for Challenge 3.
"""

import time
import collections
import socket
import json
import logging
import numpy as np
import sys
import os

# ==========================================================
# PATH FIXUP FOR UAV MODULES
# ==========================================================

project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.append(project_root)
sys.path.append(os.path.join(project_root, "uav"))

from uav.DroneConnect import DroneConnect
from uav.jetson import JetsonCamera
from uav.aruco_detector import ArucoDetector, GOAL_MARKER_ID, UGV_MARKER_ID


# ==========================================================
# LOGGING
# ==========================================================

LOG_FILE = os.path.join(os.path.dirname(__file__), "challenge3AV.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [C3-AV] %(levelname)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("Challenge3AV")


# ==========================================================
# NETWORK / MISSION PARAMETERS
# ==========================================================

UGV_IP = "192.168.4.1"      # TODO: set to actual UGV Pi or ESP32 IP
UGV_PORT = 5006             # Vector streaming port (must match GV side)

UGV_CONNECT_RETRIES = 5
UGV_RETRY_DELAY = 2.0

TAKEOFF_ALT = 4.0
MAX_VEL = 0.8
SEARCH_FORWARD_VX = 0.20
GUIDE_DESCENT_RATE = 0.05
LANDING_DESCENT_RATE = 0.12

ALIGN_TOL_PX = 30
STABILITY_FRAMES = 10

LANDED_ALT = 0.25
VERT_SPEED_TOL = 0.05
MARKER_MIN_AREA = 80000.0
ACCEL_Z_VAR_WINDOW = 20
ACCEL_Z_VAR_TOL = 0.15

LOOP_DT = 0.05
MISSION_TIMEOUT_SEC = 8 * 60

NEAR_GOAL_DISTANCE_M = 1.0


# ==========================================================
# COMM HELPERS
# ==========================================================

def connect_to_ugv():
    for attempt in range(1, UGV_CONNECT_RETRIES + 1):
        try:
            log.info(f"[NET] Connecting to UGV at {UGV_IP}:{UGV_PORT} (attempt {attempt})...")
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((UGV_IP, UGV_PORT))
            sock.settimeout(None)
            log.info("[NET] UGV vector channel established.")
            return sock
        except Exception as e:
            log.warning(f"[NET] Connection failed: {e}")
            time.sleep(UGV_RETRY_DELAY)

    log.error("[NET] Could not establish vector channel.")
    return None


def send_vector(sock, payload):
    if sock is None:
        log.warning("[NET] No socket; skipping vector send.")
        return
    try:
        msg = json.dumps(payload) + "\n"
        sock.sendall(msg.encode("utf-8"))
        d = payload.get("distance_m", None)
        if d is not None:
            log.info(f"[NET] Sent vector (distance={d:.2f} m).")
    except Exception as e:
        log.warning(f"[NET] Failed to send vector: {e}")


# ==========================================================
# CONTROL HELPERS
# ==========================================================

def compute_xy_velocity(err_x, err_y, forward_bias=0.0):
    """PID-lite centering control + optional forward drift."""
    k = 0.003
    vx = -err_y * k + forward_bias
    vy = -err_x * k
    return (
        float(np.clip(vx, -MAX_VEL, MAX_VEL)),
        float(np.clip(vy, -MAX_VEL, MAX_VEL))
    )


def touchdown_detected(alt, vs, area, accel_hist):
    if alt is None:
        return False

    alt_ok = alt < LANDED_ALT
    vs_ok = abs(vs) < VERT_SPEED_TOL
    area_ok = (area is not None) and (area > MARKER_MIN_AREA)

    if len(accel_hist) < 5:
        var_ok = False
    else:
        var_ok = np.var(np.array(accel_hist)) < ACCEL_Z_VAR_TOL

    log.info(
        f"[TOUCHDOWN CHECK] alt_ok={alt_ok} vs_ok={vs_ok} area_ok={area_ok} var_ok={var_ok}"
    )
    return alt_ok and vs_ok and area_ok and var_ok


# ==========================================================
# MAIN STATE MACHINE
# ==========================================================

def main():
    start_time = time.time()

    mode = "SEARCH_GOAL"
    accel_z_hist = collections.deque(maxlen=ACCEL_Z_VAR_WINDOW)
    last_distance_m = None

    # Connect to UGV
    ugv_sock = connect_to_ugv()
    if ugv_sock is None:
        log.warning("[WARN] Running without UGV vector output (TEST MODE).")

    # UAV setup
    log.info("[BOOT] Connecting to UAV flight controller...")
    uav = DroneConnect(connection_string="/dev/ttyACM0", baud=115200)
    uav.wait_ready()

    log.info("[BOOT] Starting Jetson camera...")
    cam = JetsonCamera(resolution=(640, 480), framerate=30)

    log.info("[BOOT] Initializing ArUco detector...")
    detector = ArucoDetector()

    log.info("[MISSION] Arming UAV...")
    uav.arm()

    log.info(f"[MISSION] Taking off to {TAKEOFF_ALT} m...")
    uav.takeoff(TAKEOFF_ALT)

    log.info("[MISSION] Entering main Challenge 3 control loop...")

    try:
        while True:

            if time.time() - start_time > MISSION_TIMEOUT_SEC:
                log.error("[TIMEOUT] Mission exceeded time limit.")
                break

            frame = cam.read()
            if frame is None:
                uav.send_body_velocity(0, 0, 0)
                log.warning("[CAM] No frame; hovering.")
                accel_z_hist.append(uav.get_accel_z())
                time.sleep(LOOP_DT)
                continue

            h, w = frame.shape[:2]
            cx_mid, cy_mid = w / 2, h / 2

            detections = detector.detect_markers(frame)
            ugv_marker = next((d for d in detections if d["marker_id"] == UGV_MARKER_ID), None)
            goal_marker = next((d for d in detections if d["marker_id"] == GOAL_MARKER_ID), None)

            # ===========================
            # LANDING MODE
            # ===========================
            if mode == "LANDING":

                if ugv_marker is None:
                    log.warning("[LANDING] Lost UGV marker; hovering.")
                    uav.send_body_velocity(0, 0, 0)
                    accel_z_hist.append(uav.get_accel_z())
                    time.sleep(LOOP_DT)
                    continue

                cx, cy = ugv_marker["pixel"]
                area = ugv_marker["area"]

                err_x, err_y = cx - cx_mid, cy - cy_mid
                vx, vy = compute_xy_velocity(err_x, err_y)
                vz = -LANDING_DESCENT_RATE

                uav.send_body_velocity(vx, vy, vz)

                alt = uav.get_altitude()
                vs = uav.get_vertical_speed()
                az = uav.get_accel_z()
                accel_z_hist.append(az)

                log.info(
                    f"[LANDING] alt={alt:.2f} vs={vs:.2f} "
                    f"err=({err_x:.1f},{err_y:.1f}) area={area:.0f}"
                )

                if touchdown_detected(alt, vs, area, accel_z_hist):
                    log.info("[LANDING] Touchdown detected!")
                    break

                time.sleep(LOOP_DT)
                continue

            # ===========================
            # SEARCH_GOAL / GUIDE_AND_DESCEND
            # ===========================

            if ugv_marker is None:
                log.info("[TRACK] Lost UGV marker; hovering.")
                uav.send_body_velocity(0, 0, 0)
                accel_z_hist.append(uav.get_accel_z())
                time.sleep(LOOP_DT)
                continue

            cx_u, cy_u = ugv_marker["pixel"]
            area_u = ugv_marker["area"]
            err_x, err_y = cx_u - cx_mid, cy_u - cy_mid

            if mode == "SEARCH_GOAL":
                forward_bias = SEARCH_FORWARD_VX
                vz = 0.0
            else:
                forward_bias = 0.0
                vz = -GUIDE_DESCENT_RATE

            vx, vy = compute_xy_velocity(err_x, err_y, forward_bias)

            # Vector calculation if goal visible
            if goal_marker is not None:
                vec_data = detector.compute_vector_between_markers(goal_marker, ugv_marker)
                last_distance_m = vec_data["distance_m"]

                payload = {
                    "cmd": "vector_to_goal",
                    "ugv_marker_id": vec_data["ugv_marker_id"],
                    "goal_marker_id": vec_data["goal_marker_id"],
                    "vector_ugv_to_goal": vec_data["vector_ugv_to_goal"],
                    "direction_unit": vec_data["direction_unit"],
                    "distance_m": vec_data["distance_m"],
                    "timestamp": vec_data["timestamp"]
                }
                send_vector(ugv_sock, payload)

                if mode == "SEARCH_GOAL":
                    log.info("[MODE] Goal marker acquired → GUIDE_AND_DESCEND")
                    mode = "GUIDE_AND_DESCEND"

                if mode == "GUIDE_AND_DESCEND" and last_distance_m is not None:
                    if last_distance_m < NEAR_GOAL_DISTANCE_M:
                        log.info("[MODE] UGV near goal → LANDING")
                        mode = "LANDING"

            uav.send_body_velocity(vx, vy, vz)

            alt = uav.get_altitude()
            vs = uav.get_vertical_speed()
            az = uav.get_accel_z()
            accel_z_hist.append(az)

            log.info(
                f"[{mode}] alt={alt:.2f} vs={vs:.2f} "
                f"UGV_err=({err_x:.1f},{err_y:.1f}) "
                f"d_goal={last_distance_m if last_distance_m else -1:.2f}"
            )

            time.sleep(LOOP_DT)

    finally:
        log.info("[MISSION] Stopping UAV and closing resources.")
        try:
            uav.send_body_velocity(0, 0, 0)
        except:
            pass
        uav.disarm()

        try:
            cam.release()
        except:
            pass

        if ugv_sock:
            try: ugv_sock.close()
            except: pass

        log.info("[MISSION COMPLETE] Challenge 3 AV finished.")


if __name__ == "__main__":
    main()
