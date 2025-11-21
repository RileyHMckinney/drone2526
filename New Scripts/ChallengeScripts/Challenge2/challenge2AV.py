"""
challenge2AV.py

Challenge 2 - UAV Master Script
--------------------------------
High-level behavior:
- Takes off from UGV
- Phase 1 (SEARCH_GOAL):
    * Keep UGV marker in frame & near center
    * Drift forward to 'scan' for goal marker
- Phase 2 (GUIDE_AND_DESCEND):
    * Once goal marker is visible with UGV marker, compute UGV->goal vector
    * Send vector to UGV over TCP as JSON
    * Continue tracking UGV marker and slowly descend
    * Update vector each frame as needed
- Phase 3 (LANDING):
    * When UGV is close to goal (distance small), switch to landing mode
    * Land on UGV using multi-cue touchdown detection
"""

import time
import collections
import socket
import json
import logging
import numpy as np
import sys, os


# ==========================================================
# PATHS & IMPORTS
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

LOG_FILE = os.path.join(os.path.dirname(__file__), "challenge2AV.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [C2-AV] %(levelname)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("Challenge2AV")


# ==========================================================
# NETWORK / MISSION PARAMETERS
# ==========================================================

UGV_IP = "192.168.4.1"      # TODO: set to actual UGV/ESP32 IP
UGV_PORT = 5006             # Separate port for Challenge 2 vector stream

UGV_CONNECT_RETRIES = 5
UGV_RETRY_DELAY = 2.0

# Flight & control parameters
TAKEOFF_ALT = 4.0
MAX_VEL = 0.8
SEARCH_FORWARD_VX = 0.2     # Forward bias during search
GUIDE_DESCENT_RATE = 0.05   # Slow descent in GUIDE phase
LANDING_DESCENT_RATE = 0.12

ALIGN_TOL_PX = 30           # tolerance for "good tracking" of UGV marker
STABILITY_FRAMES = 10

# Touchdown
LANDED_ALT = 0.25
VERT_SPEED_TOL = 0.05
MARKER_MIN_AREA = 80000.0
ACCEL_Z_VAR_WINDOW = 20
ACCEL_Z_VAR_TOL = 0.15

LOOP_DT = 0.05
MISSION_TIMEOUT_SEC = 8 * 60  # 8 minutes for C2

# Distance threshold to switch from GUIDE to LANDING
NEAR_GOAL_DISTANCE_M = 1.0


# ==========================================================
# NETWORK HELPERS
# ==========================================================

def connect_to_ugv():
    """Connect to UGV vector receiver with retries."""
    for attempt in range(1, UGV_CONNECT_RETRIES + 1):
        try:
            log.info(f"Connecting to UGV {UGV_IP}:{UGV_PORT} (attempt {attempt})...")
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((UGV_IP, UGV_PORT))
            s.settimeout(None)
            log.info("UGV vector channel established.")
            return s
        except Exception as e:
            log.warning(f"UGV connection failed: {e}")
            time.sleep(UGV_RETRY_DELAY)
    log.error("Failed to connect to UGV after multiple attempts.")
    return None


def send_vector_to_ugv(sock, vector_payload: dict):
    """Send JSON vector payload to UGV; non-fatal on failure."""
    if sock is None:
        log.warning("No UGV socket; skipping vector send.")
        return
    try:
        data = json.dumps(vector_payload) + "\n"
        sock.sendall(data.encode("utf-8"))
        log.info(f"Sent vector to UGV: d={vector_payload.get('distance_m', None):.2f}")
    except Exception as e:
        log.warning(f"Failed to send vector to UGV: {e}")


# ==========================================================
# CONTROL HELPERS
# ==========================================================

def compute_xy_velocity(err_x, err_y, forward_bias=0.0):
    """
    Control UGV marker to the center with optional forward bias in body frame.
    err_x, err_y in pixels (marker vs image center).
    forward_bias is a constant velocity added in +x (forward).
    """
    k = 0.003
    vx = -err_y * k + forward_bias
    vy = -err_x * k
    vx = np.clip(vx, -MAX_VEL, MAX_VEL)
    vy = np.clip(vy, -MAX_VEL, MAX_VEL)
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

    log.info(f"[TOUCHDOWN] alt_ok={alt_ok}, vs_ok={vs_ok}, "
             f"area_ok={area_ok}, var_ok={var_ok}")
    return alt_ok and vs_ok and area_ok and var_ok


# ==========================================================
# MAIN STATE MACHINE
# ==========================================================

def main():
    start_time = time.time()

    # State flags
    mode = "SEARCH_GOAL"   # -> "GUIDE_AND_DESCEND" -> "LANDING"
    stable_frames = 0

    accel_z_hist = collections.deque(maxlen=ACCEL_Z_VAR_WINDOW)
    last_distance_m = None

    # --- Connect to UGV vector listener ---
    ugv_sock = connect_to_ugv()
    if ugv_sock is None:
        log.warning("Proceeding without UGV vector channel (for testing only).")

    # --- UAV / vision setup ---
    log.info("Connecting to UAV flight controller...")
    uav = DroneConnect(connection_string="/dev/ttyACM0", baud=115200)
    uav.wait_ready()

    log.info("Starting Jetson camera...")
    cam = JetsonCamera(resolution=(640, 480), framerate=30)

    log.info("Initializing ArUco detector...")
    detector = ArucoDetector()

    log.info("Arming UAV...")
    uav.arm()

    log.info(f"Taking off to {TAKEOFF_ALT} m...")
    uav.takeoff(TAKEOFF_ALT)
    time.sleep(1.0)

    log.info("Entering Challenge 2 main control loop...")

    try:
        while True:
            # Mission timeout
            if time.time() - start_time > MISSION_TIMEOUT_SEC:
                log.error("MISSION TIMEOUT reached. Aborting and returning to safe state.")
                break

            frame = cam.read()
            if frame is None:
                log.warning("No camera frame; holding position.")
                uav.send_body_velocity(0, 0, 0)
                accel_z_hist.append(uav.get_accel_z())
                time.sleep(LOOP_DT)
                continue

            h, w = frame.shape[:2]
            cx_center = w / 2.0
            cy_center = h / 2.0

            detections = detector.detect_markers(frame)

            ugv_marker = next((d for d in detections if d["marker_id"] == UGV_MARKER_ID), None)
            goal_marker = next((d for d in detections if d["marker_id"] == GOAL_MARKER_ID), None)

            # If we ever reach LANDING mode: behave like Challenge 1 (focus only on UGV marker)
            if mode == "LANDING":
                if ugv_marker is None:
                    log.info("[LANDING] Lost UGV marker; hovering in place.")
                    uav.send_body_velocity(0, 0, 0)
                    accel_z_hist.append(uav.get_accel_z())
                    time.sleep(LOOP_DT)
                    continue

                cx, cy = ugv_marker["pixel"]
                area = ugv_marker["area"]
                err_x = cx - cx_center
                err_y = cy - cy_center

                vx, vy = compute_xy_velocity(err_x, err_y, forward_bias=0.0)
                vz = -LANDING_DESCENT_RATE

                uav.send_body_velocity(vx, vy, vz)

                alt = uav.get_altitude()
                vs = uav.get_vertical_speed()
                az = uav.get_accel_z()
                accel_z_hist.append(az)

                log.info(f"[LANDING TELEM] alt={alt:.2f}, vs={vs:.2f}, area={area:.1f}, "
                         f"err=({err_x:.1f},{err_y:.1f})")

                if touchdown_detected(alt, vs, area, accel_z_hist):
                    log.info("TOUCHDOWN detected (LANDING).")
                    break

                time.sleep(LOOP_DT)
                continue  # next loop

            # For SEARCH_GOAL / GUIDE_AND_DESCEND:
            if ugv_marker is None:
                log.info("[TRACK] UGV marker lost; hover.")
                uav.send_body_velocity(0, 0, 0)
                accel_z_hist.append(uav.get_accel_z())
                time.sleep(LOOP_DT)
                continue

            # Compute pixel error for UGV marker
            cx_u, cy_u = ugv_marker["pixel"]
            area_u = ugv_marker["area"]
            err_x = cx_u - cx_center
            err_y = cy_u - cy_center

            # Decide forward_bias based on mode
            if mode == "SEARCH_GOAL":
                forward_bias = SEARCH_FORWARD_VX
                vz = 0.0  # stay roughly level while searching
            elif mode == "GUIDE_AND_DESCEND":
                forward_bias = 0.0  # UGV is now moving toward goal; UAV mostly tracks above UGV
                vz = -GUIDE_DESCENT_RATE
            else:
                forward_bias = 0.0
                vz = 0.0

            vx, vy = compute_xy_velocity(err_x, err_y, forward_bias=forward_bias)

            # VECTOR CALCULATION if both markers visible
            if goal_marker is not None:
                vector_data = detector.compute_vector_between_markers(goal_marker, ugv_marker)
                last_distance_m = vector_data["distance_m"]

                # Build JSON payload
                payload = {
                    "cmd": "vector_to_goal",
                    "ugv_marker_id": vector_data["ugv_marker_id"],
                    "goal_marker_id": vector_data["goal_marker_id"],
                    "vector_ugv_to_goal": vector_data["vector_ugv_to_goal"],
                    "direction_unit": vector_data["direction_unit"],
                    "distance_m": vector_data["distance_m"],
                    "timestamp": vector_data["timestamp"]
                }
                send_vector_to_ugv(ugv_sock, payload)

                if mode == "SEARCH_GOAL":
                    log.info("Goal marker acquired. Switching to GUIDE_AND_DESCEND.")
                    mode = "GUIDE_AND_DESCEND"

                # Check for near-goal condition
                if mode == "GUIDE_AND_DESCEND" and last_distance_m is not None:
                    if last_distance_m < NEAR_GOAL_DISTANCE_M:
                        log.info(f"UGV near goal (d={last_distance_m:.2f} m). Switching to LANDING.")
                        mode = "LANDING"
                        # Don't descend aggressively yet; next loop handles landing state
            else:
                if mode == "GUIDE_AND_DESCEND":
                    log.info("Lost goal marker temporarily; still guiding from last known vector.")
                # In SEARCH_GOAL mode, this is normal until goal appears

            # Send body velocity command
            uav.send_body_velocity(vx, vy, vz)

            # Telemetry
            alt = uav.get_altitude()
            vs = uav.get_vertical_speed()
            az = uav.get_accel_z()
            accel_z_hist.append(az)

            log.info(f"[{mode}] alt={alt:.2f}, vs={vs:.2f}, "
                     f"UGV_err=({err_x:.1f},{err_y:.1f}), UGV_area={area_u:.1f}, "
                     f"d_goal={last_distance_m if last_distance_m is not None else -1:.2f}")

            time.sleep(LOOP_DT)

    finally:
        log.info("Stopping UAV motion and closing resources.")
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

        log.info("Challenge 2 AV mission finished.")


if __name__ == "__main__":
    main()
