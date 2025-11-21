"""
challenge2GV.py
Author: Riley McKinney

Challenge 2 Ground Vehicle Controller
-------------------------------------
This script supports TWO communication modes:

1) ESP32 UART Mode (existing pipeline):
   UAV → WiFi → ESP32 → UART → Pi
   Receives JSON through /dev/ttyUSB0 exactly like IntegratedGV.py

2) Direct TCP Mode (recommended):
   UAV → WiFi TCP → Pi
   Receives JSON via socket directly from challenge2AV.py

The script:
• Receives vector updates from the UAV (UGV→Goal vector)
• Runs Akida-based obstacle avoidance in parallel
• Computes forward/turn signals for UGV
• Provides clean structure for future motor driver integration
• Contains safety timeouts, logging, and reconnect logic
"""

import time
import json
import socket
import serial
import logging
import threading
import sys
import os
import cv2
import numpy as np

# Obstacle avoidance modules (from your repo)
from akida_obstacle_module import AkidaObstacleDetector
from obstacle_avoidance import ObstacleAvoidance


# ==========================================================
# LOGGING SETUP
# ==========================================================
LOG_FILE = os.path.join(os.path.dirname(__file__), "challenge2GV.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [C2-GV] %(levelname)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("Challenge2GV")


# ==========================================================
# CONFIGURATION
# ==========================================================

# UART config (ESP32 path)
UART_PORT = "/dev/ttyUSB0"
UART_BAUD = 115200

# TCP config (Jetson direct-to-Pi path)
TCP_HOST = "0.0.0.0"
TCP_PORT = 5006

# Safety timeouts
MAX_NO_VECTOR_TIME = 10.0   # seconds before UGV auto-stop
LOOP_DT = 0.05              # 20 Hz control loop

# Obstacle Avoidance
avoidance = ObstacleAvoidance(debug=True)

# Vector storage (shared between threads)
latest_vector = {
    "vector": [0, 0, 0],
    "distance_m": None,
    "timestamp": 0
}
vector_lock = threading.Lock()


# ==========================================================
# MOTOR CONTROL PLACEHOLDERS
# ==========================================================
def motors_forward(speed=0.3):
    log.info(f"[MOTOR] Forward at speed {speed}")


def motors_turn_left(speed=0.2):
    log.info(f"[MOTOR] Turn LEFT at {speed}")


def motors_turn_right(speed=0.2):
    log.info(f"[MOTOR] Turn RIGHT at {speed}")


def motors_stop():
    log.info("[MOTOR] STOP all motors")


# ==========================================================
# COMMUNICATION HANDLERS
# ==========================================================

def uart_listener():
    """Listen for vector JSON arriving over UART from ESP32."""
    global latest_vector

    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        log.info(f"[UART] Listening on {UART_PORT} at {UART_BAUD}")
    except Exception as e:
        log.error(f"UART open failed: {e}")
        return

    while True:
        try:
            raw = ser.readline().decode("utf-8", errors="ignore").strip()
            if not raw:
                continue

            log.info(f"[UART RAW] {raw}")
            data = json.loads(raw)

            if "vector_ugv_to_goal" in data:
                with vector_lock:
                    latest_vector["vector"] = data["vector_ugv_to_goal"]
                    latest_vector["distance_m"] = data["distance_m"]
                    latest_vector["timestamp"] = time.time()

        except Exception as e:
            log.warning(f"[UART ERROR] {e}")
            time.sleep(0.1)


def tcp_listener():
    """Listen for vector JSON from the UAV via direct TCP."""
    global latest_vector

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((TCP_HOST, TCP_PORT))
    sock.listen(1)

    log.info(f"[TCP] Listening on {TCP_HOST}:{TCP_PORT}")

    while True:
        conn, addr = sock.accept()
        log.info(f"[TCP] Connected by {addr}")

        try:
            while True:
                raw = conn.recv(4096)
                if not raw:
                    break

                try:
                    text = raw.decode("utf-8").strip()
                    log.info(f"[TCP RAW] {text}")
                    data = json.loads(text)

                    if "vector_ugv_to_goal" in data:
                        with vector_lock:
                            latest_vector["vector"] = data["vector_ugv_to_goal"]
                            latest_vector["distance_m"] = data["distance_m"]
                            latest_vector["timestamp"] = time.time()

                except Exception as e:
                    log.warning(f"[TCP PARSE ERROR] {e}")

        finally:
            conn.close()
            log.info("[TCP] Connection closed")


# ==========================================================
# MOTION INTERPRETATION
# ==========================================================
def interpret_vector(vec):
    """
    Vector format from challenge2AV.py:
    vec = [x, y, z]
    x → forward/back
    y → left/right
    z → unused for ground motion
    """
    x, y, _ = vec

    forward_mag = x
    turn_mag = y

    # Threshold tuning
    THRESH_FORWARD = 0.1
    THRESH_TURN = 0.05

    if abs(forward_mag) < THRESH_FORWARD and abs(turn_mag) < THRESH_TURN:
        motors_stop()
        return

    if forward_mag > 0:
        motors_forward(min(0.5, abs(forward_mag)))
    elif forward_mag < 0:
        motors_stop()  # Don't reverse; just stop for safety

    if turn_mag > THRESH_TURN:
        motors_turn_right(min(0.4, abs(turn_mag)))
    elif turn_mag < -THRESH_TURN:
        motors_turn_left(min(0.4, abs(turn_mag)))


# ==========================================================
# MAIN CONTROL LOOP
# ==========================================================

def main():
    print("Select communication mode:")
    print("1 → ESP32 UART (serial)")
    print("2 → Direct TCP (recommended)")
    mode = input("Enter 1 or 2: ").strip()

    if mode == "1":
        log.info("Using ESP32 UART mode.")
        comm_thread = threading.Thread(target=uart_listener, daemon=True)
    else:
        log.info("Using direct TCP mode.")
        comm_thread = threading.Thread(target=tcp_listener, daemon=True)

    comm_thread.start()

    log.info("Starting camera for obstacle avoidance...")
    cap = cv2.VideoCapture(0)

    last_vector_time = 0

    try:
        while True:
            # --- 1. Obstacle avoidance ---
            ret, frame = cap.read()
            if ret:
                decision = avoidance.process_frame(frame)
                frame = avoidance.visualize(frame, decision)

                if decision != "CLEAR":
                    log.info(f"[AVOID] Obstacle {decision} → overriding motion")
                    motors_stop()
                    cv2.imshow("Obstacle Avoidance", frame)
                    cv2.waitKey(1)
                    time.sleep(0.1)
                    continue

                cv2.imshow("Obstacle Avoidance", frame)
                cv2.waitKey(1)

            # --- 2. Retrieve latest vector ---
            with vector_lock:
                vec = latest_vector["vector"]
                dist = latest_vector["distance_m"]
                last_ts = latest_vector["timestamp"]

            # --- 3. Safety: timeout if no vector received ---
            if time.time() - last_ts > MAX_NO_VECTOR_TIME:
                motors_stop()
                log.warning("No vector received recently; stopping motors.")
                time.sleep(LOOP_DT)
                continue

            # --- 4. If near goal, stop for landing ---
            if dist is not None and dist < 1.0:
                log.info(f"[NEAR GOAL] d={dist:.2f} m → stopping for AV landing")
                motors_stop()
                time.sleep(0.2)
                continue

            # --- 5. Normal vector interpretation ---
            interpret_vector(vec)

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        log.info("User interrupt — stopping motors.")
    finally:
        motors_stop()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
