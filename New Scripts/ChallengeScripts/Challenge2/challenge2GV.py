"""
challenge2GV.py

Challenge 2 Ground Vehicle Controller
-------------------------------------
Vector following ONLY.
NO Akida obstacle avoidance.
NO camera-based obstacle override.

Supports two communication modes (select at runtime):
1) ESP32 UART (/dev/ttyUSB0)
2) Direct TCP (Jetson -> Pi)

Reads UGV->Goal vectors from UAV,
computes forward/turn motor commands,
stops when near the goal for UAV landing.
"""

import time
import json
import socket
import serial
import logging
import threading
import os

import numpy as np

# ==========================================================
# LOGGING
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

UART_PORT = "/dev/ttyUSB0"
UART_BAUD = 115200

TCP_HOST = "0.0.0.0"
TCP_PORT = 5006

MAX_NO_VECTOR_TIME = 10.0
LOOP_DT = 0.05

# Shared vector
latest_vector = {
    "vector": [0.0, 0.0, 0.0],
    "distance_m": None,
    "timestamp": 0.0
}
vector_lock = threading.Lock()


# ==========================================================
# MOTOR CONTROL PLACEHOLDERS
# ==========================================================

def motors_forward(speed=0.3):
    log.info(f"[MOTOR] Forward {speed:.2f}")


def motors_turn_left(speed=0.2):
    log.info(f"[MOTOR] Turn LEFT {speed:.2f}")


def motors_turn_right(speed=0.2):
    log.info(f"[MOTOR] Turn RIGHT {speed:.2f}")


def motors_stop():
    log.info("[MOTOR] STOP")


# ==========================================================
# COMMUNICATION LISTENERS
# ==========================================================

def uart_listener():
    global latest_vector
    try:
        ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
        log.info(f"[UART] Listening on {UART_PORT}")
    except Exception as e:
        log.error(f"[UART] Failed to open port: {e}")
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
                    latest_vector["distance_m"] = data.get("distance_m", None)
                    latest_vector["timestamp"] = time.time()

        except Exception as e:
            log.warning(f"[UART ERROR] {e}")
            time.sleep(0.1)


def tcp_listener():
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
            buffer = ""
            while True:
                raw = conn.recv(4096)
                if not raw:
                    break

                buffer += raw.decode("utf-8", errors="ignore")

                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue

                    log.info(f"[TCP RAW] {line}")
                    try:
                        data = json.loads(line)
                        if "vector_ugv_to_goal" in data:
                            with vector_lock:
                                latest_vector["vector"] = data["vector_ugv_to_goal"]
                                latest_vector["distance_m"] = data.get("distance_m", None)
                                latest_vector["timestamp"] = time.time()
                    except json.JSONDecodeError:
                        log.warning(f"[TCP] Invalid JSON: {line}")

        finally:
            conn.close()
            log.info("[TCP] Connection closed")


# ==========================================================
# VECTOR INTERPRETATION
# ==========================================================

def interpret_vector(vec):
    """
    vec = [x, y, z]
    x = forward magnitude
    y = lateral steering magnitude
    """
    x, y, _ = vec

    THRESH_FORWARD = 0.1
    THRESH_TURN = 0.05

    if abs(x) < THRESH_FORWARD and abs(y) < THRESH_TURN:
        motors_stop()
        return

    if x > 0:
        motors_forward(min(0.5, abs(x)))
    elif x < 0:
        motors_stop()

    if y > THRESH_TURN:
        motors_turn_right(min(0.4, abs(y)))
    elif y < -THRESH_TURN:
        motors_turn_left(min(0.4, abs(y)))


# ==========================================================
# MAIN LOOP
# ==========================================================

def main():
    print("Select communication mode (Challenge 2 GV):")
    print("1 → ESP32 UART")
    print("2 → Direct TCP")
    mode = input("Enter 1 or 2: ").strip()

    if mode == "1":
        log.info("Using ESP32 UART mode.")
        comm_thread = threading.Thread(target=uart_listener, daemon=True)
    else:
        log.info("Using direct TCP mode.")
        comm_thread = threading.Thread(target=tcp_listener, daemon=True)

    comm_thread.start()

    try:
        while True:
            with vector_lock:
                vec = latest_vector["vector"]
                dist = latest_vector["distance_m"]
                ts = latest_vector["timestamp"]

            if time.time() - ts > MAX_NO_VECTOR_TIME:
                log.warning("No recent vector; stopping.")
                motors_stop()
                time.sleep(LOOP_DT)
                continue

            if dist is not None and dist < 1.0:
                log.info(f"[NEAR GOAL] d={dist:.2f} m. Stopping for UAV landing.")
                motors_stop()
                time.sleep(LOOP_DT)
                continue

            interpret_vector(vec)
            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        log.info("Stopping GV (user interrupt).")
    finally:
        motors_stop()


if __name__ == "__main__":
    main()
