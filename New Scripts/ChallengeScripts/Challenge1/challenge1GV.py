"""
challenge1GV.py
Author: Riley McKinney

UGV Listener Script for Challenge 1
-----------------------------------
- Listens for "start" and "stop" commands over TCP
- On "start": drives forward at a fixed speed (>= 0.2 mph)
- On "stop": stops motors
- Includes logging, reconnect behavior, and fallback drive timeout
"""

import socket
import time
import logging
import os


# ==========================================================
# LOGGING
# ==========================================================

LOG_FILE = os.path.join(os.path.dirname(__file__), "challenge1GV.log")
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [GV] %(levelname)s: %(message)s",
    handlers=[
        logging.FileHandler(LOG_FILE),
        logging.StreamHandler()
    ]
)
log = logging.getLogger("Challenge1GV")


# ==========================================================
# NETWORK / BEHAVIOR PARAMETERS
# ==========================================================

LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005

# How long the UGV is allowed to drive without a STOP before auto-stop (seconds)
MAX_DRIVE_DURATION = 7 * 60  # 7 minutes


# ==========================================================
# MOTOR CONTROL PLACEHOLDERS
# ==========================================================

def motors_forward():
    """
    TODO: Replace this with actual motor driver code.
    Example for future:
        - Send serial command to Roboclaw
        - Set GPIO PWM for L298N
        - Publish to ROS node, etc.
    """
    log.info("[MOTORS] Driving forward...")


def motors_stop():
    """
    TODO: Replace this with actual motor driver stop code.
    """
    log.info("[MOTORS] STOP.")


# ==========================================================
# MAIN SERVER LOOP
# ==========================================================

def handle_client(conn, addr):
    log.info(f"Client connected from {addr}")
    driving = False
    drive_start_time = None

    try:
        while True:
            data = conn.recv(1024)
            if not data:
                # Client disconnected
                log.info("Client disconnected.")
                break

            msg = data.decode("utf-8").strip()
            log.info(f"Received command: '{msg}'")

            if msg == "start" and not driving:
                log.info("START command received. Beginning forward motion.")
                motors_forward()
                driving = True
                drive_start_time = time.time()

            elif msg == "stop" and driving:
                log.info("STOP command received. Stopping motion.")
                motors_stop()
                driving = False
                drive_start_time = None

            # Fallback timeout if driving too long without STOP
            if driving and drive_start_time is not None:
                elapsed = time.time() - drive_start_time
                if elapsed > MAX_DRIVE_DURATION:
                    log.warning("MAX_DRIVE_DURATION exceeded! Auto-stopping motors.")
                    motors_stop()
                    driving = False
                    drive_start_time = None

            time.sleep(0.05)

    except Exception as e:
        log.warning(f"Exception in client handler: {e}")
    finally:
        if driving:
            log.info("Client ended while driving. Stopping motors as safety measure.")
            motors_stop()
        conn.close()
        log.info("Client handler finished.")


def main():
    log.info(f"Starting UGV TCP listener on {LISTEN_IP}:{LISTEN_PORT}")
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((LISTEN_IP, LISTEN_PORT))
    sock.listen(1)
    log.info("UGV listener ready for connections.")

    try:
        while True:
            log.info("Waiting for new connection...")
            conn, addr = sock.accept()
            handle_client(conn, addr)

    except KeyboardInterrupt:
        log.info("KeyboardInterrupt received. Shutting down listener.")
    finally:
        motors_stop()
        sock.close()
        log.info("UGV listener terminated.")


if __name__ == "__main__":
    main()
