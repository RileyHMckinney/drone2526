"""
IntegratedUAVcode.py
--------------------
UAV main control script for the Raytheon AVC project (2025).

This program runs on the Jetson companion computer and handles:
  • Establishing a DroneKit connection to the Pixhawk flight controller
  • Safely arming and taking off with GPS enabled for failsafe (RTL/fencing)
  • Capturing camera frames and detecting ArUco markers in real time
  • Estimating each marker’s relative position vector (x, y, z) in meters
  • Sending those relative vectors as JSON packets over serial to the UAV ESP32

The ESP32 then relays this data to the UGV system for coordinated navigation.
GPS is retained for safety but is not used for navigation or target localization.
"""

from __future__ import print_function
import time
import json
import math
from serial import Serial
import cv2
import cv2.aruco as aruco
import numpy as np
from dronekit import connect, Vehicle, VehicleMode  # Pixhawk communication

# Project imports
from DroneConnect import connectMyCopter, arm_drone, takeoff_drone
# from DroneCode.CameraProcess import *     # optional if you separate camera code

# --- Legacy modules (commented out) ---
# import multiprocessing                    # only needed for parallelized camera/comms
# import csv                                # old waypoint generation
# from SearchAlgoScript import *            # old GPS-based search logic
# from DroneProcess import *                # older helper functions
# from math import radians, cos, sin, sqrt, atan2, atan, tan   # legacy trig utilities

# === UAV SETUP ===

MARKER_SIZE = 0.1  # meters
CALIBRATION_FILE_PATH = "calibration_chessboard.yaml"
SERIAL_PORT = "/dev/ttyUSB0"
SERIAL_BAUD = 115200
SIMULATE_DRONE = False

# 1. Connect to Pixhawk
print("[INFO] Connecting to UAV...")
vehicle = connectMyCopter(SIMULATE_DRONE)
arm_drone(vehicle)
takeoff_drone(vehicle, targetAltitude=2.0)
print("[INFO] UAV armed, airborne, and stable.")

# 2. Load calibration
print("[INFO] Loading camera calibration parameters...")
cv_file = cv2.FileStorage(CALIBRATION_FILE_PATH, cv2.FILE_STORAGE_READ)
camera_matrix = cv_file.getNode("K").mat()
dist_coeffs = cv_file.getNode("D").mat()
cv_file.release()
if camera_matrix is None or dist_coeffs is None:
    raise RuntimeError("[ERROR] Failed to load calibration file.")

# 3. Initialize camera
print("[INFO] Initializing camera feed...")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
if not cap.isOpened():
    raise RuntimeError("[ERROR] Unable to open camera.")

# 4. Initialize serial connection to ESP32
print(f"[INFO] Opening serial connection to ESP32 on {SERIAL_PORT}...")
ser = Serial(SERIAL_PORT, SERIAL_BAUD, timeout=1)
time.sleep(2)
if not ser.is_open:
    raise RuntimeError("[ERROR] Serial connection failed.")
print("[INFO] Serial link established.")

# === MAIN LOOP: detect markers and send relative vectors ===
print("[INFO] Entering detection loop. Press 'q' to quit.")

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
parameters = cv2.aruco.DetectorParameters()

while True:
    ret, frame = cap.read()
    if not ret:
        continue

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Draw markers for visualization
        aruco.drawDetectedMarkers(frame, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]],  # wrap in list to satisfy type checker
                MARKER_SIZE,
                camera_matrix,
                dist_coeffs
            )
            rvec, tvec = rvecs[0], tvecs[0]

            # Convert OpenCV tvec (X=right, Y=down, Z=forward) to BODY_ENU
            rel_x = float(tvec[0][0][2])   # forward
            rel_y = float(tvec[0][0][0])   # right
            rel_z = float(-tvec[0][0][1])  # up

            # Package into JSON for UGV
            data_dict = {
                "type": "relative_target",
                "source": "uav",
                "frame": "BODY_ENU",
                "marker_id": int(marker_id),
                "x": rel_x,
                "y": rel_y,
                "z": rel_z
            }

            json_str = json.dumps(data_dict)
            ser.write((json_str + "\n").encode("utf-8"))
            print(f"[SEND] {json_str}")

            # Draw coordinate axes for visual debugging
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

    cv2.imshow("UAV Camera Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# --- Cleanup ---
cap.release()
ser.close()
cv2.destroyAllWindows()
print("[INFO] Detection loop terminated, camera and serial closed.")
