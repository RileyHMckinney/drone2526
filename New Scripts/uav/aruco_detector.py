"""
aruco_detector.py
-----------------
Detects multiple ArUco markers from a downward-facing monocular camera,
estimates their positions (X, Y, Z) relative to the UAV, and computes the
straight-line vector and distance between a UGV marker and a goal marker.

Updated to also provide:
    - pixel center (cx, cy) of each marker
    - marker area in pixels^2
    - helper method detect_single_marker(frame, marker_id)
"""

import cv2
import numpy as np
import math
from collections import deque
import time
import json

# === Configuration ===
CALIBRATION_FILE_PATH = "config/calibration_chessboard.yaml"
MARKER_SIZE = 0.254  # meters (10-inch inner black square)
ROLLING_WINDOW = 5   # smoothing window for pose averaging
GOAL_MARKER_ID = 0   # marker ID on the goal target
UGV_MARKER_ID = 1    # marker ID on the ground vehicle


# === Helper ===
def load_calibration(file_path):
    fs = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    K = fs.getNode("K").mat()
    D = fs.getNode("D").mat()
    fs.release()
    if K is None or D is None:
        raise ValueError("Calibration file missing K or D matrix.")
    return K, D


# === Detector Class ===
class ArucoDetector:
    def __init__(self,
                 calibration_path=CALIBRATION_FILE_PATH,
                 marker_size=MARKER_SIZE,
                 smoothing_window=ROLLING_WINDOW):

        self.camera_matrix, self.dist_coeffs = load_calibration(calibration_path)
        self.marker_size = marker_size

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
        self.detector_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.detector_params)

        # Rolling average buffer per marker_id
        self.position_buffers = {}  # marker_id → deque of tvecs

    # --- Utility ---
    @staticmethod
    def rotation_matrix_to_euler(R):
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6
        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0
        return [math.degrees(a) for a in [x, y, z]]

    # --- Core ---
    def detect_markers(self, frame):
        """
        Detect all visible markers and return structured pose data as a list of dicts.

        Each dict contains:
            - marker_id: int
            - position_m: [x, y, z] in meters (smoothed tvec)
            - rotation_deg: [roll, pitch, yaw] in degrees
            - pixel: [cx, cy] center in image coordinates
            - area: float, marker area in pixel^2
            - timestamp: float (time.time())
        """
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)
        detections = []

        if ids is None:
            self.position_buffers.clear()
            return detections

        for i, marker_id in enumerate(ids.flatten()):
            # Pose estimation
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                [corners[i]], self.marker_size, self.camera_matrix, self.dist_coeffs
            )
            tvec = tvec[0][0]
            rvec = rvec[0][0]

            # Rolling average per marker
            buf = self.position_buffers.setdefault(marker_id, deque(maxlen=ROLLING_WINDOW))
            buf.append(tvec)
            smoothed_tvec = np.mean(buf, axis=0)

            # Rotation to Euler
            R, _ = cv2.Rodrigues(rvec)
            euler = self.rotation_matrix_to_euler(R)

            # Pixel center and area
            pts = corners[i][0]  # shape (4, 2)
            cx = float(np.mean(pts[:, 0]))
            cy = float(np.mean(pts[:, 1]))
            area = float(cv2.contourArea(pts.astype(np.float32)))

            # Draw markers for debug
            cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
            cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.05)

            detections.append({
                "marker_id": int(marker_id),
                "position_m": smoothed_tvec.tolist(),
                "rotation_deg": euler,
                "pixel": [cx, cy],
                "area": area,
                "timestamp": time.time()
            })

        return detections

    def detect_single_marker(self, frame, target_marker_id):
        """
        Convenience method: detect markers and return the dict for a specific marker ID.
        Returns None if not found.
        """
        detections = self.detect_markers(frame)
        for d in detections:
            if d["marker_id"] == target_marker_id:
                return d
        return None

    # --- Vector Computation ---
    @staticmethod
    def compute_vector_between_markers(goal_marker, ugv_marker):
        """Compute the 3D vector and distance from UGV → Goal."""

        t_goal = np.array(goal_marker["position_m"])
        t_ugv = np.array(ugv_marker["position_m"])

        vector = t_goal - t_ugv
        distance = float(np.linalg.norm(vector))
        direction = (vector / distance).tolist() if distance > 1e-6 else [0, 0, 0]

        return {
            "goal_marker_id": goal_marker["marker_id"],
            "ugv_marker_id": ugv_marker["marker_id"],
            "vector_ugv_to_goal": vector.tolist(),
            "direction_unit": direction,
            "distance_m": distance,
            "timestamp": time.time()
        }


# === Simple Test Loop ===
def main():
    detector = ArucoDetector()
    cap = cv2.VideoCapture(0)

    print("Starting multi-marker ArUco detection (press 'q' to quit).")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        detections = detector.detect_markers(frame)

        # Find the goal and UGV markers in the detections
        goal_marker = next((d for d in detections if d["marker_id"] == GOAL_MARKER_ID), None)
        ugv_marker  = next((d for d in detections if d["marker_id"] == UGV_MARKER_ID), None)

        if goal_marker and ugv_marker:
            vector_data = detector.compute_vector_between_markers(goal_marker, ugv_marker)
            print(json.dumps(vector_data, indent=2))

        # Debug: show centers and area
        for d in detections:
            cx, cy = d["pixel"]
            area = d["area"]
            cv2.circle(frame, (int(cx), int(cy)), 4, (0, 255, 0), -1)
            cv2.putText(frame,
                        f"ID {d['marker_id']} A={int(area)}",
                        (int(cx) - 40, int(cy) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.imshow("Aruco Detector", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
