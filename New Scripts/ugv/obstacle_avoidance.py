"""
obstacle_avoidance.py
---------------------
Decision layer between Akida obstacle detection and UGV motion.

This module:
  • Takes frames from the Pi camera
  • Uses Akida inference to detect obstacles
  • Determines whether the obstacle is LEFT / CENTER / RIGHT in the frame
  • Suggests steering or stop commands accordingly

Typical use:
    from obstacle_avoidance import ObstacleAvoidance
    avoidance = ObstacleAvoidance()
    result = avoidance.process_frame(frame)
    # result -> one of ["CLEAR", "LEFT", "RIGHT", "STOP"]
"""

import cv2
import numpy as np
import time
from akida_obstacle_module import AkidaObstacleDetector


class ObstacleAvoidance:
    def __init__(self, debug: bool = True):
        print("[INFO] Initializing obstacle avoidance module...")
        self.detector = AkidaObstacleDetector()
        self.state = "CLEAR"
        self.debug = debug
        self.last_decision_time = 0
        self.decision_cooldown = 1.0  # seconds between major decision updates

    # === IMAGE ANALYSIS HELPERS ===

    def _localize_obstacle(self, frame):
        """
        Use simple computer vision (color/brightness thresholding)
        to approximate where the obstacle is in the image.
        Returns: "LEFT", "CENTER", "RIGHT" or None if no dominant region.
        """
        # Convert to grayscale and blur for stability
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (7, 7), 0)

        # Adaptive threshold to segment bright/dark objects
        _, mask = cv2.threshold(blurred, 120, 255, cv2.THRESH_BINARY_INV)

        # Find contours of potential obstacles
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Choose largest contour (most likely obstacle)
        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < 500:  # ignore small blobs
            return None

        # Compute centroid
        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None
        cx = int(M["m10"] / M["m00"])

        # Determine position relative to frame width
        width = frame.shape[1]
        third = width // 3
        if cx < third:
            return "LEFT"
        elif cx > 2 * third:
            return "RIGHT"
        else:
            return "CENTER"

    # === MAIN PROCESSOR ===

    def process_frame(self, frame):
        """
        Main obstacle analysis routine.
        Returns one of: "CLEAR", "LEFT", "RIGHT", "STOP"
        """

        # Step 1: Inference with Akida model
        obstacle_detected = self.detector.detect(frame)

        # Step 2: If obstacle detected, determine where
        if obstacle_detected:
            region = self._localize_obstacle(frame)
            decision = "STOP" if region is None else region

            # Avoid excessive flickering decisions
            if time.time() - self.last_decision_time > self.decision_cooldown:
                self.last_decision_time = time.time()
                self.state = "AVOIDING"
                if self.debug:
                    print(f"[AVOID] Obstacle detected in {region or 'unknown region'} — "
                          f"Decision: {decision}")

            return decision

        else:
            # No obstacle — ensure we transition back to clear state
            if self.state != "CLEAR":
                if self.debug:
                    print("[AVOID] Path clear — resuming normal navigation.")
                self.state = "CLEAR"
            return "CLEAR"

    # === OPTIONAL VISUALIZATION ===

    def visualize(self, frame, decision):
        """
        Overlays decision info on the frame for debugging.
        """
        overlay = frame.copy()
        h, w, _ = frame.shape

        # Highlight region based on decision
        if decision == "LEFT":
            cv2.rectangle(overlay, (0, 0), (w//3, h), (0, 0, 255), -1)
        elif decision == "RIGHT":
            cv2.rectangle(overlay, (2*w//3, 0), (w, h), (0, 0, 255), -1)
        elif decision == "CENTER":
            cv2.rectangle(overlay, (w//3, 0), (2*w//3, h), (0, 0, 255), -1)

        alpha = 0.3
        cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)
        cv2.putText(frame, f"Decision: {decision}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        return frame


# === Standalone Test ===
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    avoidance = ObstacleAvoidance(debug=True)

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        decision = avoidance.process_frame(frame)
        frame = avoidance.visualize(frame, decision)
        cv2.imshow("Obstacle Avoidance Test", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
