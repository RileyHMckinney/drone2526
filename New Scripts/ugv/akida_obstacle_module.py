"""
akida_obstacle_module.py
------------------------
Handles obstacle detection using the Akida AKD1000 neuromorphic processor.

This file works in two modes:
  • Development mode (no hardware): uses a MockAkidaDevice that randomly detects obstacles.
  • Deployment mode (on Raspberry Pi + Akida): uses the real Akida SDK.

Typical use:
    from akida_obstacle_module import AkidaObstacleDetector
    detector = AkidaObstacleDetector()
    obstacle = detector.detect(frame)
"""

import cv2
import numpy as np
import random
import time

try:
    # On the Raspberry Pi this will succeed
    from akida import AkidaDevice
    from akida_models import mobilenetv1_imagenet
    REAL_DEVICE = True
except ImportError:
    # On Windows dev machines this mock lets you simulate
    REAL_DEVICE = False

    class AkidaDevice:
        def __init__(self):
            print("[MockAkida] Initialized (no hardware detected)")

        def load_model(self, *args, **kwargs):
            print("[MockAkida] Loaded dummy model")

        def forward(self, data):
            # Randomly trigger obstacle ~15% of the time
            return np.array([[random.random() > 0.85]], dtype=np.float32)


class AkidaObstacleDetector:
    def __init__(self):
        print("[INFO] Initializing Akida obstacle detector...")
        self.device = AkidaDevice()
        self.model = None

        if REAL_DEVICE:
            # Use a lightweight pretrained model for feature extraction
            from akida_models import mobilenetv1_imagenet
            self.model = mobilenetv1_imagenet(pretrained=True)
            self.device.load_model(self.model)
            print("[INFO] Loaded pretrained MobilenetV1 on Akida device.")
        else:
            self.device.load_model("mock_model")

        print("[INFO] Akida obstacle detector ready.\n")

    def preprocess(self, frame):
        """Resize and normalize image for model input."""
        img = cv2.resize(frame, (224, 224))
        img = np.expand_dims(img / 255.0, axis=0).astype(np.float32)
        return img

    def detect(self, frame) -> bool:
        """
        Run inference and return True if an obstacle is detected.
        """
        input_data = self.preprocess(frame)
        preds = self.device.forward(input_data)
        detected = bool(preds[0][0] > 0.5)

        if detected:
            print("[AKIDA] 🚧 Obstacle detected!")
        else:
            print("[AKIDA] Clear path.")
        return detected


# Stand-alone test
if __name__ == "__main__":
    detector = AkidaObstacleDetector()
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        raise RuntimeError("Camera not found")

    print("Press 'q' to quit.")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        _ = detector.detect(frame)
        cv2.imshow("Akida Obstacle Test", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        time.sleep(0.1)

    cap.release()
    cv2.destroyAllWindows()
