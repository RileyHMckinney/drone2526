"""
akida_obstacle_module.py
------------------------
Handles obstacle detection using the Akida AKD1000 neuromorphic processor.

Supports:
  • Development mode (no hardware): mock device for testing
  • Deployment mode (on Raspberry Pi + Akida M.2 card): MetaTF / pretrained models

Typical use:
    from akida_obstacle_module import AkidaObstacleDetector
    detector = AkidaObstacleDetector(model_path="obstacle_model.akmodel")  # optional
    obstacle = detector.detect(frame)
"""

import cv2
import numpy as np
import random
import time
import os

try:
    # On-device: Raspberry Pi with Akida M.2 + MetaTF installed
    from akida import AkidaDevice, Model
    from akida_models import mobilenetv1_imagenet
    REAL_DEVICE = True
except ImportError:
    # On development machine (Windows/macOS)
    REAL_DEVICE = False

    class AkidaDevice:
        """Mock Akida device for local dev testing."""
        def __init__(self):
            print("[MockAkida] Initialized (no hardware detected)")

        def load_model(self, *args, **kwargs):
            print("[MockAkida] Loaded dummy model")

        def forward(self, data):
            # Randomly trigger obstacle ~15% of the time
            return np.array([[random.random() > 0.85]], dtype=np.float32)


class AkidaObstacleDetector:
    def __init__(self, model_path: str = None):
        """
        Initialize Akida obstacle detector.
        model_path: optional path to a local .akmodel file trained with MetaTF
        """
        print("[INFO] Initializing Akida obstacle detector...")
        self.device = AkidaDevice()
        self.model = None

        if REAL_DEVICE:
            try:
                # Load MetaTF-trained model if provided
                if model_path and os.path.exists(model_path):
                    print(f"[INFO] Loading MetaTF model from {model_path}")
                    self.model = Model(model_path)
                    self.device.load_model(self.model)
                    print("[INFO] MetaTF Akida model loaded successfully.")
                else:
                    # Fallback to built-in pretrained model for quick testing
                    print("[INFO] No .akmodel provided — using pretrained MobileNetV1.")
                    self.model = mobilenetv1_imagenet(pretrained=True)
                    self.device.load_model(self.model)
                    print("[INFO] Loaded pretrained MobileNetV1 on Akida device.")
            except Exception as e:
                print(f"[ERROR] Failed to load model on real device: {e}")
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


# === Stand-alone test ===
if __name__ == "__main__":
    # Optional: specify path to .akmodel here for on-device testing
    model_file = "obstacle_model.akmodel"

    detector = AkidaObstacleDetector(model_path=model_file)
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
