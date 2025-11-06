# New Scripts/uav_ugv_relay/test_uav_sender.py
import argparse, json, random, socket, sys, time
from typing import Optional

try:
    import serial  # optional; only for --mode serial
except Exception:
    serial = None

# Optional ArUco import from your repo (uav/aruco_detector.py)
def _try_load_aruco_detector(calib_path: str, marker_size_m: float):
    """
    Attempts to import uav.aruco_detector.ArucoDetector from project root.
    Returns (detector_instance, errstr). If errstr is not None, ArUco is unavailable.
    """
    try:
        import os
        # Add project root to sys.path so "uav" package can be found
        proj_root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
        if proj_root not in sys.path:
            sys.path.append(proj_root)

        from uav.aruco_detector import ArucoDetector  # your teammate's file
        det = ArucoDetector(calibration_path=calib_path, marker_size=marker_size_m)
        return det, None
    except Exception as e:
        return None, f"[WARN] ArUco unavailable ({e}). Falling back to random source."

def _map_camvec_to_body_enu(vec_xyz):
    """
    Map camera-frame vector to BODY_ENU (x=fwd, y=right, z=up).
    This mapping depends on camera mounting. For a typical down-facing camera:
      - forward_body ~ +Z_cam
      - right_body   ~ +X_cam
      - up_body      ~ -Y_cam
    Adjust if your physical mounting differs.
    """
    x_cam, y_cam, z_cam = vec_xyz  # from detector (meters)
    x_body =  z_cam
    y_body =  x_cam
    z_body = -y_cam
    return float(x_body), float(y_body), float(z_body)

def build_msg_random(seq=0):
    # Simulate BODY_ENU offsets (x=fwd, y=right, z=up)
    x = random.uniform(-0.5, 0.5)
    y = random.uniform(-0.5, 0.5)
    z = random.uniform( 0.2, 0.8)
    return {
        "type": "relative_target",
        "marker_id": 42,
        "frame": "BODY_ENU",
        "x": x, "y": y, "z": z,
        "timestamp": int(time.time()*1000),
        "seq": seq
    }

def build_msg_from_aruco(detector, cap, ugv_id: int, goal_id: int, seq: int) -> Optional[dict]:
    import cv2
    ok, frame = cap.read()
    if not ok:
        return None
    detections = detector.detect_markers(frame)
    ugv = next((d for d in detections if d["marker_id"] == ugv_id), None)
    goal = next((d for d in detections if d["marker_id"] == goal_id), None)
    if not (ugv and goal):
        return None

    vec = detector.compute_vector_between_markers(goal, ugv)["vector_ugv_to_goal"]  # [dx,dy,dz] in camera frame
    x_b, y_b, z_b = _map_camvec_to_body_enu(vec)
    return {
        "type": "relative_target",
        "marker_id": ugv_id,
        "frame": "BODY_ENU",
        "x": x_b, "y": y_b, "z": z_b,
        "timestamp": int(time.time()*1000),
        "seq": seq
    }

def run_tcp(host, port, rate_hz, source, calib, marker_size, ugv_id, goal_id, cam_index):
    print(f"[TCP] Connecting to {host}:{port} …")
    with socket.create_connection((host, port), timeout=5) as s:
        print("[TCP] Connected.")
        seq = 0
        period = 1.0 / max(1.0, rate_hz)

        aruco_det = None
        cap = None

        if source == "aruco":
            aruco_det, err = _try_load_aruco_detector(calib, marker_size)
            if err:
                print(err)
                source = "random"
            else:
                import cv2
                cap = cv2.VideoCapture(cam_index)
                if not cap.isOpened():
                    print("[WARN] Camera open failed; falling back to random.")
                    source = "random"

        try:
            while True:
                if source == "random":
                    msg = build_msg_random(seq)
                else:  # aruco
                    msg = build_msg_from_aruco(aruco_det, cap, ugv_id, goal_id, seq)
                    if msg is None:
                        # No markers this frame—don’t spam; just skip send this cycle
                        time.sleep(period)
                        continue

                line = (json.dumps(msg) + "\n").encode("utf-8")
                s.sendall(line)
                print(f"[SEND] {msg}")
                seq += 1
                time.sleep(period)
        finally:
            if cap is not None:
                cap.release()

def run_serial(port, baud, rate_hz):
    if serial is None:
        print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        sys.exit(1)
    print(f"[SERIAL] Opening {port} @ {baud} …")
    with serial.Serial(port, baud, timeout=1) as ser:
        print("[SERIAL] Connected. Sending lines …")
        seq = 0
        period = 1.0 / max(1.0, rate_hz)
        while True:
            msg = build_msg_random(seq)
            ser.write((json.dumps(msg) + "\n").encode("utf-8"))
            print(f"[SEND] {msg}")
            seq += 1
            time.sleep(period)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["tcp","serial"], default="tcp")
    ap.add_argument("--host", default="127.0.0.1")                     # tcp
    ap.add_argument("--port", type=int, default=5761)                  # tcp
    ap.add_argument("--serial-port", default="COM8")                   # serial
    ap.add_argument("--baud", type=int, default=115200)                # serial
    ap.add_argument("--rate-hz", type=float, default=2.0)

    # Source selection
    ap.add_argument("--source", choices=["random","aruco"], default="random")
    ap.add_argument("--calib", default="config/calibration_chessboard.yaml")
    ap.add_argument("--marker-size-m", type=float, default=0.254)      # 10-inch inner square ≈ 0.254 m
    ap.add_argument("--ugv-id", type=int, default=1)
    ap.add_argument("--goal-id", type=int, default=0)
    ap.add_argument("--camera-index", type=int, default=0)

    args = ap.parse_args()

    if args.mode == "tcp":
        run_tcp(args.host, args.port, args.rate_hz, args.source,
                args.calib, args.marker_size_m, args.ugv_id, args.goal_id, args.camera_index)
    else:
        run_serial(args.serial_port, args.baud, args.rate_hz)

if __name__ == "__main__":
    main()
