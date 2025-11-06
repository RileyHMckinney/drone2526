# New Scripts/uav_ugv_relay/test_ugv_receiver.py
import argparse, json, socket, sys, time

try:
    import serial  # optional; for --motor-serial
except Exception:
    serial = None

FORWARD_THRESHOLD = 0.10   # meters
TURN_THRESHOLD    = 0.10   # meters

def motion_intent(x, y):
    fwd = "Forward" if x >  FORWARD_THRESHOLD else ("Reverse" if x < -FORWARD_THRESHOLD else "Stop")
    if   y >  TURN_THRESHOLD: turn = "Turn Right"
    elif y < -TURN_THRESHOLD: turn = "Turn Left"
    else:                     turn = "No Turn"
    return fwd, turn

def build_velocity_command(x, y, v_scale: float, w_scale: float):
    """
    Convert relative (x,y) target in BODY_ENU into a simple diff-drive-ish command.
      v ~ proportional to forward offset (clip to [-1,1] then scale)
      w ~ proportional to lateral offset  (clip to [-1,1] then scale)
    """
    def clip(a): return max(-1.0, min(1.0, a))
    v = clip(x) * v_scale
    w = clip(y) * w_scale
    return {"cmd": "vel", "v": v, "w": w}

def _emit_velocity(out_ser, payload: dict):
    line = (json.dumps(payload) + "\n").encode("utf-8")
    if out_ser is None:
        print("[EMIT]", payload)  # console only
    else:
        out_ser.write(line)
        out_ser.flush()
        print("[EMIT -> SERIAL]", payload)

def handle_line(line, emit_vel=False, v_scale=0.3, w_scale=0.8, motor_ser=None):
    line = line.strip()
    if not line:
        return
    print(f"[RAW] {line}")
    try:
        data = json.loads(line)
    except json.JSONDecodeError:
        print("[WARN] Bad JSON line; skipping")
        return

    if data.get("type") != "relative_target":
        print("[INFO] Ignoring msg type:", data.get("type"))
        return

    try:
        x = float(data["x"]); y = float(data["y"]); z = float(data["z"])
    except Exception:
        print("[WARN] Missing or non-numeric fields; skipping")
        return

    fwd, turn = motion_intent(x, y)
    print(f"[PARSED] id={data.get('marker_id')} frame={data.get('frame')} pos=({x:.2f},{y:.2f},{z:.2f})")
    print(f"[MOTION] {fwd} | {turn}")

    if emit_vel:
        vel = build_velocity_command(x, y, v_scale=v_scale, w_scale=w_scale)
        _emit_velocity(motor_ser, vel)

def run_tcp(host, port, emit_vel, v_scale, w_scale, motor_serial, motor_baud):
    print(f"[TCP] Listening on {host}:{port} …")
    out_ser = None
    try:
        if motor_serial:
            if serial is None:
                print("[WARN] pyserial not installed; cannot open motor serial. Continuing console-only.")
            else:
                print(f"[MOTOR] Opening {motor_serial} @ {motor_baud} …")
                out_ser = serial.Serial(motor_serial, motor_baud, timeout=1)
                print("[MOTOR] Serial ready")

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as srv:
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind((host, port))
            srv.listen(1)
            conn, addr = srv.accept()
            print(f"[TCP] Connected from {addr}")
            with conn:
                buf = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("[TCP] Connection closed")
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        handle_line(
                            line.decode("utf-8", errors="ignore"),
                            emit_vel=emit_vel, v_scale=v_scale, w_scale=w_scale, motor_ser=out_ser
                        )
    finally:
        if out_ser is not None:
            try: out_ser.close()
            except Exception: pass

def run_serial(in_port, in_baud, emit_vel, v_scale, w_scale, motor_serial, motor_baud):
    if serial is None:
        print("pyserial not installed. Run: pip install pyserial", file=sys.stderr)
        sys.exit(1)

    out_ser = None
    try:
        print(f"[SERIAL-IN] Opening {in_port} @ {in_baud} …")
        with serial.Serial(in_port, in_baud, timeout=1) as ser_in:
            print("[SERIAL-IN] Ready. Waiting for lines …")
            if motor_serial:
                print(f"[MOTOR] Opening {motor_serial} @ {motor_baud} …")
                out_ser = serial.Serial(motor_serial, motor_baud, timeout=1)
                print("[MOTOR] Serial ready")

            while True:
                line = ser_in.readline().decode("utf-8", errors="ignore")
                handle_line(line, emit_vel=emit_vel, v_scale=v_scale, w_scale=w_scale, motor_ser=out_ser)
    finally:
        if out_ser is not None:
            try: out_ser.close()
            except Exception: pass

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["tcp","serial"], default="tcp")

    # TCP listener
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5761)

    # Serial listener (if instead of TCP)
    ap.add_argument("--serial-port", default="COM7")
    ap.add_argument("--baud", type=int, default=115200)

    # Navigation output
    ap.add_argument("--emit-vel", action="store_true", help="Emit velocity JSON for 'initiate ground navigation'")
    ap.add_argument("--v-scale", type=float, default=0.3, help="linear velocity scale")
    ap.add_argument("--w-scale", type=float, default=0.8, help="angular velocity scale")
    ap.add_argument("--motor-serial", default="", help="optional serial port to write velocity JSON to (e.g., COM6 or /dev/ttyUSB0)")
    ap.add_argument("--motor-baud", type=int, default=115200)

    args = ap.parse_args()

    if args.mode == "tcp":
        run_tcp(args.host, args.port, args.emit_vel, args.v_scale, args.w_scale, args.motor_serial or "", args.motor_baud)
    else:
        run_serial(args.serial_port, args.baud, args.emit_vel, args.v_scale, args.w_scale, args.motor_serial or "", args.motor_baud)

if __name__ == "__main__":
    main()
