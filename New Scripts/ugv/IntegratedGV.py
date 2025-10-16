"""
IntegratedGV.py
---------------
Unified UGV control module (Raspberry Pi side).

• Listens for JSON packets from the UAV via the UGV ESP32 (UART)
• Parses relative position vectors (x, y, z)
• Simulates motion decisions the UGV would execute to move toward the target

Later, motion commands can be replaced with real Pixhawk or motor control logic.
"""

import serial
import json
import time

# === SERIAL CONFIGURATION ===
PORT = "/dev/ttyUSB0"     # ESP32 → Raspberry Pi connection
BAUDRATE = 115200
READ_DELAY = 0.5          # seconds between read cycles

# === CONTROL PARAMETERS ===
FORWARD_THRESHOLD = 0.2   # meters
TURN_THRESHOLD = 0.1      # meters
SPEED_SCALE = 0.5         # proportional gain for visualization

# === FUNCTIONS ===
def interpret_motion(x, y):
    """Simulate motion intent based on relative vector (x, y)."""
    move_forward = "Forward" if x > FORWARD_THRESHOLD else (
                   "Reverse" if x < -FORWARD_THRESHOLD else "Stop")

    if y > TURN_THRESHOLD:
        turn_dir = "Turn Right"
    elif y < -TURN_THRESHOLD:
        turn_dir = "Turn Left"
    else:
        turn_dir = "Straight"

    forward_mag = abs(x) * SPEED_SCALE
    turn_mag = abs(y) * SPEED_SCALE
    print(f"[SIM] {move_forward} ({forward_mag:.2f}) | {turn_dir} ({turn_mag:.2f})")

def main():
    print(f"[INFO] Opening serial port {PORT} at {BAUDRATE} baud…")
    with serial.Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)
        print("[INFO] Listening for incoming JSON data...\n")

        while True:
            try:
                raw = ser.readline().decode("utf-8", errors="ignore").strip()
                if not raw:
                    continue

                print(f"[RAW] {raw}")
                data = json.loads(raw)

                if data.get("type") == "relative_target":
                    x = float(data.get("x", 0))
                    y = float(data.get("y", 0))
                    marker = data.get("marker_id", "?")

                    print(f"[RX] Marker {marker}: x={x:.2f} m  y={y:.2f} m")
                    interpret_motion(x, y)

            except json.JSONDecodeError:
                print(f"[WARN] Invalid JSON: {raw}")
            except Exception as e:
                print(f"[ERROR] {e}")
                time.sleep(1)

            time.sleep(READ_DELAY)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] UGV controller terminated by user.")
