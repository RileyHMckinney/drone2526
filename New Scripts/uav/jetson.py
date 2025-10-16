"""
jetson.py
---------
Standalone serial test script for the UAV-side ESP32 connection.
Used to verify that the Jetson can send properly formatted JSON
packets to the UAV ESP32 over UART.

This script does NOT use camera data or DroneKit.
It simply sends placeholder relative-target vectors every 500 ms
to validate serial and ESP-NOW communication chains.
"""

import time
import json
from serial import Serial

# === Configuration ===
PORT = "/dev/ttyUSB0"   # Change to your actual serial port
BAUDRATE = 115200
TX_DELAY = 0.5          # seconds between transmissions

def main():
    print(f"[INFO] Opening serial connection on {PORT}...")
    with Serial(PORT, BAUDRATE, timeout=1) as ser:
        time.sleep(2)
        print("[INFO] Serial connection established. Sending test data...")

        packet_number = 0

        while True:
            # Example relative vector payload (dummy data)
            data_dict = {
                "type": "relative_target",
                "source": "uav_test",
                "frame": "BODY_ENU",
                "marker_id": 99,
                "x": round(1.0 + 0.1 * packet_number, 3),
                "y": round(-0.5, 3),
                "z": 0.0
            }

            json_str = json.dumps(data_dict)
            ser.write((json_str + "\n").encode("utf-8"))
            print(f"[SEND] {json_str}")

            packet_number += 1
            time.sleep(TX_DELAY)

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[INFO] Transmission halted by user.")
