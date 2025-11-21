# New Scripts/overwatch/test_overwatch_integration.py
"""
Complete integration test for continuous UAV overwatch (no hardware).
Simulates:
- UGV moving toward a target with drift
- Overwatch computing BODY_ENU correction (x,y)
- Controller generating (v,w)
- Arrival/timeout/loss-of-sight logic
"""

import time
import math
import random
import argparse

from overwatch_manager import OverwatchManager, OverwatchConfig, SimpleLocation

class SimulatedUGV:
    def __init__(self, start_lat: float, start_lon: float, speed_ms: float = 0.3):
        self.lat = start_lat
        self.lon = start_lon
        self.heading = 0.0  # degrees
        self.max_speed = speed_ms

    def update(self, v_cmd: float, w_cmd: float, dt: float = 0.5):
        # Update heading (rad/s -> deg)
        self.heading = (self.heading + math.degrees(w_cmd * dt)) % 360.0

        # Move forward/back
        distance = max(-self.max_speed, min(self.max_speed, v_cmd)) * dt

        # Add a small random drift (±8°) to simulate imperfect execution
        drift_deg = random.uniform(-8.0, 8.0)
        move_dir_deg = (self.heading + drift_deg) % 360.0

        # Convert meters to lat/lon deltas (approx)
        meters_per_deg_lat = 111320.0
        meters_per_deg_lon = 111320.0 * math.cos(math.radians(self.lat))

        dlat = (distance * math.cos(math.radians(move_dir_deg))) / meters_per_deg_lat
        dlon = (distance * math.sin(math.radians(move_dir_deg))) / meters_per_deg_lon

        self.lat += dlat
        self.lon += dlon

    def get_state(self):
        return self.lat, self.lon, self.heading

def run_sim(target_lat: float, target_lon: float, duration_s: int = 90):
    # Start/Target (fake)
    start_lat = 32.92019
    start_lon = -96.94831
    target = SimpleLocation(target_lat, target_lon, 0.0)

    # Config
    cfg = OverwatchConfig(
        max_deviation_meters=2.0,
        correction_threshold_meters=1.0,
        arrival_radius_meters=1.5,
        update_rate_hz=2.0,
        max_mission_time_seconds=duration_s
    )

    ugv = SimulatedUGV(start_lat, start_lon, speed_ms=0.35)
    ow = OverwatchManager(target, cfg)
    ow.start_mission()

    period = 1.0 / cfg.update_rate_hz
    last_status = 0.0

    try:
        t0 = time.time()
        while True:
            loop_start = time.time()

            # "UAV observation" (here we use the sim UGV's position directly)
            lat, lon, hdg = ugv.get_state()
            conf = 0.9

            state = ow.update_ugv_state(lat, lon, hdg, start_lat, start_lon, conf)

            # Decide if correction needed (status only)
            needs_corr, _ = ow.needs_correction()
            if needs_corr:
                ow.total_corrections += 1

            # Compute BODY_ENU error and control
            x, y = ow.correction_vector_body_enu(lat, lon, hdg)
            cmd = ow.generate_velocity_command(x, y, confidence=conf)

            # Apply to UGV sim
            ugv.update(cmd["v"], cmd["w"], dt=period)

            # Print status @ ~1 Hz
            if time.time() - last_status > 1.0:
                ow.print_status()
                print(f"         Command: v={cmd['v']:.3f} m/s, w={cmd['w']:.3f} rad/s")
                print(f"         UGV: lat={lat:.6f}, lon={lon:.6f}, hdg={hdg:.1f}°\n")
                last_status = time.time()

            # Stopping conditions
            if ow.has_reached_target():
                print("\n================ ✓ TARGET REACHED =================\n")
                break
            if ow.is_mission_timeout():
                print("\n================ ✗ MISSION TIMEOUT ================\n")
                break

            # Rate limit
            elapsed = time.time() - loop_start
            if elapsed < period:
                time.sleep(period - elapsed)

    except KeyboardInterrupt:
        print("\n[SIM] Interrupted by user.")

    # Summary
    summary = ow.mission_summary()
    print("================ MISSION SUMMARY ================")
    print(f"Duration (s):       {summary['mission_time_seconds']:.1f}")
    print(f"Reached Target:     {summary['reached_target']}")
    print(f"Total Corrections:  {summary['total_corrections']}")
    print(f"Max Deviation (m):  {summary['max_deviation_meters']:.2f}")
    if summary['final_distance_to_target'] is not None:
        print(f"Final Distance (m): {summary['final_distance_to_target']:.2f}")
    print("=================================================")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--target-lat", type=float, default=32.92030, help="Target latitude")
    ap.add_argument("--target-lon", type=float, default=-96.94825, help="Target longitude")
    ap.add_argument("--duration", type=int, default=90, help="Max sim duration (s)")
    args = ap.parse_args()
    run_sim(args.target_lat, args.target_lon, args.duration)

if __name__ == "__main__":
    main()
