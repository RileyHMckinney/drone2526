from __future__ import print_function
import argparse
import time

from UGVConnect import connectMyRover, arm_rover, set_speed
from dronekit import VehicleMode, LocationGlobalRelative

parser = argparse.ArgumentParser(description="Connect to a UGV (Pixhawk Rover or SITL).")
parser.add_argument("--liverover", action="store_true", help="Connect to a real rover instead of SITL.")
args = parser.parse_args()

# Simulation vs live rover
SIMULATE_ROVER = not args.liverover

# Connect to rover
rover = connectMyRover(SIMULATE_ROVER)
print(f"Starting Location: ({rover.location.global_relative_frame.lat}, {rover.location.global_relative_frame.lon})")
print("Heading:", rover.heading)

# Arm rover
arm_rover(rover)

# Set speed and try a simple move
set_speed(rover, 1.0)

# Drive toward a nearby waypoint (small offset from start)
target = LocationGlobalRelative(
    rover.location.global_relative_frame.lat + 0.00005,
    rover.location.global_relative_frame.lon + 0.00005,
    0
)
print(f"Going to test waypoint: {target}")
rover.simple_goto(target)

time.sleep(15)

print("Stopping rover (HOLD mode)")
rover.mode = VehicleMode("HOLD")

rover.close()
print("UGV test complete.")
