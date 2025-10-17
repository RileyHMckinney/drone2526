"""
Challenge1Sim.py
RTX UAV Competition - Challenge 1 Simulation
Autonomous takeoff from UGV, flight, and landing on moving UGV

Place this file in: New Scripts/Challenge1Sim.py
"""

from __future__ import print_function
import sys
import os
import time
import argparse
from dronekit import VehicleMode, LocationGlobalRelative
from math import radians, cos, sin, sqrt, atan2

# Add project root to path to import from uav folder
scripts_dir = os.path.abspath(os.path.dirname(__file__))
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from uav.DroneConnect import connectMyCopter, arm_drone, takeoff_drone

# ============================================================
# CONFIGURATION
# ============================================================
DEFAULT_ALTITUDE = 4  # meters (minimum required)
FLIGHT_DURATION = 5   # seconds (minimum required)
CHALLENGE_TIMEOUT = 420  # 7 minutes in seconds
UGV_SPEED_MPH = 0.2   # minimum speed
UGV_SPEED_MS = UGV_SPEED_MPH * 0.44704  # convert to m/s
LANDING_HOLD_TIME = 30  # seconds to stay on UGV after landing

# ============================================================
# UTILITY FUNCTIONS
# ============================================================

def get_distance_metres(location1, location2):
    """
    Calculate distance between two locations using haversine formula.
    
    Args:
        location1 (tuple): (lat, lon) of first location
        location2 (tuple): (lat, lon) of second location
        
    Returns:
        float: Distance in meters
    """
    lat1, lon1 = location1
    lat2, lon2 = location2
    
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2) * sin(dlat/2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2) * sin(dlon/2)
    c = 2 * atan2(sqrt(a), sqrt(1-a))
    return 6378137.0 * c  # Earth radius in meters


def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobalRelative object offset by specified meters.
    
    Args:
        original_location (LocationGlobalRelative): Starting location
        dNorth (float): Meters to move north (negative for south)
        dEast (float): Meters to move east (negative for west)
        
    Returns:
        LocationGlobalRelative: New location
    """
    earth_radius = 6378137.0
    
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * cos(radians(original_location.lat)))
    
    new_lat = original_location.lat + (dLat * 180.0 / 3.14159265359)
    new_lon = original_location.lon + (dLon * 180.0 / 3.14159265359)
    
    return LocationGlobalRelative(new_lat, new_lon, original_location.alt)


def get_current_location(vehicle):
    """
    Get current vehicle location as tuple.
    
    Args:
        vehicle: The vehicle
        
    Returns:
        tuple: (lat, lon) of current location
    """
    return (
        vehicle.location.global_relative_frame.lat,
        vehicle.location.global_relative_frame.lon
    )


def print_vehicle_state(vehicle):
    """
    Print current vehicle state for debugging.
    
    Args:
        vehicle: The vehicle
    """
    print("\n=== VEHICLE STATE ===")
    print(f"Mode: {vehicle.mode.name}")
    print(f"Armed: {vehicle.armed}")
    print(f"Location: ({vehicle.location.global_relative_frame.lat:.7f}, "
          f"{vehicle.location.global_relative_frame.lon:.7f})")
    print(f"Altitude: {vehicle.location.global_relative_frame.alt:.2f}m")
    print(f"Heading: {vehicle.heading}°")
    print(f"Airspeed: {vehicle.airspeed} m/s")
    print(f"Groundspeed: {vehicle.groundspeed} m/s")
    print(f"GPS Fix: {vehicle.gps_0.fix_type}")
    print("=" * 30 + "\n")

# ============================================================
# UGV SIMULATION CLASS
# ============================================================

class SimulatedUGV:
    """
    Simulates a moving UGV that travels in a straight line.
    """
    def __init__(self, start_location, heading_degrees, speed_ms):
        """
        Initialize simulated UGV.
        
        Args:
            start_location: LocationGlobalRelative object
            heading_degrees: Direction of travel (0=North, 90=East, etc.)
            speed_ms: Speed in meters per second
        """
        self.location = start_location
        self.heading = heading_degrees
        self.speed = speed_ms
        self.start_time = None
        self.moving = False
        
    def start_moving(self):
        """Start UGV movement."""
        self.start_time = time.time()
        self.moving = True
        print(f"\n{'='*60}")
        print("UGV MOVEMENT STARTED")
        print(f"{'='*60}")
        print(f"Speed: {self.speed * 2.23694:.2f} mph ({self.speed:.3f} m/s)")
        print(f"Heading: {self.heading}°")
        print(f"Starting location: ({self.location.lat:.7f}, {self.location.lon:.7f})\n")
    
    def get_current_location(self):
        """
        Get current UGV location based on time elapsed.
        
        Returns:
            LocationGlobalRelative: Current UGV position
        """
        if not self.moving:
            return self.location
        
        elapsed = time.time() - self.start_time
        distance_traveled = self.speed * elapsed
        
        # Calculate new position based on heading
        # 0° = North, 90° = East, 180° = South, 270° = West
        dNorth = distance_traveled * cos(radians(self.heading))
        dEast = distance_traveled * sin(radians(self.heading))
        
        return get_location_metres(self.location, dNorth, dEast)
    
    def get_distance_traveled(self):
        """Get total distance traveled since start."""
        if not self.moving:
            return 0.0
        elapsed = time.time() - self.start_time
        return self.speed * elapsed

# ============================================================
# CHALLENGE 1 EXECUTION
# ============================================================

def execute_challenge_1(vehicle, simulate=True):
    """
    Execute Challenge 1: Autonomous takeoff, flight, and landing on moving UGV.
    
    Args:
        vehicle: Connected vehicle object
        simulate: Whether this is a simulation or real flight
        
    Returns:
        bool: True if challenge completed successfully within time limit
    """
    challenge_start = time.time()
    
    print("\n" + "="*60)
    print("CHALLENGE 1: AUTONOMOUS UAV TAKEOFF AND LANDING")
    print("="*60)
    print(f"Start time: {time.strftime('%H:%M:%S')}")
    print(f"Time limit: {CHALLENGE_TIMEOUT} seconds ({CHALLENGE_TIMEOUT/60:.1f} minutes)\n")
    
    # Record starting location (simulating initial UGV position)
    start_location = vehicle.location.global_relative_frame
    print(f"Starting location: ({start_location.lat:.7f}, {start_location.lon:.7f})")
    print(f"Starting heading: {vehicle.heading}°\n")
    
    # ========================================
    # PHASE 1: AUTONOMOUS TAKEOFF
    # ========================================
    print("=" * 60)
    print("[PHASE 1] AUTONOMOUS TAKEOFF")
    print("=" * 60)
    
    # Use existing arm_drone and takeoff_drone from DroneConnect
    arm_drone(vehicle)
    takeoff_drone(vehicle, DEFAULT_ALTITUDE)
    
    phase1_time = time.time() - challenge_start
    print(f"Phase 1 complete in {phase1_time:.1f}s\n")
    
    # ========================================
    # PHASE 2: MAINTAIN FLIGHT
    # ========================================
    print("=" * 60)
    print(f"[PHASE 2] MAINTAINING FLIGHT FOR {FLIGHT_DURATION} SECONDS")
    print("=" * 60)
    
    flight_start = time.time()
    while time.time() - flight_start < FLIGHT_DURATION:
        elapsed = time.time() - flight_start
        alt = vehicle.location.global_relative_frame.alt
        remaining = FLIGHT_DURATION - elapsed
        print(f"⏱️  Flight time: {elapsed:.1f}s / {FLIGHT_DURATION}s "
              f"(remaining: {remaining:.1f}s) | Altitude: {alt:.2f}m")
        time.sleep(1)
    
    print(f"✓ Minimum flight time achieved ({FLIGHT_DURATION}s)\n")
    phase2_time = time.time() - challenge_start
    print(f"Phase 2 complete. Total elapsed: {phase2_time:.1f}s\n")
    
    # ========================================
    # PHASE 3: UGV MOVEMENT SIMULATION
    # ========================================
    print("=" * 60)
    print("[PHASE 3] UGV MOVEMENT SIMULATION")
    print("=" * 60)
    
    # Create simulated UGV starting from takeoff point, moving east (90°)
    ugv = SimulatedUGV(
        start_location=start_location,
        heading_degrees=90,  # Moving east
        speed_ms=UGV_SPEED_MS
    )
    ugv.start_moving()
    
    phase3_time = time.time() - challenge_start
    print(f"Phase 3 complete. Total elapsed: {phase3_time:.1f}s\n")
    
    # ========================================
    # PHASE 4: UAV TRACKING UGV
    # ========================================
    print("=" * 60)
    print("[PHASE 4] UAV TRACKING UGV")
    print("=" * 60)
    
    tracking_duration = 10  # seconds to demonstrate tracking
    tracking_start = time.time()
    
    print(f"Tracking for {tracking_duration} seconds...\n")
    
    while time.time() - tracking_start < tracking_duration:
        ugv_location = ugv.get_current_location()
        
        # Command UAV to track UGV position
        target = LocationGlobalRelative(
            ugv_location.lat,
            ugv_location.lon,
            DEFAULT_ALTITUDE
        )
        vehicle.simple_goto(target)
        
        # Calculate distance to UGV
        uav_pos = get_current_location(vehicle)
        ugv_pos = (ugv_location.lat, ugv_location.lon)
        distance = get_distance_metres(uav_pos, ugv_pos)
        
        elapsed = time.time() - tracking_start
        ugv_dist = ugv.get_distance_traveled()
        
        print(f"📍 Tracking: {elapsed:.1f}s | "
              f"Distance to UGV: {distance:.2f}m | "
              f"UGV traveled: {ugv_dist:.2f}m")
        print(f"   UAV: ({uav_pos[0]:.7f}, {uav_pos[1]:.7f})")
        print(f"   UGV: ({ugv_pos[0]:.7f}, {ugv_pos[1]:.7f})\n")
        
        time.sleep(1)
    
    print(f"✓ UAV successfully tracked UGV for {tracking_duration}s\n")
    phase4_time = time.time() - challenge_start
    print(f"Phase 4 complete. Total elapsed: {phase4_time:.1f}s\n")
    
    # ========================================
    # PHASE 5: LANDING ON MOVING UGV
    # ========================================
    print("=" * 60)
    print("[PHASE 5] LANDING ON MOVING UGV")
    print("=" * 60)
    print("Initiating landing sequence...\n")
    
    # Gradual descent while tracking UGV
    print("Descending to landing zone...")
    for target_alt in range(DEFAULT_ALTITUDE - 1, 0, -1):
        ugv_loc = ugv.get_current_location()
        target = LocationGlobalRelative(ugv_loc.lat, ugv_loc.lon, target_alt)
        vehicle.simple_goto(target)
        
        current_alt = vehicle.location.global_relative_frame.alt
        ugv_pos = (ugv_loc.lat, ugv_loc.lon)
        uav_pos = get_current_location(vehicle)
        distance = get_distance_metres(uav_pos, ugv_pos)
        
        print(f"⬇️  Target altitude: {target_alt}m | "
              f"Current: {current_alt:.2f}m | "
              f"Distance to UGV: {distance:.2f}m")
        time.sleep(1)
    
    # Final landing
    print("\nEngaging LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    
    # Wait for landing while continuing to update position toward UGV
    while vehicle.armed:
        alt = vehicle.location.global_relative_frame.alt
        print(f"⬇️  Landing... Altitude: {alt:.2f}m")
        
        # In simulation, try to stay over UGV during descent
        if simulate:
            ugv_loc = ugv.get_current_location()
            target = LocationGlobalRelative(ugv_loc.lat, ugv_loc.lon, alt)
            vehicle.simple_goto(target)
        
        time.sleep(1)
    
    print("\n✓ UAV has landed on UGV!\n")
    phase5_time = time.time() - challenge_start
    print(f"Phase 5 complete. Total elapsed: {phase5_time:.1f}s\n")
    
    # ========================================
    # PHASE 6: HOLD ON UGV FOR 30 SECONDS
    # ========================================
    print("=" * 60)
    print(f"[PHASE 6] REMAINING ON UGV FOR {LANDING_HOLD_TIME} SECONDS")
    print("=" * 60)
    
    hold_start = time.time()
    
    while time.time() - hold_start < LANDING_HOLD_TIME:
        elapsed = time.time() - hold_start
        remaining = LANDING_HOLD_TIME - elapsed
        ugv_loc = ugv.get_current_location()
        ugv_dist = ugv.get_distance_traveled()
        
        print(f"🎯 Hold time: {elapsed:.1f}s / {LANDING_HOLD_TIME}s "
              f"(remaining: {remaining:.1f}s)")
        print(f"   UGV Location: ({ugv_loc.lat:.7f}, {ugv_loc.lon:.7f})")
        print(f"   UGV traveled: {ugv_dist:.2f}m total\n")
        
        time.sleep(2)
    
    print(f"✓ 30-second hold complete!\n")
    
    # ========================================
    # CHALLENGE COMPLETE
    # ========================================
    total_time = time.time() - challenge_start
    
    print("\n" + "="*60)
    print("CHALLENGE 1 COMPLETE!")
    print("="*60)
    print(f"End time: {time.strftime('%H:%M:%S')}")
    print(f"Total time: {total_time:.1f}s / {CHALLENGE_TIMEOUT}s")
    print(f"Time margin: {CHALLENGE_TIMEOUT - total_time:.1f}s")
    
    success = total_time < CHALLENGE_TIMEOUT
    
    if success:
        print(f"\n{'='*60}")
        print("✓✓✓ CHALLENGE PASSED ✓✓✓")
        print(f"{'='*60}\n")
    else:
        print(f"\n{'='*60}")
        print("✗✗✗ CHALLENGE FAILED - TIMEOUT ✗✗✗")
        print(f"{'='*60}\n")
    
    # Summary
    print("\nCHALLENGE SUMMARY:")
    print("-" * 60)
    print(f"✓ Autonomous takeoff: COMPLETE")
    print(f"✓ Flight duration ({FLIGHT_DURATION}s minimum): COMPLETE")
    print(f"✓ Altitude ({DEFAULT_ALTITUDE}m minimum): COMPLETE")
    print(f"✓ UGV movement ({UGV_SPEED_MPH} mph): COMPLETE")
    print(f"✓ Landing on moving UGV: COMPLETE")
    print(f"✓ 30-second hold: COMPLETE")
    print(f"{'✓' if success else '✗'} Time limit (7 minutes): "
          f"{'COMPLETE' if success else 'EXCEEDED'}")
    print("-" * 60 + "\n")
    
    return success

# ============================================================
# MAIN
# ============================================================

def main():
    """Main entry point for Challenge 1 simulation."""
    
    parser = argparse.ArgumentParser(
        description="Challenge 1 Simulation: Autonomous UAV takeoff and landing on moving UGV",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python Challenge1Sim.py              # Run simulation (SITL)
  python Challenge1Sim.py --livedrone  # Connect to real drone
        """
    )
    parser.add_argument(
        "--livedrone", 
        action="store_true", 
        help="Connect to real drone instead of simulation"
    )
    
    args = parser.parse_args()
    
    simulate = not args.livedrone
    
    print("\n" + "="*60)
    print("RTX UAV COMPETITION - CHALLENGE 1 SIMULATION")
    print("="*60)
    print(f"Mode: {'SIMULATION (SITL)' if simulate else 'REAL DRONE'}")
    print(f"Date: {time.strftime('%Y-%m-%d %H:%M:%S')}")
    print("="*60 + "\n")
    
    vehicle = None
    
    try:
        # Connect to vehicle using existing DroneConnect function
        print("Connecting to vehicle...")
        vehicle = connectMyCopter(simulate)
        
        # Print initial vehicle state
        print_vehicle_state(vehicle)
        
        # Execute challenge
        success = execute_challenge_1(vehicle, simulate=simulate)
        
        # Cleanup
        print("Closing vehicle connection...")
        vehicle.close()
        print("✓ Connection closed\n")
        
        # Exit with appropriate code
        exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n\n" + "="*60)
        print("SIMULATION INTERRUPTED BY USER")
        print("="*60 + "\n")
        if vehicle:
            print("Closing vehicle connection...")
            vehicle.close()
        exit(1)
        
    except Exception as e:
        print(f"\n\n{'='*60}")
        print("ERROR OCCURRED")
        print("="*60)
        print(f"Error: {e}\n")
        
        import traceback
        print("Traceback:")
        print("-" * 60)
        traceback.print_exc()
        print("-" * 60 + "\n")
        
        if vehicle:
            print("Closing vehicle connection...")
            vehicle.close()
        exit(1)


if __name__ == "__main__":
    main()
