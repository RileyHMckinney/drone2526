"""
UGV Connection Module
Handles connection, arming, and basic control of the rover
Adapted from UAV connection code for ground vehicle
"""
from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode

def connectMyRover(SIMULATE_ROVER: bool) -> Vehicle:
    """
    Connect to rover (Pixhawk ArduRover or SITL).
    
    Args:
        SIMULATE_ROVER: If True, launches SITL rover. If False, connects to hardware.
    
    Returns:
        Vehicle object connected to the rover
    """
    if SIMULATE_ROVER:
        # Create a SITL rover instance
        import dronekit_sitl
        sitl = dronekit_sitl.SITL()
        # Download/ensure rover firmware is available
        sitl.download('rover', 'stable')
        # Launch SITL with rover model at your competition location
        # Home location: UTD area (32.9201927, -96.9483108)
        sitl.launch(['--model', 'rover', '--home', '32.9201927,-96.9483108,0,180'], await_ready=True)
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)
        print(f"✓ Connected to SITL rover at {connection_string}")
    else:
        # Connect to physical rover via serial
        # Change to your Pixhawk serial port if needed
        # Common ports: /dev/ttyACM0, /dev/ttyUSB0 (Linux/Mac), COM3 (Windows)
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
        print("✓ Connected to physical rover at /dev/ttyACM0")
    
    return vehicle


def arm_rover(vehicle: Vehicle):
    """
    Arm the rover and put it in GUIDED mode.
    
    Args:
        vehicle: DroneKit Vehicle object
    """
    print("\n[UGV] Preparing to arm...")
    
    # Wait for rover to be armable
    while not vehicle.is_armable:
        print("  Waiting for rover to become armable...")
        time.sleep(1)
    print("✓ Rover is now armable")
    
    # Set to GUIDED mode
    print("[UGV] Setting GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print("  Waiting for rover to enter GUIDED mode...")
        time.sleep(1)
    print("✓ Rover now in GUIDED mode")
    
    # Check GPS before arming
    print("[UGV] Checking GPS...")
    while vehicle.gps_0.fix_type < 2:  # 2 = 2D fix, 3 = 3D fix
        print(f"  Waiting for GPS fix... (current: {vehicle.gps_0.fix_type})")
        time.sleep(1)
    print(f"✓ GPS ready (fix type: {vehicle.gps_0.fix_type})")
    
    # Arm the rover
    print("[UGV] Arming rover...")
    vehicle.armed = True
    while not vehicle.armed:
        print("  Waiting for rover to arm...")
        time.sleep(1)
    print("✓ Rover is armed and ready!")


def disarm_rover(vehicle: Vehicle):
    """
    Disarm the rover.
    
    Args:
        vehicle: DroneKit Vehicle object
    """
    print("\n[UGV] Disarming rover...")
    vehicle.armed = False
    while vehicle.armed:
        print("  Waiting for rover to disarm...")
        time.sleep(1)
    print("✓ Rover disarmed")


def set_speed(vehicle: Vehicle, speed: float):
    """
    Set rover ground speed.
    
    Args:
        vehicle: DroneKit Vehicle object
        speed: Speed in m/s
    """
    print(f"[UGV] Setting speed to {speed} m/s")
    # For rovers, we set the groundspeed parameter
    vehicle.groundspeed = speed


def stop_rover(vehicle: Vehicle):
    """
    Stop the rover by switching to HOLD mode.
    
    Args:
        vehicle: DroneKit Vehicle object
    """
    print("[UGV] Stopping rover (HOLD mode)...")
    vehicle.mode = VehicleMode("HOLD")
    while vehicle.mode != 'HOLD':
        time.sleep(0.5)
    print("✓ Rover stopped")


def get_distance_meters(lat1, lon1, lat2, lon2):
    """
    Calculate distance between two GPS coordinates using Haversine formula.
    
    Args:
        lat1, lon1: First coordinate
        lat2, lon2: Second coordinate
    
    Returns:
        Distance in meters
    """
    import math
    
    R = 6371000  # Earth radius in meters
    
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    
    a = (math.sin(delta_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    distance = R * c
    return distance


def navigate_to_location(vehicle: Vehicle, target_lat, target_lon, 
                        arrival_radius=1.5, timeout=420):
    """
    Navigate rover to a GPS location and monitor progress.
    
    Args:
        vehicle: DroneKit Vehicle object
        target_lat: Target latitude
        target_lon: Target longitude
        arrival_radius: Distance threshold in meters (default 1.5m)
        timeout: Maximum navigation time in seconds (default 420 = 7 min)
    
    Returns:
        True if arrived, False if timeout
    """
    from dronekit import LocationGlobalRelative
    
    print(f"\n[UGV] Navigating to ({target_lat:.8f}, {target_lon:.8f})")
    print(f"  Arrival radius: {arrival_radius}m")
    print(f"  Timeout: {timeout}s")
    
    # Create target location
    target_location = LocationGlobalRelative(target_lat, target_lon, 0)
    
    # Get starting position
    start_lat = vehicle.location.global_relative_frame.lat
    start_lon = vehicle.location.global_relative_frame.lon
    total_distance = get_distance_meters(start_lat, start_lon, target_lat, target_lon)
    
    print(f"  Starting position: ({start_lat:.8f}, {start_lon:.8f})")
    print(f"  Total distance: {total_distance:.2f}m")
    
    # Send navigation command
    vehicle.simple_goto(target_location)
    
    # Monitor progress
    start_time = time.time()
    last_distance = None
    
    while True:
        # Get current position
        current_lat = vehicle.location.global_relative_frame.lat
        current_lon = vehicle.location.global_relative_frame.lon
        
        # Calculate distance to target
        distance = get_distance_meters(current_lat, current_lon, target_lat, target_lon)
        
        # Print progress (only if changed significantly)
        if last_distance is None or abs(distance - last_distance) > 0.5:
            elapsed = time.time() - start_time
            print(f"  Distance: {distance:.2f}m | Time: {elapsed:.1f}s | Speed: {vehicle.groundspeed:.2f}m/s")
            last_distance = distance
        
        # Check if arrived
        if distance <= arrival_radius:
            elapsed = time.time() - start_time
            print(f"\n✓ Arrived at target!")
            print(f"  Final distance: {distance:.2f}m")
            print(f"  Travel time: {elapsed:.1f}s")
            return True
        
        # Check timeout
        if time.time() - start_time > timeout:
            print(f"\n✗ Navigation timeout ({timeout}s exceeded)")
            print(f"  Final distance: {distance:.2f}m")
            return False
        
        time.sleep(1)


# Example usage / testing
if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Test UGV connection and navigation")
    parser.add_argument("--liverover", action="store_true", 
                       help="Connect to real rover instead of SITL")
    args = parser.parse_args()
    
    # Connect
    print("="*60)
    print("UGV CONNECTION TEST")
    print("="*60)
    
    SIMULATE = not args.liverover
    rover = connectMyRover(SIMULATE)
    
    print(f"\nStarting location: ({rover.location.global_relative_frame.lat:.8f}, "
          f"{rover.location.global_relative_frame.lon:.8f})")
    print(f"Heading: {rover.heading}°")
    print(f"GPS fix type: {rover.gps_0.fix_type}")
    
    # Arm
    arm_rover(rover)
    
    # Set speed
    set_speed(rover, 0.5)
    
    # Test navigation to nearby point
    print("\n" + "="*60)
    print("TESTING NAVIGATION")
    print("="*60)
    
    # Move 5 meters north (approximately)
    target_lat = rover.location.global_relative_frame.lat + 0.000045
    target_lon = rover.location.global_relative_frame.lon
    
    success = navigate_to_location(rover, target_lat, target_lon, 
                                   arrival_radius=1.5, timeout=60)
    
    if success:
        print("\n✓ Navigation test passed!")
    else:
        print("\n✗ Navigation test failed!")
    
    # Stop and disarm
    stop_rover(rover)
    disarm_rover(rover)
    
    # Close connection
    rover.close()
    print("\n✓ Test complete")
