import time

# Ensures compiler can find DroneTest module from this file (DroneTest.TestSim.py)
import sys, os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

# Project file imports
from dronekit import connect, Vehicle, VehicleMode

#commented out search algo for phase 1 2526
#from DroneCode.SearchAlgoScript import load_waypoints_from_csv, equirectangular_approximation 

# Allow relative imports from project root
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

# Used to connect to copter with args from command line
def connectMyCopter(SIMULATE_DRONE: bool) -> Vehicle:
    """
    Connect to the UAV (Pixhawk or SITL).
    GPS is kept enabled for safety/failsafe systems, but not used for navigation.
    """
    if SIMULATE_DRONE:
        import dronekit_sitl
        # Keep default start position for GPS/RTL boundaries
        sitl = dronekit_sitl.start_default(lat=32.92019, lon=-96.94831)
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)
        print("[SIM] Connected to SITL UAV with GPS enabled.")
    else:
        print("[HW] Connecting to Pixhawk on /dev/ttyACM0 ...")
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
        print("[HW] Connection established with GPS.")

    return vehicle

# Used to arm the drone
def arm_drone(vehicle):
    """
    Arms the UAV safely. Requires GPS fix to ensure failsafe behavior.
    """
    while not vehicle.is_armable:
        print("Waiting for UAV to become armable...")
        time.sleep(1)

    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)

    # Ensure GPS has a valid fix (2D/3D)
    while vehicle.gps_0.fix_type < 2:
        print(f"Waiting for GPS fix... (current: {vehicle.gps_0.fix_type})")
        time.sleep(1)
    print("GPS fix acquired. Ready for arming.")

    print("Arming UAV motors...")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    print("UAV armed successfully and ready for takeoff.")

# Used to take off the drone to a specific altitude, uses GPS still, may need to be changed 
def takeoff_drone(vehicle, targetAltitude=2.0):
    """
    Basic takeoff. GPS/barometer data is used to ensure safe climb and bounding.
    """
    print(f"Taking off to {targetAltitude} meters...")
    vehicle.simple_takeoff(targetAltitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.2f} m")
        if alt >= targetAltitude * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

    # Optional: Verify boundaries if you’re enforcing a safety box
    print("Takeoff complete. GPS and barometer active.")
 
#COMMENTED OUT SEARCH PATTERN FOR PHASE 1
# def flyInSearchPattern(vehicle, SIMULATE_DRONE: bool):
#     # helper function
#     def getCurrentLocation(vehicle):
#         currentLoc = (vehicle.location.global_relative_frame.lat, vehicle.location.global_relative_frame.lon)
#         return currentLoc
    
#     search_waypoints = load_waypoints_from_csv('generated_search_pattern_waypoints.csv')
#     # Iterate over waypoints, expecting lists of [latitude, longitude]
#     for wp in search_waypoints:
#         currentWP = (wp.lat, wp.lon)
#         print("Waypoint: ", currentWP)
#         # Go to the waypoint
#         vehicle.simple_goto(wp)
#         #time.sleep(20)
#         while(equirectangular_approximation(getCurrentLocation(vehicle), currentWP) > .5): 
#             if not SIMULATE_DRONE:
#                 print(vehicle.location.global_relative_frame.alt)
#             print(f"Current Location: ({vehicle.location.global_relative_frame.lat}, {vehicle.location.global_relative_frame.lon})")
#             print("Distance to WP:", equirectangular_approximation(getCurrentLocation(vehicle),currentWP))
#             time.sleep(1)