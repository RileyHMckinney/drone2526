import time

# Ensures compiler can find DroneTest module from this file (DroneTest.TestSim.py)
import sys, os
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)

# Project file imports
from dronekit import connect, Vehicle, VehicleMode
from pymavlink import mavutil  # Needed for GUIDED mode and MAVLink commands

# NOTE: Search algo for phase 1 2526 left commented out
# from DroneCode.SearchAlgoScript import load_waypoints_from_csv, equirectangular_approximation 

# Allow relative imports from project root
project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(project_root)


# ============================================================
# HELPER FUNCTIONS
# ============================================================

def connectMyCopter(SIMULATE_DRONE: bool) -> Vehicle:
    """
    Connect to the UAV (Pixhawk or SITL).
    GPS is kept enabled for safety/failsafe systems, but not used for navigation.
    """
    if SIMULATE_DRONE:
        import dronekit_sitl
        # Keep default start position for GPS/RTL boundaries
        sitl = dronekit_sitl.start_default(lat=32.92019, lon=-96.94831)
        time.sleep(1.0)  # Small delay helps stabilize SITL
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)
        vehicle.wait_ready('mode', 'gps_0', timeout=30)
        print("[SIM] Connected to SITL UAV with GPS enabled.")
    else:
        print("[HW] Connecting to Pixhawk on /dev/ttyACM0 ...")
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)
        print("[HW] Connection established with GPS.")

    return vehicle


def arm_drone(vehicle):
    """
    Arms the UAV safely. Requires GPS fix to ensure failsafe behavior.
    """
    while not vehicle.is_armable:
        print("Waiting for UAV to become armable...")
        time.sleep(1)

    print("Switching to GUIDED mode...")
    vehicle.mode = VehicleMode("GUIDED")
    
    # Robust mode switching with MAVLink fallback for SITL compatibility
    timeout = 20
    start_time = time.time()
    while vehicle.mode.name != "GUIDED":
        if time.time() - start_time > timeout:
            # Try MAVLink fallback
            print("[WARN] Forcing GUIDED via MAVLink...")
            vehicle._master.mav.set_mode_send(
                vehicle._master.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                4  # GUIDED mode
            )
            time.sleep(2)
            if vehicle.mode.name != "GUIDED":
                raise RuntimeError(f"Timeout waiting for GUIDED mode (stuck in {vehicle.mode.name})")
            break
        print(f"Waiting for GUIDED mode... (current: {vehicle.mode.name})")
        time.sleep(0.5)

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

    print("Takeoff complete. GPS and barometer active.")


# ============================================================
# DroneConnect CLASS FOR AUTONOMOUS CONTROL
# ============================================================

class DroneConnect:
    """
    High-level wrapper for DroneKit Vehicle providing:
      - Connection / arming / takeoff
      - Body-frame velocity control
      - Telemetry helpers for altitude, vertical speed, and IMU accel
    This class is used by the Challenge 1 real mission script.
    Existing functions (connectMyCopter, arm_drone, takeoff_drone) remain intact.
    """

    def __init__(self, connection_string: str = None, baud: int = 115200, simulate: bool = False):
        self.sitl = None

        if simulate:
            import dronekit_sitl
            print("[SIM] Starting SITL for DroneConnect...")
            self.sitl = dronekit_sitl.start_default(lat=32.92019, lon=-96.94831)
            time.sleep(1.0)
            if connection_string is None:
                connection_string = self.sitl.connection_string()
            self.vehicle = connect(connection_string, wait_ready=True)
            self.vehicle.wait_ready('mode', 'gps_0', timeout=30)
            print("[SIM] Connected to SITL UAV via DroneConnect.")
        else:
            if connection_string is None:
                connection_string = "/dev/ttyACM0"
            print(f"[HW] Connecting to Pixhawk on {connection_string} via DroneConnect...")
            self.vehicle = connect(connection_string, baud=baud, wait_ready=True)
            print("[HW] Connection established via DroneConnect.")

    def wait_ready(self):
        """
        Wait for basic vehicle attributes to be ready.
        """
        self.vehicle.wait_ready(True)
        print("[UAV] Vehicle ready.")

    def arm(self):
        """
        Arm the UAV safely in GUIDED mode with GPS fix.
        """
        while not self.vehicle.is_armable:
            print("[UAV] Waiting for UAV to become armable...")
            time.sleep(1)

        print("[UAV] Switching to GUIDED mode...")
        self.vehicle.mode = VehicleMode("GUIDED")

        timeout = 20
        start_time = time.time()
        while self.vehicle.mode.name != "GUIDED":
            if time.time() - start_time > timeout:
                print("[WARN] Forcing GUIDED via MAVLink in DroneConnect...")
                self.vehicle._master.mav.set_mode_send(
                    self.vehicle._master.target_system,
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    4  # GUIDED mode
                )
                time.sleep(2)
                if self.vehicle.mode.name != "GUIDED":
                    raise RuntimeError(f"Timeout waiting for GUIDED mode (stuck in {self.vehicle.mode.name})")
                break
            print(f"[UAV] Waiting for GUIDED mode... (current: {self.vehicle.mode.name})")
            time.sleep(0.5)

        while self.vehicle.gps_0.fix_type < 2:
            print(f"[UAV] Waiting for GPS fix... (current: {self.vehicle.gps_0.fix_type})")
            time.sleep(1)
        print("[UAV] GPS fix acquired. Ready for arming.")

        print("[UAV] Arming motors...")
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("[UAV] Waiting for arming...")
            time.sleep(1)
        print("[UAV] UAV armed successfully.")

    def takeoff(self, target_altitude: float):
        """
        Take off to a target altitude in meters using simple_takeoff.
        """
        print(f"[UAV] Taking off to {target_altitude:.2f} m...")
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f"[UAV] Altitude: {alt:.2f} m")
            if alt >= target_altitude * 0.95:
                print("[UAV] Reached target altitude.")
                break
            time.sleep(1)
        print("[UAV] Takeoff complete.")

    def send_body_velocity(self, vx: float, vy: float, vz: float):
        """
        Send body-frame velocity command in m/s.
        - vx: forward (+), backward (-)
        - vy: right (+), left (-)
        - vz: down (+), up (-) in BODY_NED
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,                                    # time_boot_ms (ignored)
            0, 0,                                 # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_NED,   # frame
            0b0000111111000111,                   # type_mask (only velocity enabled)
            0, 0, 0,                              # x, y, z positions (ignored)
            vx, vy, vz,                           # vx, vy, vz
            0, 0, 0,                              # ax, ay, az (ignored)
            0, 0                                  # yaw, yaw_rate (ignored)
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def get_altitude(self) -> float:
        """
        Returns altitude above home (takeoff) in meters using global_relative_frame.
        """
        return self.vehicle.location.global_relative_frame.alt

    def get_vertical_speed(self) -> float:
        """
        Returns vertical speed in m/s, with up = positive.
        DroneKit's velocity is [vx, vy, vz] in NED; vz is down-positive.
        """
        vx, vy, vz = self.vehicle.velocity
        # Convert NED vz (down-positive) to up-positive
        return -vz if vz is not None else 0.0

    def get_accel_z(self) -> float:
        """
        Returns vertical acceleration (Z) from IMU.
        Units are implementation-dependent; for touchdown detection we mainly care
        about variance, not absolute scale.
        """
        try:
            imu = self.vehicle.raw_imu
            # raw_imu.zacc may be in custom units; for variance, scaling is not critical.
            return float(imu.zacc)
        except Exception:
            # Fallback if IMU not available; safe default.
            return 0.0

    def disarm(self):
        """
        Disarm the UAV safely.
        """
        print("[UAV] Disarming motors...")
        self.vehicle.armed = False
        timeout = 10
        start = time.time()
        while self.vehicle.armed and (time.time() - start) < timeout:
            print("[UAV] Waiting for disarm...")
            time.sleep(1)
        print("[UAV] Disarm complete.")

    def close(self):
        """
        Close vehicle and SITL (if used).
        """
        if hasattr(self, "vehicle") and self.vehicle is not None:
            self.vehicle.close()
        if self.sitl is not None:
            self.sitl.stop()
