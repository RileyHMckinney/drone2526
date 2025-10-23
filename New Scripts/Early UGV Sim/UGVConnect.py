from __future__ import print_function
import time
from dronekit import connect, Vehicle, VehicleMode

def connectMyRover(SIMULATE_ROVER: bool) -> Vehicle:
    """
    Connect to rover (Pixhawk ArduRover or SITL).
    """
    if SIMULATE_ROVER:
        import dronekit_sitl
        sitl = dronekit_sitl.SITL()
        # Download/ensure rover firmware is available
        sitl.download('rover', 'stable')
        # Launch SITL with rover model at given home location
        sitl.launch(['--model', 'rover', '--home', '32.9201927,-96.9483108,0,180'], await_ready=True)
        connection_string = sitl.connection_string()
        vehicle = connect(connection_string, wait_ready=True)
    else:
        # Change to your Pixhawk serial port if needed
        vehicle = connect('/dev/ttyACM0', baud=115200, wait_ready=True)

    return vehicle


def arm_rover(vehicle: Vehicle):
    while not vehicle.is_armable:
        print("Waiting for rover to become armable")
        time.sleep(1)
    print("Rover is now armable")

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode != 'GUIDED':
        print("Waiting for rover to enter GUIDED mode")
        time.sleep(1)
    print("Rover now in GUIDED mode")

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for rover to arm...")
        time.sleep(1)
    print("Rover is armed!")


def set_speed(vehicle: Vehicle, speed: float):
    print(f"Setting speed to {speed} m/s")
    vehicle.airspeed = speed
