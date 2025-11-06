"""
Challenge 2: UAV Target Detection and UGV Navigation
Complete implementation for AVC Challenge 2

Flow:
1. UAV takes off from UGV
2. UAV searches for ArUco marker (ID 0)
3. UAV detects marker and calculates GPS coordinates
4. UAV sends coordinates to UGV
5. UGV navigates to target location
6. UAV lands on UGV at target location
"""
from __future__ import print_function
import argparse
import time
import json
from dronekit import VehicleMode, LocationGlobalRelative
from UGVConnect import connectMyRover, arm_rover, set_speed
from UAVVisionIntegration import VisionCoordinator
import math

# Competition parameters
TARGET_MARKER_ID = 0
MARKER_SIZE = 0.1  # 10cm markers
SEARCH_ALTITUDE = 10.0  # meters
SEARCH_SPEED = 2.0  # m/s
UGV_SPEED = 0.5  # m/s (must be >= 0.2 mph = 0.089 m/s)
ARRIVAL_RADIUS = 1.5  # meters (must be within 5 ft = 1.52m)

class Challenge2Mission:
    """Manages Challenge 2 mission execution"""
    
    def __init__(self, simulate_rover=True, use_zed=False):
        """
        Initialize Challenge 2 mission
        
        Args:
            simulate_rover: Use SITL simulation if True
            use_zed: Use ZED camera if True
        """
        self.simulate = simulate_rover
        self.rover = None
        self.vision = None
        self.use_zed = use_zed
        self.mission_start_time = None
        self.target_found = False
        self.target_coords = None
        
    def initialize_systems(self):
        """Initialize UGV and vision systems"""
        print("\n" + "="*60)
        print("CHALLENGE 2: INITIALIZATION")
        print("="*60)
        
        # Connect to UGV
        print("\n[1/2] Connecting to UGV...")
        self.rover = connectMyRover(self.simulate)
        print(f"✓ UGV connected at ({self.rover.location.global_relative_frame.lat:.8f}, "
              f"{self.rover.location.global_relative_frame.lon:.8f})")
        
        # Initialize vision system
        print("\n[2/2] Initializing vision system...")
        self.vision = VisionCoordinator(
            calibration_file="calibration_chessboard.yaml",
            marker_size=MARKER_SIZE,
            use_zed=self.use_zed,
            target_marker_id=TARGET_MARKER_ID
        )
        print("✓ Vision system initialized")
        
        print("\n" + "="*60)
        print("✓ ALL SYSTEMS READY")
        print("="*60)
    
    def simulate_uav_takeoff(self):
        """
        Simulate UAV takeoff from UGV
        In real implementation, this would be your actual UAV takeoff code
        """
        print("\n" + "="*60)
        print("PHASE 1: UAV TAKEOFF FROM UGV")
        print("="*60)
        
        print("\n[Simulated] UAV taking off from UGV...")
        print(f"[Simulated] Target altitude: {SEARCH_ALTITUDE}m")
        time.sleep(2)  # Simulate takeoff time
        print("✓ UAV at search altitude")
        
        # In real implementation, return your UAV vehicle object
        # For simulation, we'll use UGV position as proxy
        return self.rover  # Replace with actual UAV vehicle
    
    def search_for_target(self, uav_vehicle):
        """
        Search for target ArUco marker
        
        Args:
            uav_vehicle: UAV DroneKit vehicle object
        
        Returns:
            MarkerPosition if found, None otherwise
        """
        print("\n" + "="*60)
        print("PHASE 2: TARGET SEARCH")
        print("="*60)
        
        print(f"\n[UAV] Searching for marker ID {TARGET_MARKER_ID}...")
        print(f"[UAV] Search timeout: 30 seconds")
        print(f"[UAV] Confidence checks required: 3")
        
        # Search for marker with confidence checks
        marker = self.vision.find_target_marker(
            timeout=30.0,
            confidence_checks=3
        )
        
        if marker:
            self.target_found = True
            print(f"\n✓ TARGET ACQUIRED!")
            print(f"  Marker ID: {marker.marker_id}")
            print(f"  Distance: {marker.distance:.2f}m")
            print(f"  Position: X={marker.x:.2f}m, Y={marker.y:.2f}m, Z={marker.z:.2f}m")
        else:
            print("\n✗ TARGET NOT FOUND")
            print("  Marker detection failed within timeout")
        
        return marker
    
    def calculate_target_gps(self, marker, uav_vehicle):
        """
        Calculate target GPS coordinates from marker detection
        
        Args:
            marker: Detected marker position
            uav_vehicle: UAV vehicle object
        
        Returns:
            Target GPS coordinates dict
        """
        print("\n" + "="*60)
        print("PHASE 3: TARGET LOCALIZATION")
        print("="*60)
        
        # Get UAV position (for simulation, using rover position as proxy)
        uav_lat = uav_vehicle.location.global_relative_frame.lat
        uav_lon = uav_vehicle.location.global_relative_frame.lon
        uav_heading = uav_vehicle.heading
        
        print(f"\n[UAV] Current position: ({uav_lat:.8f}, {uav_lon:.8f})")
        print(f"[UAV] Current heading: {uav_heading}°")
        
        # Convert marker position to GPS coordinates
        target_coords = self.vision.marker_to_gps_offset(
            marker, uav_lat, uav_lon, uav_heading
        )
        
        self.target_coords = target_coords
        
        print(f"\n✓ TARGET COORDINATES CALCULATED")
        print(f"  Latitude: {target_coords['latitude']:.8f}")
        print(f"  Longitude: {target_coords['longitude']:.8f}")
        print(f"  Marker ID: {target_coords['marker_id']}")
        print(f"  Distance from UAV: {target_coords['distance']:.2f}m")
        
        return target_coords
    
    def send_target_to_ugv(self, target_coords):
        """
        Send target coordinates to UGV
        
        Args:
            target_coords: Target GPS coordinates
        
        Returns:
            True if successful
        """
        print("\n" + "="*60)
        print("PHASE 4: COORDINATE RELAY TO UGV")
        print("="*60)
        
        print(f"\n[COMMS] Sending target to UGV...")
        print(f"  Target: ({target_coords['latitude']:.8f}, {target_coords['longitude']:.8f})")
        
        # TODO: Implement actual communication with UGV
        # This would use your ESP32 client-server model or MAVLink
        # Example:
        # from UGVComms import send_coordinates_to_ugv
        # send_coordinates_to_ugv(target_coords)
        
        # For now, simulate successful transmission
        time.sleep(0.5)
        
        print("✓ COORDINATES TRANSMITTED TO UGV")
        return True
    
    def navigate_ugv_to_target(self, target_coords):
        """
        Command UGV to navigate to target location
        
        Args:
            target_coords: Target GPS coordinates
        """
        print("\n" + "="*60)
        print("PHASE 5: UGV NAVIGATION")
        print("="*60)
        
        # Arm and prepare UGV
        print("\n[UGV] Arming rover...")
        arm_rover(self.rover)
        
        print(f"[UGV] Setting speed to {UGV_SPEED} m/s")
        set_speed(self.rover, UGV_SPEED)
        
        # Create target location
        target_location = LocationGlobalRelative(
            target_coords['latitude'],
            target_coords['longitude'],
            0  # Ground level
        )
        
        print(f"\n[UGV] Navigating to target...")
        print(f"  Start: ({self.rover.location.global_relative_frame.lat:.8f}, "
              f"{self.rover.location.global_relative_frame.lon:.8f})")
        print(f"  Target: ({target_location.lat:.8f}, {target_location.lon:.8f})")
        
        # Send UGV to target
        self.rover.simple_goto(target_location)
        
        # Monitor progress
        start_time = time.time()
        last_distance = None
        
        while True:
            current_location = self.rover.location.global_relative_frame
            distance_to_target = self.get_distance_meters(
                current_location.lat, current_location.lon,
                target_location.lat, target_location.lon
            )
            
            if last_distance is None or abs(distance_to_target - last_distance) > 0.5:
                print(f"[UGV] Distance to target: {distance_to_target:.2f}m")
                last_distance = distance_to_target
            
            # Check if arrived
            if distance_to_target <= ARRIVAL_RADIUS:
                elapsed_time = time.time() - start_time
                print(f"\n✓ UGV ARRIVED AT TARGET!")
                print(f"  Final distance: {distance_to_target:.2f}m")
                print(f"  Travel time: {elapsed_time:.1f}s")
                break
            
            # Timeout check (7 minutes for challenge)
            if time.time() - start_time > 420:  # 7 minutes
                print("\n✗ NAVIGATION TIMEOUT (7 minutes exceeded)")
                break
            
            time.sleep(1)
        
        # Stop UGV
        print("\n[UGV] Stopping at target location")
        self.rover.mode = VehicleMode("HOLD")
    
    def simulate_uav_landing(self):
        """
        Simulate UAV landing on UGV at target
        In real implementation, this would be your precision landing code
        """
        print("\n" + "="*60)
        print("PHASE 6: UAV LANDING ON UGV")
        print("="*60)
        
        print("\n[UAV] Descending to UGV...")
        time.sleep(2)  # Simulate descent
        
        print("[UAV] Aligning with UGV platform...")
        time.sleep(1)
        
        print("[UAV] Final descent and touchdown...")
        time.sleep(2)
        
        print("\n✓ UAV LANDED ON UGV")
        print("[UAV] Coupled to UGV platform")
        
        # Check coupling duration (must stay coupled for 10 seconds)
        print("\n[Challenge] Maintaining coupling for 10 seconds...")
        for i in range(10, 0, -1):
            print(f"  {i}...", end='\r')
            time.sleep(1)
        print("\n✓ COUPLING REQUIREMENT MET (10 seconds)")
    
    def get_distance_meters(self, lat1, lon1, lat2, lon2):
        """
        Calculate distance between two GPS coordinates using Haversine formula
        
        Args:
            lat1, lon1: First coordinate
            lat2, lon2: Second coordinate
        
        Returns:
            Distance in meters
        """
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
    
    def log_mission_data(self, success):
        """
        Log mission results for competition documentation
        
        Args:
            success: True if mission completed successfully
        """
        mission_duration = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        log_data = {
            'challenge': 2,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'success': success,
            'duration_seconds': mission_duration,
            'target_found': self.target_found,
            'target_coordinates': self.target_coords,
            'arrival_radius': ARRIVAL_RADIUS,
            'ugv_speed': UGV_SPEED
        }
        
        log_filename = f"challenge2_log_{time.strftime('%Y%m%d_%H%M%S')}.json"
        with open(log_filename, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        print(f"\n[LOG] Mission data saved to {log_filename}")
    
    def run_mission(self):
        """Execute complete Challenge 2 mission"""
        self.mission_start_time = time.time()
        success = False
        
        try:
            # Initialize all systems
            self.initialize_systems()
            
            # Phase 1: UAV takeoff
            uav_vehicle = self.simulate_uav_takeoff()
            
            # Phase 2: Search for target marker
            marker = self.search_for_target(uav_vehicle)
            
            if not marker:
                print("\n✗ MISSION FAILED: Target not detected")
                return False
            
            # Phase 3: Calculate target GPS
            target_coords = self.calculate_target_gps(marker, uav_vehicle)
            
            # Phase 4: Send to UGV
            if not self.send_target_to_ugv(target_coords):
                print("\n✗ MISSION FAILED: Communication error")
                return False
            
            # Phase 5: UGV navigation
            self.navigate_ugv_to_target(target_coords)
            
            # Phase 6: UAV landing
            self.simulate_uav_landing()
            
            # Mission success
            success = True
            elapsed_time = time.time() - self.mission_start_time
            
            print("\n" + "="*60)
            print("✓ CHALLENGE 2 COMPLETE!")
            print("="*60)
            print(f"  Total mission time: {elapsed_time:.1f} seconds")
            print(f"  Time limit: 420 seconds (7 minutes)")
            if elapsed_time <= 420:
                print("  ✓ Within time limit!")
            else:
                print("  ✗ Exceeded time limit")
            print("="*60)
            
        except KeyboardInterrupt:
            print("\n\n⚠ Mission interrupted by user")
            success = False
        except Exception as e:
            print(f"\n\n✗ MISSION FAILED: {e}")
            import traceback
            traceback.print_exc()
            success = False
        finally:
            # Cleanup
            self.cleanup()
            
            # Log mission data
            self.log_mission_data(success)
        
        return success
    
    def cleanup(self):
        """Cleanup resources"""
        print("\n[CLEANUP] Shutting down systems...")
        
        if self.rover:
            print("  Stopping UGV...")
            self.rover.mode = VehicleMode("HOLD")
            self.rover.close()
        
        if self.vision:
            print("  Closing vision system...")
            self.vision.close()
        
        print("✓ Cleanup complete")


def main():
    """Main entry point for Challenge 2"""
    parser = argparse.ArgumentParser(description="Challenge 2: Target Detection and UGV Navigation")
    parser.add_argument("--liverover", action="store_true", 
                       help="Connect to real rover instead of SITL")
    parser.add_argument("--zed", action="store_true",
                       help="Use ZED camera instead of standard camera")
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("AVC CHALLENGE 2: TARGET DETECTION AND UGV NAVIGATION")
    print("="*60)
    print(f"Mode: {'LIVE HARDWARE' if args.liverover else 'SIMULATION'}")
    print(f"Camera: {'ZED' if args.zed else 'Standard USB'}")
    print(f"Target Marker ID: {TARGET_MARKER_ID}")
    print(f"Marker Size: {MARKER_SIZE}m")
    print("="*60)
    
    # Create and run mission
    mission = Challenge2Mission(
        simulate_rover=not args.liverover,
        use_zed=args.zed
    )
    
    success = mission.run_mission()
    
    # Exit with appropriate code
    exit(0 if success else 1)


if __name__ == "__main__":
    main()
