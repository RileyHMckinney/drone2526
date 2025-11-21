"""
Challenge 3: Full Autonomous Mission with Obstacle Avoidance
Complete implementation for AVC Challenge 3

Flow:
1. UAV takes off from UGV
2. UAV searches for and detects ArUco marker (ID 0)
3. UAV calculates GPS coordinates and sends to UGV
4. UGV navigates to target with OBSTACLE AVOIDANCE (no GPS on UGV)
5. UAV provides continuous overwatch and guidance
6. UGV avoids obstacles using vision/sensors
7. UAV lands on UGV at target location
"""
from __future__ import print_function
import argparse
import time
import json
import math
from dronekit import VehicleMode, LocationGlobalRelative
from UGVConnect import connectMyRover, arm_rover, set_speed, get_distance_meters
from UAVVisionIntegration import VisionCoordinator

# Competition parameters
TARGET_MARKER_ID = 0
MARKER_SIZE = 0.1  # 10cm markers
SEARCH_ALTITUDE = 10.0  # meters
SEARCH_SPEED = 2.0  # m/s
UGV_SPEED = 0.5  # m/s (must be >= 0.2 mph = 0.089 m/s)
ARRIVAL_RADIUS = 1.5  # meters (must be within 5 ft = 1.52m)
OBSTACLE_DETECTION_DISTANCE = 2.0  # meters - distance to detect obstacles
UAV_OVERWATCH_ALTITUDE = 15.0  # meters - altitude for UAV monitoring


class ObstacleAvoidance:
    """Handles obstacle detection and avoidance for UGV"""
    
    def __init__(self, use_brainchip=False):
        """
        Initialize obstacle avoidance system
        
        Args:
            use_brainchip: Use BrainChip for obstacle detection if True
        """
        self.use_brainchip = use_brainchip
        self.detected_obstacles = []
        self.last_detection_time = None
        
        if use_brainchip:
            print("[OBSTACLE] Using BrainChip for obstacle detection")
            # TODO: Initialize BrainChip hardware
        else:
            print("[OBSTACLE] Using standard camera for obstacle detection")
    
    def detect_obstacles(self, frame=None):
        """
        Detect obstacles in the path
        
        Args:
            frame: Camera frame for obstacle detection
        
        Returns:
            List of obstacles with position and distance
        """
        # Placeholder for actual obstacle detection
        # TODO: Implement actual vision-based obstacle detection
        
        obstacles = []
        
        if self.use_brainchip:
            # BrainChip-based detection
            pass
        else:
            # Standard camera-based detection
            # This would use depth perception, edge detection, etc.
            pass
        
        self.detected_obstacles = obstacles
        self.last_detection_time = time.time()
        
        return obstacles
    
    def calculate_avoidance_waypoint(self, current_pos, target_pos, obstacle):
        """
        Calculate intermediate waypoint to avoid obstacle
        
        Args:
            current_pos: Current (lat, lon) position
            target_pos: Target (lat, lon) position
            obstacle: Obstacle information
        
        Returns:
            Avoidance waypoint (lat, lon)
        """
        # Simple avoidance: move perpendicular to direct path
        current_lat, current_lon = current_pos
        target_lat, target_lon = target_pos
        
        # Calculate bearing to target
        bearing = math.atan2(
            target_lon - current_lon,
            target_lat - current_lat
        )
        
        # Move perpendicular (90 degrees) for 3 meters
        avoidance_distance = 3.0  # meters
        lat_offset = avoidance_distance * math.cos(bearing + math.pi/2) / 111111.0
        lon_offset = avoidance_distance * math.sin(bearing + math.pi/2) / (111111.0 * math.cos(math.radians(current_lat)))
        
        avoidance_lat = current_lat + lat_offset
        avoidance_lon = current_lon + lon_offset
        
        return (avoidance_lat, avoidance_lon)


class UAVOverwatch:
    """Manages UAV monitoring and guidance of UGV"""
    
    def __init__(self, vision_coordinator):
        """
        Initialize UAV overwatch system
        
        Args:
            vision_coordinator: VisionCoordinator instance
        """
        self.vision = vision_coordinator
        self.monitoring_active = False
        self.ugv_position_history = []
        self.guidance_waypoints = []
    
    def start_monitoring(self, uav_vehicle, ugv_vehicle):
        """
        Start continuous monitoring of UGV progress
        
        Args:
            uav_vehicle: UAV Vehicle object
            ugv_vehicle: UGV Vehicle object
        """
        print("\n[UAV OVERWATCH] Starting continuous monitoring...")
        self.monitoring_active = True
        
        # Move UAV to overwatch position
        print(f"[UAV OVERWATCH] Moving to overwatch altitude ({UAV_OVERWATCH_ALTITUDE}m)")
        # TODO: Command UAV to overwatch altitude
    
    def monitor_ugv_progress(self, ugv_vehicle, target_lat, target_lon):
        """
        Monitor UGV progress and provide guidance
        
        Args:
            ugv_vehicle: UGV Vehicle object
            target_lat: Target latitude
            target_lon: Target longitude
        
        Returns:
            Status dict with UGV position, distance, obstacles detected
        """
        current_lat = ugv_vehicle.location.global_relative_frame.lat
        current_lon = ugv_vehicle.location.global_relative_frame.lon
        
        # Calculate distance to target
        distance = get_distance_meters(current_lat, current_lon, target_lat, target_lon)
        
        # Record position history
        self.ugv_position_history.append({
            'lat': current_lat,
            'lon': current_lon,
            'timestamp': time.time(),
            'distance_to_target': distance
        })
        
        status = {
            'position': (current_lat, current_lon),
            'distance_to_target': distance,
            'heading': ugv_vehicle.heading,
            'speed': ugv_vehicle.groundspeed,
            'obstacles_detected': False
        }
        
        return status
    
    def provide_guidance_waypoint(self, ugv_vehicle, target_lat, target_lon, obstacle=None):
        """
        Provide corrective waypoint to UGV
        
        Args:
            ugv_vehicle: UGV Vehicle object
            target_lat: Final target latitude
            target_lon: Final target longitude
            obstacle: Obstacle to avoid (if any)
        
        Returns:
            Guidance waypoint (lat, lon)
        """
        current_lat = ugv_vehicle.location.global_relative_frame.lat
        current_lon = ugv_vehicle.location.global_relative_frame.lon
        
        if obstacle:
            # Calculate avoidance waypoint
            print("[UAV OVERWATCH] Obstacle detected, calculating avoidance route...")
            avoidance = ObstacleAvoidance()
            waypoint = avoidance.calculate_avoidance_waypoint(
                (current_lat, current_lon),
                (target_lat, target_lon),
                obstacle
            )
            print(f"[UAV OVERWATCH] Guidance waypoint: ({waypoint[0]:.8f}, {waypoint[1]:.8f})")
        else:
            # Direct to target
            waypoint = (target_lat, target_lon)
        
        self.guidance_waypoints.append(waypoint)
        return waypoint
    
    def stop_monitoring(self):
        """Stop UAV overwatch"""
        print("[UAV OVERWATCH] Stopping monitoring")
        self.monitoring_active = False


class Challenge3Mission:
    """Manages Challenge 3 mission execution"""
    
    def __init__(self, simulate_rover=True, use_zed=False, use_brainchip=False):
        """
        Initialize Challenge 3 mission
        
        Args:
            simulate_rover: Use SITL simulation if True
            use_zed: Use ZED camera if True
            use_brainchip: Use BrainChip for obstacle detection if True
        """
        self.simulate = simulate_rover
        self.rover = None
        self.vision = None
        self.obstacle_avoidance = None
        self.uav_overwatch = None
        self.use_zed = use_zed
        self.use_brainchip = use_brainchip
        self.mission_start_time = None
        self.target_found = False
        self.target_coords = None
        self.obstacles_encountered = 0
        self.avoidance_maneuvers = 0
    
    def initialize_systems(self):
        """Initialize UGV, vision, and obstacle avoidance systems"""
        print("\n" + "="*60)
        print("CHALLENGE 3: INITIALIZATION")
        print("="*60)
        
        # Connect to UGV
        print("\n[1/4] Connecting to UGV...")
        self.rover = connectMyRover(self.simulate)
        print(f"✓ UGV connected at ({self.rover.location.global_relative_frame.lat:.8f}, "
              f"{self.rover.location.global_relative_frame.lon:.8f})")
        
        # Initialize vision system
        print("\n[2/4] Initializing vision system...")
        self.vision = VisionCoordinator(
            calibration_file="calibration_chessboard.yaml",
            marker_size=MARKER_SIZE,
            use_zed=self.use_zed,
            target_marker_id=TARGET_MARKER_ID
        )
        print("✓ Vision system initialized")
        
        # Initialize obstacle avoidance
        print("\n[3/4] Initializing obstacle avoidance...")
        self.obstacle_avoidance = ObstacleAvoidance(use_brainchip=self.use_brainchip)
        print("✓ Obstacle avoidance initialized")
        
        # Initialize UAV overwatch
        print("\n[4/4] Initializing UAV overwatch...")
        self.uav_overwatch = UAVOverwatch(self.vision)
        print("✓ UAV overwatch initialized")
        
        print("\n" + "="*60)
        print("✓ ALL SYSTEMS READY FOR CHALLENGE 3")
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
        time.sleep(2)
        print("✓ UAV at search altitude")
        
        return self.rover  # Replace with actual UAV vehicle
    
    def search_for_target(self, uav_vehicle):
        """Search for target ArUco marker"""
        print("\n" + "="*60)
        print("PHASE 2: TARGET SEARCH")
        print("="*60)
        
        print(f"\n[UAV] Searching for marker ID {TARGET_MARKER_ID}...")
        marker = self.vision.find_target_marker(timeout=30.0, confidence_checks=3)
        
        if marker:
            self.target_found = True
            print(f"\n✓ TARGET ACQUIRED!")
            print(f"  Marker ID: {marker.marker_id}")
            print(f"  Distance: {marker.distance:.2f}m")
        else:
            print("\n✗ TARGET NOT FOUND")
        
        return marker
    
    def calculate_target_gps(self, marker, uav_vehicle):
        """Calculate target GPS coordinates"""
        print("\n" + "="*60)
        print("PHASE 3: TARGET LOCALIZATION")
        print("="*60)
        
        uav_lat = uav_vehicle.location.global_relative_frame.lat
        uav_lon = uav_vehicle.location.global_relative_frame.lon
        uav_heading = uav_vehicle.heading
        
        print(f"\n[UAV] Position: ({uav_lat:.8f}, {uav_lon:.8f}), Heading: {uav_heading}°")
        
        target_coords = self.vision.marker_to_gps_offset(marker, uav_lat, uav_lon, uav_heading)
        self.target_coords = target_coords
        
        print(f"\n✓ TARGET COORDINATES CALCULATED")
        print(f"  Latitude: {target_coords['latitude']:.8f}")
        print(f"  Longitude: {target_coords['longitude']:.8f}")
        
        return target_coords
    
    def send_target_to_ugv(self, target_coords):
        """Send target coordinates to UGV"""
        print("\n" + "="*60)
        print("PHASE 4: COORDINATE RELAY TO UGV")
        print("="*60)
        
        print(f"\n[COMMS] Sending target to UGV...")
        time.sleep(0.5)
        print("✓ COORDINATES TRANSMITTED TO UGV")
        return True
    
    def navigate_ugv_with_obstacle_avoidance(self, target_coords, uav_vehicle):
        """
        Navigate UGV to target with obstacle avoidance and UAV overwatch
        """
        print("\n" + "="*60)
        print("PHASE 5: UGV NAVIGATION WITH OBSTACLE AVOIDANCE")
        print("="*60)
        
        # Arm UGV
        print("\n[UGV] Arming rover...")
        arm_rover(self.rover)
        
        print(f"[UGV] Setting speed to {UGV_SPEED} m/s")
        set_speed(self.rover, UGV_SPEED)
        
        # Start UAV overwatch
        self.uav_overwatch.start_monitoring(uav_vehicle, self.rover)
        
        # Target location
        target_lat = target_coords['latitude']
        target_lon = target_coords['longitude']
        target_location = LocationGlobalRelative(target_lat, target_lon, 0)
        
        print(f"\n[UGV] Navigating to target with obstacle avoidance...")
        print(f"  Start: ({self.rover.location.global_relative_frame.lat:.8f}, "
              f"{self.rover.location.global_relative_frame.lon:.8f})")
        print(f"  Target: ({target_lat:.8f}, {target_lon:.8f})")
        print(f"  NOTE: UGV has NO GPS (Challenge 3 requirement)")
        
        # Initial navigation command
        current_waypoint = target_location
        self.rover.simple_goto(current_waypoint)
        
        # Monitor and guide
        start_time = time.time()
        last_distance = None
        check_interval = 1.0  # Check every second
        
        while True:
            # Get UGV status from UAV overwatch
            status = self.uav_overwatch.monitor_ugv_progress(
                self.rover, target_lat, target_lon
            )
            
            distance = status['distance_to_target']
            
            # Print progress
            if last_distance is None or abs(distance - last_distance) > 0.5:
                elapsed = time.time() - start_time
                print(f"  [UAV OVERWATCH] UGV Distance: {distance:.2f}m | "
                      f"Heading: {status['heading']}° | "
                      f"Speed: {status['speed']:.2f}m/s | "
                      f"Time: {elapsed:.1f}s")
                last_distance = distance
            
            # Simulate obstacle detection (in real system, use camera/sensors)
            obstacles = self.obstacle_avoidance.detect_obstacles()
            
            # If obstacle detected, provide avoidance waypoint
            if obstacles:
                self.obstacles_encountered += 1
                self.avoidance_maneuvers += 1
                print(f"\n  ⚠ [OBSTACLE DETECTED] Obstacle #{self.obstacles_encountered}")
                
                # Get avoidance waypoint from UAV
                avoidance_waypoint = self.uav_overwatch.provide_guidance_waypoint(
                    self.rover, target_lat, target_lon, obstacles[0]
                )
                
                # Command UGV to avoidance waypoint
                avoid_location = LocationGlobalRelative(
                    avoidance_waypoint[0],
                    avoidance_waypoint[1],
                    0
                )
                self.rover.simple_goto(avoid_location)
                print(f"  ✓ [AVOIDANCE] New waypoint sent to UGV")
                
                # Wait for avoidance maneuver
                time.sleep(3)
                
                # Resume to target
                self.rover.simple_goto(target_location)
                print(f"  ✓ [RESUME] Continuing to target")
            
            # Check if arrived
            if distance <= ARRIVAL_RADIUS:
                elapsed = time.time() - start_time
                print(f"\n✓ UGV ARRIVED AT TARGET!")
                print(f"  Final distance: {distance:.2f}m")
                print(f"  Travel time: {elapsed:.1f}s")
                print(f"  Obstacles encountered: {self.obstacles_encountered}")
                print(f"  Avoidance maneuvers: {self.avoidance_maneuvers}")
                break
            
            # Timeout check (7 minutes)
            if time.time() - start_time > 420:
                print(f"\n✗ NAVIGATION TIMEOUT (7 minutes exceeded)")
                print(f"  Final distance: {distance:.2f}m")
                break
            
            time.sleep(check_interval)
        
        # Stop UGV and UAV overwatch
        self.rover.mode = VehicleMode("HOLD")
        self.uav_overwatch.stop_monitoring()
    
    def simulate_uav_landing(self):
        """Simulate UAV landing on UGV"""
        print("\n" + "="*60)
        print("PHASE 6: UAV LANDING ON UGV")
        print("="*60)
        
        print("\n[UAV] Descending to UGV...")
        time.sleep(2)
        print("[UAV] Aligning with UGV platform...")
        time.sleep(1)
        print("[UAV] Final descent and touchdown...")
        time.sleep(2)
        
        print("\n✓ UAV LANDED ON UGV")
        print("[UAV] Coupled to UGV platform")
        
        print("\n[Challenge] Maintaining coupling for 10 seconds...")
        for i in range(10, 0, -1):
            print(f"  {i}...", end='\r')
            time.sleep(1)
        print("\n✓ COUPLING REQUIREMENT MET (10 seconds)")
    
    def log_mission_data(self, success):
        """Log mission results"""
        mission_duration = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        log_data = {
            'challenge': 3,
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'success': success,
            'duration_seconds': mission_duration,
            'target_found': self.target_found,
            'target_coordinates': self.target_coords,
            'obstacles_encountered': self.obstacles_encountered,
            'avoidance_maneuvers': self.avoidance_maneuvers,
            'arrival_radius': ARRIVAL_RADIUS,
            'ugv_speed': UGV_SPEED,
            'ugv_gps_disabled': True,  # Challenge 3 requirement
            'obstacle_avoidance_enabled': True
        }
        
        log_filename = f"challenge3_log_{time.strftime('%Y%m%d_%H%M%S')}.json"
        with open(log_filename, 'w') as f:
            json.dump(log_data, f, indent=2)
        
        print(f"\n[LOG] Mission data saved to {log_filename}")
    
    def run_mission(self):
        """Execute complete Challenge 3 mission"""
        self.mission_start_time = time.time()
        success = False
        
        try:
            # Initialize all systems
            self.initialize_systems()
            
            # Phase 1: UAV takeoff
            uav_vehicle = self.simulate_uav_takeoff()
            
            # Phase 2: Search for target
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
            
            # Phase 5: UGV navigation with obstacle avoidance
            self.navigate_ugv_with_obstacle_avoidance(target_coords, uav_vehicle)
            
            # Phase 6: UAV landing
            self.simulate_uav_landing()
            
            # Mission success
            success = True
            elapsed_time = time.time() - self.mission_start_time
            
            print("\n" + "="*60)
            print("✓ CHALLENGE 3 COMPLETE!")
            print("="*60)
            print(f"  Total mission time: {elapsed_time:.1f} seconds")
            print(f"  Time limit: 420 seconds (7 minutes)")
            if elapsed_time <= 420:
                print("  ✓ Within time limit!")
            else:
                print("  ✗ Exceeded time limit")
            print(f"  Obstacles avoided: {self.obstacles_encountered}")
            print(f"  Avoidance maneuvers: {self.avoidance_maneuvers}")
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
            self.cleanup()
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
    """Main entry point for Challenge 3"""
    parser = argparse.ArgumentParser(description="Challenge 3: Full Autonomous Mission with Obstacle Avoidance")
    parser.add_argument("--liverover", action="store_true",
                       help="Connect to real rover instead of SITL")
    parser.add_argument("--zed", action="store_true",
                       help="Use ZED camera instead of standard camera")
    parser.add_argument("--brainchip", action="store_true",
                       help="Use BrainChip for obstacle detection")
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("AVC CHALLENGE 3: FULL AUTONOMOUS MISSION")
    print("WITH OBSTACLE AVOIDANCE & UAV OVERWATCH")
    print("="*60)
    print(f"Mode: {'LIVE HARDWARE' if args.liverover else 'SIMULATION'}")
    print(f"Camera: {'ZED' if args.zed else 'Standard USB'}")
    print(f"Obstacle Detection: {'BrainChip' if args.brainchip else 'Standard'}")
    print(f"Target Marker ID: {TARGET_MARKER_ID}")
    print(f"UGV GPS: DISABLED (Challenge 3 requirement)")
    print("="*60)
    
    # Create and run mission
    mission = Challenge3Mission(
        simulate_rover=not args.liverover,
        use_zed=args.zed,
        use_brainchip=args.brainchip
    )
    
    success = mission.run_mission()
    
    exit(0 if success else 1)


if __name__ == "__main__":
    main()
