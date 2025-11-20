"""
UAV Vision Integration for Challenge 2
Integrates ArUco detection with UAV-UGV communication
"""
from UAVVision import UAVVision, MarkerPosition
# from UGVComms import send_coordinates_to_ugv  # Uncomment when UGVComms is ready
import time
import json
from typing import Optional, Dict
from dataclasses import asdict

class VisionCoordinator:
    """Coordinates vision detection with UAV-UGV communication"""
    
    def __init__(self, calibration_file: str = "calibration_chessboard.yaml",
                 marker_size: float = 0.1, use_zed: bool = False,
                 target_marker_id: int = 0):
        """
        Initialize vision coordinator
        
        Args:
            calibration_file: Camera calibration file path
            marker_size: ArUco marker size in meters
            use_zed: Use ZED camera if True
            target_marker_id: ID of the target marker to track
        """
        self.vision = UAVVision(calibration_file, marker_size, use_zed)
        self.target_marker_id = target_marker_id
        self.last_detection_time = None
        self.detection_count = 0
        
    def find_target_marker(self, timeout: float = 30.0, 
                          confidence_checks: int = 3) -> Optional[MarkerPosition]:
        """
        Search for target marker with confidence checks
        
        Args:
            timeout: Maximum search time in seconds
            confidence_checks: Number of consecutive detections required
        
        Returns:
            MarkerPosition if target found reliably, None otherwise
        """
        print(f"Searching for target marker (ID {self.target_marker_id})...")
        start_time = time.time()
        consecutive_detections = 0
        last_position = None
        
        while (time.time() - start_time) < timeout:
            positions, _ = self.vision.process_frame(display=True)
            
            if positions:
                # Look for target marker
                target = next((p for p in positions 
                             if p.marker_id == self.target_marker_id), None)
                
                if target:
                    consecutive_detections += 1
                    last_position = target
                    print(f"Detection {consecutive_detections}/{confidence_checks}: "
                          f"Distance={target.distance:.2f}m")
                    
                    if consecutive_detections >= confidence_checks:
                        print(f"✓ Target confirmed at {target.distance:.2f}m")
                        self.last_detection_time = time.time()
                        self.detection_count += 1
                        return target
                else:
                    consecutive_detections = 0
            
            time.sleep(0.1)  # 10Hz check rate
        
        print("✗ Target marker not found within timeout")
        return None
    
    def marker_to_gps_offset(self, marker_pos: MarkerPosition, 
                            uav_lat: float, uav_lon: float,
                            uav_heading: float) -> Dict[str, float]:
        """
        Convert marker position to GPS coordinates
        
        Args:
            marker_pos: Detected marker position
            uav_lat: UAV current latitude
            uav_lon: UAV current longitude
            uav_heading: UAV heading in degrees
        
        Returns:
            Dict with target GPS coordinates
        """
        # Convert camera frame to world frame
        # Camera frame: X=right, Y=forward, Z=down
        # This is a simplified conversion - adjust based on your coordinate system
        
        import math
        
        # Convert heading to radians
        heading_rad = math.radians(uav_heading)
        
        # Rotate marker position by UAV heading
        forward = marker_pos.z  # Distance forward
        right = marker_pos.x    # Distance right
        
        # World frame offsets
        north_offset = forward * math.cos(heading_rad) - right * math.sin(heading_rad)
        east_offset = forward * math.sin(heading_rad) + right * math.cos(heading_rad)
        
        # Convert meters to degrees (approximate)
        # 1 degree latitude ≈ 111,111 meters
        # 1 degree longitude ≈ 111,111 * cos(latitude) meters
        lat_offset = north_offset / 111111.0
        lon_offset = east_offset / (111111.0 * math.cos(math.radians(uav_lat)))
        
        target_lat = uav_lat + lat_offset
        target_lon = uav_lon + lon_offset
        
        return {
            'latitude': target_lat,
            'longitude': target_lon,
            'altitude': 0,  # Ground level
            'marker_id': marker_pos.marker_id,
            'distance': marker_pos.distance
        }
    
    def send_target_to_ugv(self, marker_pos: MarkerPosition,
                          uav_lat: float, uav_lon: float,
                          uav_heading: float) -> bool:
        """
        Calculate target GPS and send to UGV
        
        Args:
            marker_pos: Detected marker position
            uav_lat: UAV latitude
            uav_lon: UAV longitude  
            uav_heading: UAV heading
        
        Returns:
            True if successfully sent, False otherwise
        """
        # Calculate target GPS coordinates
        target_coords = self.marker_to_gps_offset(
            marker_pos, uav_lat, uav_lon, uav_heading
        )
        
        print(f"\n>>> Sending target to UGV:")
        print(f"    Marker ID: {target_coords['marker_id']}")
        print(f"    Latitude: {target_coords['latitude']:.8f}")
        print(f"    Longitude: {target_coords['longitude']:.8f}")
        print(f"    Distance: {target_coords['distance']:.2f}m")
        
        # Send to UGV via your communication system
        # Uncomment and implement based on your UGVComms module:
        # try:
        #     send_coordinates_to_ugv(target_coords)
        #     print("✓ Coordinates sent to UGV")
        #     return True
        # except Exception as e:
        #     print(f"✗ Failed to send coordinates: {e}")
        #     return False
        
        # For testing without UGV comms:
        print("(Simulated send - implement UGVComms integration)")
        return True
    
    def run_challenge_2_sequence(self, uav_vehicle):
        """
        Run the complete Challenge 2 sequence:
        1. Search for target marker
        2. Calculate GPS coordinates
        3. Send to UGV
        
        Args:
            uav_vehicle: DroneKit vehicle object with location and heading
        
        Returns:
            True if sequence completed successfully
        """
        print("\n" + "="*50)
        print("CHALLENGE 2: Target Detection and Relay")
        print("="*50)
        
        # Step 1: Find target marker
        marker = self.find_target_marker(timeout=30.0, confidence_checks=3)
        
        if marker is None:
            print("✗ Challenge 2 failed: Target marker not found")
            return False
        
        # Step 2: Get UAV position
        uav_lat = uav_vehicle.location.global_relative_frame.lat
        uav_lon = uav_vehicle.location.global_relative_frame.lon
        uav_heading = uav_vehicle.heading
        
        print(f"\nUAV Position: ({uav_lat:.8f}, {uav_lon:.8f}), Heading: {uav_heading}°")
        
        # Step 3: Send to UGV
        success = self.send_target_to_ugv(marker, uav_lat, uav_lon, uav_heading)
        
        if success:
            print("\n✓ Challenge 2 sequence completed successfully")
        else:
            print("\n✗ Challenge 2 failed: Communication error")
        
        return success
    
    def close(self):
        """Cleanup resources"""
        self.vision.close()


def test_vision_only():
    """Test vision system without UAV/UGV"""
    print("Vision System Test (No UAV/UGV)")
    coordinator = VisionCoordinator(marker_size=0.1, use_zed=False)
    
    try:
        # Simulate finding target
        marker = coordinator.find_target_marker(timeout=60.0)
        
        if marker:
            # Simulate UAV position
            fake_lat = 32.9201927
            fake_lon = -96.9483108
            fake_heading = 90.0
            
            target_gps = coordinator.marker_to_gps_offset(
                marker, fake_lat, fake_lon, fake_heading
            )
            
            print(f"\nTarget GPS: {target_gps}")
    
    finally:
        coordinator.close()


def run_with_uav(vehicle):
    """
    Run vision coordinator with actual UAV
    Call this from your main UAV script
    
    Args:
        vehicle: DroneKit Vehicle object
    """
    coordinator = VisionCoordinator(
        calibration_file="calibration_chessboard.yaml",
        marker_size=0.1,
        use_zed=False,
        target_marker_id=0
    )
    
    try:
        success = coordinator.run_challenge_2_sequence(vehicle)
        return success
    finally:
        coordinator.close()


if __name__ == "__main__":
    # Run standalone test
    test_vision_only()
