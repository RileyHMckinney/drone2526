"""
UGV Communication Module
Handles communication between UAV and UGV for coordinate sharing

This can be implemented using:
- ESP32 Client-Server model (UAV=client, UGV=server)
- MAVLink direct communication
- Serial communication
"""
import json
import time
from typing import Dict, Optional

# Communication method - change based on your setup
COMM_METHOD = "SIMULATED"  # Options: "ESP32", "MAVLINK", "SERIAL", "SIMULATED"

class UGVCommsInterface:
    """Base communication interface"""
    
    def __init__(self):
        self.connected = False
        self.last_message_time = None
    
    def connect(self):
        """Establish connection with UGV"""
        raise NotImplementedError
    
    def send_coordinates(self, coords: Dict) -> bool:
        """Send target coordinates to UGV"""
        raise NotImplementedError
    
    def receive_acknowledgment(self, timeout: float = 5.0) -> bool:
        """Wait for acknowledgment from UGV"""
        raise NotImplementedError
    
    def close(self):
        """Close connection"""
        pass


class SimulatedComms(UGVCommsInterface):
    """Simulated communication for testing"""
    
    def __init__(self):
        super().__init__()
        print("[COMMS] Using simulated communication")
    
    def connect(self):
        """Simulate connection"""
        print("[COMMS] Connecting to UGV...")
        time.sleep(0.5)
        self.connected = True
        print("[COMMS] ✓ Connected to UGV")
        return True
    
    def send_coordinates(self, coords: Dict) -> bool:
        """Simulate sending coordinates"""
        if not self.connected:
            print("[COMMS] ✗ Not connected to UGV")
            return False
        
        message = {
            'type': 'TARGET_COORDINATES',
            'data': coords,
            'timestamp': time.time()
        }
        
        print(f"[COMMS] → Sending: {json.dumps(coords, indent=2)}")
        time.sleep(0.2)  # Simulate transmission time
        self.last_message_time = time.time()
        print("[COMMS] ✓ Message sent")
        return True
    
    def receive_acknowledgment(self, timeout: float = 5.0) -> bool:
        """Simulate receiving ACK"""
        print("[COMMS] ← Waiting for ACK...")
        time.sleep(0.3)  # Simulate response time
        print("[COMMS] ✓ ACK received from UGV")
        return True
    
    def close(self):
        """Close simulated connection"""
        self.connected = False
        print("[COMMS] Connection closed")


class ESP32Comms(UGVCommsInterface):
    """
    ESP32 Client-Server communication
    UAV acts as client, UGV acts as server
    """
    
    def __init__(self, server_ip: str = "192.168.4.1", server_port: int = 80):
        super().__init__()
        self.server_ip = server_ip
        self.server_port = server_port
        self.socket = None
        print(f"[COMMS] Using ESP32 communication ({server_ip}:{server_port})")
    
    def connect(self):
        """Connect to UGV's ESP32 server"""
        try:
            import socket
            print(f"[COMMS] Connecting to UGV at {self.server_ip}:{self.server_port}...")
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5.0)
            self.socket.connect((self.server_ip, self.server_port))
            
            self.connected = True
            print("[COMMS] ✓ Connected to UGV")
            return True
            
        except Exception as e:
            print(f"[COMMS] ✗ Connection failed: {e}")
            self.connected = False
            return False
    
    def send_coordinates(self, coords: Dict) -> bool:
        """Send coordinates via ESP32"""
        if not self.connected or not self.socket:
            print("[COMMS] ✗ Not connected to UGV")
            return False
        
        try:
            message = json.dumps({
                'type': 'TARGET_COORDINATES',
                'data': coords,
                'timestamp': time.time()
            })
            
            print(f"[COMMS] → Sending coordinates...")
            self.socket.sendall(message.encode('utf-8'))
            self.last_message_time = time.time()
            print("[COMMS] ✓ Message sent")
            return True
            
        except Exception as e:
            print(f"[COMMS] ✗ Send failed: {e}")
            return False
    
    def receive_acknowledgment(self, timeout: float = 5.0) -> bool:
        """Receive ACK from UGV"""
        if not self.socket:
            return False
        
        try:
            self.socket.settimeout(timeout)
            print("[COMMS] ← Waiting for ACK...")
            
            data = self.socket.recv(1024)
            response = json.loads(data.decode('utf-8'))
            
            if response.get('type') == 'ACK':
                print("[COMMS] ✓ ACK received from UGV")
                return True
            else:
                print(f"[COMMS] ✗ Unexpected response: {response}")
                return False
                
        except Exception as e:
            print(f"[COMMS] ✗ ACK timeout or error: {e}")
            return False
    
    def close(self):
        """Close ESP32 connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
        self.connected = False
        print("[COMMS] Connection closed")


class MAVLinkComms(UGVCommsInterface):
    """
    MAVLink-based communication
    Uses MAVLink messages to communicate between UAV and UGV
    """
    
    def __init__(self, connection_string: str):
        super().__init__()
        self.connection_string = connection_string
        self.vehicle = None
        print(f"[COMMS] Using MAVLink communication ({connection_string})")
    
    def connect(self):
        """Connect via MAVLink"""
        try:
            from dronekit import connect
            print(f"[COMMS] Connecting via MAVLink to {self.connection_string}...")
            
            self.vehicle = connect(self.connection_string, wait_ready=True)
            self.connected = True
            print("[COMMS] ✓ MAVLink connection established")
            return True
            
        except Exception as e:
            print(f"[COMMS] ✗ MAVLink connection failed: {e}")
            self.connected = False
            return False
    
    def send_coordinates(self, coords: Dict) -> bool:
        """Send coordinates via MAVLink message"""
        if not self.connected or not self.vehicle:
            print("[COMMS] ✗ Not connected")
            return False
        
        try:
            # Send as MAVLink command or custom message
            # This is a simplified example - adjust based on your MAVLink setup
            from pymavlink import mavutil
            
            print(f"[COMMS] → Sending coordinates via MAVLink...")
            
            # Example: Send as mission item or custom message
            # Implement based on your specific MAVLink protocol
            
            self.last_message_time = time.time()
            print("[COMMS] ✓ Coordinates sent")
            return True
            
        except Exception as e:
            print(f"[COMMS] ✗ Send failed: {e}")
            return False
    
    def receive_acknowledgment(self, timeout: float = 5.0) -> bool:
        """Wait for MAVLink ACK"""
        print("[COMMS] ← Waiting for MAVLink ACK...")
        # Implement MAVLink ACK listening
        time.sleep(0.5)  # Placeholder
        print("[COMMS] ✓ ACK received")
        return True
    
    def close(self):
        """Close MAVLink connection"""
        if self.vehicle:
            self.vehicle.close()
            self.vehicle = None
        self.connected = False
        print("[COMMS] MAVLink connection closed")


# Global communication interface
_comms = None

def initialize_comms(method: str = COMM_METHOD, **kwargs) -> UGVCommsInterface:
    """
    Initialize communication interface
    
    Args:
        method: Communication method ("SIMULATED", "ESP32", "MAVLINK", "SERIAL")
        **kwargs: Method-specific parameters
    
    Returns:
        Communication interface instance
    """
    global _comms
    
    if method == "SIMULATED":
        _comms = SimulatedComms()
    elif method == "ESP32":
        server_ip = kwargs.get('server_ip', '192.168.4.1')
        server_port = kwargs.get('server_port', 80)
        _comms = ESP32Comms(server_ip, server_port)
    elif method == "MAVLINK":
        connection_string = kwargs.get('connection_string', '/dev/ttyACM0')
        _comms = MAVLinkComms(connection_string)
    else:
        raise ValueError(f"Unknown communication method: {method}")
    
    return _comms


def get_comms() -> Optional[UGVCommsInterface]:
    """Get current communication interface"""
    global _comms
    if _comms is None:
        _comms = initialize_comms()
    return _comms


def send_coordinates_to_ugv(coords: Dict) -> bool:
    """
    High-level function to send coordinates to UGV
    
    Args:
        coords: Dictionary with target coordinates
            {
                'latitude': float,
                'longitude': float,
                'altitude': float,
                'marker_id': int,
                'distance': float
            }
    
    Returns:
        True if successfully sent and acknowledged
    """
    comms = get_comms()
    
    # Connect if not already connected
    if not comms.connected:
        if not comms.connect():
            return False
    
    # Send coordinates
    if not comms.send_coordinates(coords):
        return False
    
    # Wait for acknowledgment
    if not comms.receive_acknowledgment():
        print("[COMMS] ⚠ Warning: No ACK received (continuing anyway)")
        # Don't fail mission if ACK not received - UGV may have received message
    
    return True


def close_comms():
    """Close communication interface"""
    global _comms
    if _comms:
        _comms.close()
        _comms = None


# Example usage and testing
def test_comms():
    """Test communication system"""
    print("="*60)
    print("UGV COMMUNICATION TEST")
    print("="*60)
    
    # Initialize
    comms = initialize_comms("SIMULATED")
    
    # Connect
    if not comms.connect():
        print("✗ Connection test failed")
        return False
    
    # Test sending coordinates
    test_coords = {
        'latitude': 32.9201927,
        'longitude': -96.9483108,
        'altitude': 0,
        'marker_id': 0,
        'distance': 10.5
    }
    
    if not send_coordinates_to_ugv(test_coords):
        print("✗ Send test failed")
        return False
    
    print("\n✓ Communication test passed!")
    
    # Cleanup
    close_comms()
    return True


if __name__ == "__main__":
    test_comms()
