"""
Quick test script to verify ArUco detection setup
Run this to make sure everything is working before integrating with UAV/UGV
"""
import cv2
import cv2.aruco as aruco
import numpy as np
import os
import sys

def test_opencv_aruco():
    """Test that OpenCV and ArUco are installed correctly"""
    print("Testing OpenCV and ArUco installation...")
    try:
        print(f"✓ OpenCV version: {cv2.__version__}")
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        print("✓ ArUco module loaded successfully")
        return True
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def test_camera():
    """Test camera connection"""
    print("\nTesting camera connection...")
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
        if not cap.isOpened():
            print("✗ Camera failed to open")
            return False
        
        ret, frame = cap.read()
        if not ret:
            print("✗ Failed to read frame from camera")
            cap.release()
            return False
        
        height, width = frame.shape[:2]
        print(f"✓ Camera opened successfully")
        print(f"  Resolution: {width}x{height}")
        cap.release()
        return True
    except Exception as e:
        print(f"✗ Camera error: {e}")
        return False

def test_calibration_file():
    """Test that calibration file exists and is valid"""
    print("\nTesting calibration file...")
    calib_file = "calibration_chessboard.yaml"
    
    if not os.path.exists(calib_file):
        print(f"✗ Calibration file not found: {calib_file}")
        return False
    
    try:
        fs = cv2.FileStorage(calib_file, cv2.FILE_STORAGE_READ)
        K = fs.getNode("K").mat()
        D = fs.getNode("D").mat()
        fs.release()
        
        if K is None or D is None:
            print("✗ Calibration file is missing data")
            return False
        
        print("✓ Calibration file loaded successfully")
        print(f"  Camera matrix shape: {K.shape}")
        print(f"  Distortion coeffs shape: {D.shape}")
        return True
    except Exception as e:
        print(f"✗ Error reading calibration file: {e}")
        return False

def test_live_detection():
    """Test live ArUco detection with camera"""
    print("\nTesting live ArUco detection...")
    print("Instructions:")
    print("1. Display a marker on your phone/tablet from: https://chev.me/arucogen/")
    print("2. Select DICT_6X6_1000, ID=0, size=500px")
    print("3. Hold it in front of the camera")
    print("4. Press 'q' to quit\n")
    
    try:
        # Load calibration
        fs = cv2.FileStorage("calibration_chessboard.yaml", cv2.FILE_STORAGE_READ)
        camera_matrix = fs.getNode("K").mat()
        dist_coeffs = fs.getNode("D").mat()
        fs.release()
        
        # Open camera
        cap = cv2.VideoCapture(0, cv2.CAP_AVFOUNDATION)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        marker_size = 0.1  # 10cm
        
        print("Camera feed started. Waiting for markers...")
        detection_count = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            
            # Detect markers
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
            
            # Process detections
            if ids is not None:
                frame = aruco.drawDetectedMarkers(frame, corners, ids)
                
                for i, marker_id in enumerate(ids.flatten()):
                    # Estimate pose
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i], marker_size, camera_matrix, dist_coeffs
                    )
                    
                    tvec = tvecs[0].flatten()
                    distance = np.linalg.norm(tvec)
                    
                    # Draw axes
                    cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                    rvecs[0], tvecs[0], marker_size * 0.5)
                    
                    # Display info
                    info = f"ID:{marker_id} Dist:{distance:.2f}m"
                    cv2.putText(frame, info, (10, 30 + i * 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                    
                    detection_count += 1
                    if detection_count % 30 == 0:  # Print every 30 frames
                        print(f"Detected marker {marker_id} at {distance:.2f}m")
            
            # Show status
            status = f"Detections: {detection_count}"
            cv2.putText(frame, status, (10, frame.shape[0] - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            
            cv2.imshow("ArUco Detection Test", frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        cap.release()
        cv2.destroyAllWindows()
        
        if detection_count > 0:
            print(f"\n✓ Detection test passed! Detected markers {detection_count} times")
            return True
        else:
            print("\n⚠ No markers detected. Make sure you have a marker visible.")
            return False
            
    except Exception as e:
        print(f"✗ Detection test failed: {e}")
        return False

def run_all_tests():
    """Run all tests"""
    print("="*60)
    print("ArUco Detection Setup Test")
    print("="*60)
    
    results = []
    results.append(("OpenCV/ArUco", test_opencv_aruco()))
    results.append(("Camera", test_camera()))
    results.append(("Calibration File", test_calibration_file()))
    
    # Only run live test if previous tests passed
    if all([r[1] for r in results]):
        input("\nPress Enter to start live detection test...")
        results.append(("Live Detection", test_live_detection()))
    else:
        print("\n⚠ Skipping live detection test due to previous failures")
    
    # Summary
    print("\n" + "="*60)
    print("TEST SUMMARY")
    print("="*60)
    for name, passed in results:
        status = "✓ PASS" if passed else "✗ FAIL"
        print(f"{name:.<40} {status}")
    
    all_passed = all([r[1] for r in results])
    print("="*60)
    if all_passed:
        print("✓ ALL TESTS PASSED! You're ready to integrate with UAV/UGV")
    else:
        print("✗ SOME TESTS FAILED. Fix the issues above before proceeding.")
    print("="*60)
    
    return all_passed

if __name__ == "__main__":
    run_all_tests()
