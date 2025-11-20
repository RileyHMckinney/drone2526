"""
Challenge 2 Setup Verification Script
Run this to verify all components are ready before running Challenge 2
"""
import os
import sys
import importlib

def test_file_exists(filename, description):
    """Test if a required file exists"""
    if os.path.exists(filename):
        print(f"✓ {description}: {filename}")
        return True
    else:
        print(f"✗ {description} MISSING: {filename}")
        return False

def test_import(module_name, description):
    """Test if a module can be imported"""
    try:
        importlib.import_module(module_name)
        print(f"✓ {description} imported successfully")
        return True
    except ImportError as e:
        print(f"✗ {description} import failed: {e}")
        return False

def test_camera():
    """Test camera availability"""
    try:
        import cv2
        cap = cv2.VideoCapture(0)
        if cap.isOpened():
            ret, frame = cap.read()
            cap.release()
            if ret:
                print("✓ Camera is working")
                return True
            else:
                print("✗ Camera opened but can't read frames")
                return False
        else:
            print("✗ Camera failed to open")
            return False
    except Exception as e:
        print(f"✗ Camera test failed: {e}")
        return False

def test_calibration():
    """Test calibration file"""
    try:
        import cv2
        fs = cv2.FileStorage("calibration_chessboard.yaml", cv2.FILE_STORAGE_READ)
        K = fs.getNode("K").mat()
        D = fs.getNode("D").mat()
        fs.release()
        
        if K is not None and D is not None:
            print(f"✓ Calibration file valid (K: {K.shape}, D: {D.shape})")
            return True
        else:
            print("✗ Calibration file missing data")
            return False
    except Exception as e:
        print(f"✗ Calibration test failed: {e}")
        return False

def test_aruco_detection():
    """Quick ArUco detection test"""
    try:
        import cv2
        import cv2.aruco as aruco
        
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        print("✓ ArUco dictionary loaded")
        
        # Try to generate a test marker
        marker = aruco.generateImageMarker(aruco_dict, 0, 200)
        if marker is not None:
            print("✓ ArUco marker generation works")
            return True
        else:
            print("✗ ArUco marker generation failed")
            return False
    except Exception as e:
        print(f"✗ ArUco test failed: {e}")
        return False

def test_dronekit():
    """Test DroneKit availability and patch if needed"""
    try:
        import dronekit

        # Safely get version info if available
        version = getattr(dronekit, "__version__", "unknown")
        print(f"✓ DroneKit imported successfully (version: {version})")

        # Check and patch for Python 3.10+ compatibility
        try:
            import fileinput

            with open(dronekit.__file__, "r", encoding="utf-8") as f:
                content = f.read()

            if "collections.MutableMapping" in content:
                print("⚙ Detected deprecated 'collections.MutableMapping' — patching DroneKit...")
                with fileinput.FileInput(dronekit.__file__, inplace=True, backup=".bak") as file:
                    for line in file:
                        print(line.replace("collections.MutableMapping", "collections.abc.MutableMapping"), end="")
                print("✓ DroneKit patched successfully (backup: .bak)")
            else:
                print("✓ DroneKit is already compatible")

        except Exception as patch_error:
            print(f"⚠ Could not check/patch DroneKit: {patch_error}")

        return True

    except ImportError as e:
        # Handle missing dependencies like 'past'
        print(f"✗ DroneKit not installed or failed to import: {e}")
        if "No module named 'past'" in str(e):
            print("⚙ Missing dependency detected: installing 'future' package...")
            import subprocess, sys
            subprocess.run([sys.executable, "-m", "pip", "install", "future"], check=False)
            return test_dronekit()
        else:
            return False

    except Exception as e:
        print(f"✗ DroneKit test failed: {e}")
        return False



def run_all_tests():
    """Run all verification tests"""
    print("="*70)
    print("CHALLENGE 2 SETUP VERIFICATION")
    print("="*70)
    print()
    
    results = []
    
    # Test required files
    print("[1/8] Checking Required Files...")
    print("-" * 70)
    results.append(test_file_exists("Challenge2_Main.py", "Main script"))
    results.append(test_file_exists("UAVVision.py", "Vision module"))
    results.append(test_file_exists("UAVVisionIntegration.py", "Integration module"))
    results.append(test_file_exists("UGVComms.py", "Communication module"))
    results.append(test_file_exists("UGVConnect.py", "UGV connection module"))
    results.append(test_file_exists("calibration_chessboard.yaml", "Calibration file"))
    print()
    
    # Test Python dependencies
    print("[2/8] Checking Python Dependencies...")
    print("-" * 70)
    results.append(test_import("cv2", "OpenCV"))
    results.append(test_import("numpy", "NumPy"))
    results.append(test_import("dronekit", "DroneKit"))
    results.append(test_import("pymavlink", "PyMAVLink"))
    print()
    
    # Test OpenCV ArUco
    print("[3/8] Testing OpenCV ArUco...")
    print("-" * 70)
    try:
        import cv2
        print(f"OpenCV version: {cv2.__version__}")
        results.append(test_import("cv2.aruco", "ArUco module"))
    except:
        results.append(False)
    print()
    
    # Test camera
    print("[4/8] Testing Camera...")
    print("-" * 70)
    results.append(test_camera())
    print()
    
    # Test calibration
    print("[5/8] Testing Calibration File...")
    print("-" * 70)
    results.append(test_calibration())
    print()
    
    # Test ArUco detection
    print("[6/8] Testing ArUco Detection...")
    print("-" * 70)
    results.append(test_aruco_detection())
    print()
    
    # Test DroneKit
    print("[7/8] Testing DroneKit...")
    print("-" * 70)
    results.append(test_dronekit())
    print()
    
    # Test module imports
    print("[8/8] Testing Custom Modules...")
    print("-" * 70)
    results.append(test_import("UGVConnect", "UGV Connect"))
    results.append(test_import("UAVVision", "UAV Vision"))
    results.append(test_import("UAVVisionIntegration", "Vision Integration"))
    results.append(test_import("UGVComms", "UGV Communications"))
    print()
    
    # Summary
    print("="*70)
    print("VERIFICATION SUMMARY")
    print("="*70)
    
    total_tests = len(results)
    passed_tests = sum(results)
    failed_tests = total_tests - passed_tests
    
    print(f"Total Tests: {total_tests}")
    print(f"Passed: {passed_tests}")
    print(f"Failed: {failed_tests}")
    print(f"Success Rate: {(passed_tests/total_tests)*100:.1f}%")
    print()
    
    if all(results):
        print("="*70)
        print("✓✓✓ ALL TESTS PASSED! YOU'RE READY FOR CHALLENGE 2! ✓✓✓")
        print("="*70)
        print()
        print("Next Steps:")
        print("1. Generate markers: python generate_aruco_markers.py")
        print("2. Print target_marker_0.png at 10cm x 10cm")
        print("3. Test vision: python test_aruco_setup.py")
        print("4. Run Challenge 2: python Challenge2_Main.py")
        print()
        return True
    else:
        print("="*70)
        print("✗✗✗ SOME TESTS FAILED - FIX ISSUES BEFORE PROCEEDING ✗✗✗")
        print("="*70)
        print()
        print("Common Fixes:")
        if not test_import("cv2", "test"):
            print("- Install OpenCV: pip install opencv-contrib-python")
        if not test_import("numpy", "test"):
            print("- Install NumPy: pip install numpy")
        if not test_import("dronekit", "test"):
            print("- Install DroneKit: pip install dronekit")
        if not test_import("pymavlink", "test"):
            print("- Install PyMAVLink: pip install pymavlink")
        print()
        return False

def show_quick_help():
    """Show quick help information"""
    print()
    print("="*70)
    print("QUICK HELP")
    print("="*70)
    print()
    print("If you're missing files, make sure you created all the modules:")
    print("  - Challenge2_Main.py")
    print("  - UAVVision.py")
    print("  - UAVVisionIntegration.py")
    print("  - UGVComms.py")
    print("  - generate_aruco_markers.py")
    print("  - test_aruco_setup.py")
    print()
    print("If dependencies are missing:")
    print("  pip install opencv-contrib-python numpy dronekit pymavlink")
    print()
    print("If camera test fails:")
    print("  - Check camera is connected")
    print("  - Try a different camera index in the code")
    print("  - Ensure no other program is using the camera")
    print()
    print("For more help, see CHALLENGE2_README.md")
    print("="*70)

if __name__ == "__main__":
    success = run_all_tests()
    
    if not success:
        show_quick_help()
    
    sys.exit(0 if success else 1)
