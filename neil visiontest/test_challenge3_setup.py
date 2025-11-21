"""
Challenge 3 Setup and Testing Script
Verifies all Challenge 3 components are ready
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

def test_challenge2_works():
    """Test that Challenge 2 is working (prerequisite)"""
    try:
        from Challenge2_Main import Challenge2Mission
        print("✓ Challenge 2 module working")
        return True
    except Exception as e:
        print(f"✗ Challenge 2 module failed: {e}")
        return False

def test_obstacle_avoidance():
    """Test obstacle avoidance module"""
    try:
        from Challenge3_Main import ObstacleAvoidance
        oa = ObstacleAvoidance(use_brainchip=False)
        print("✓ Obstacle avoidance module working")
        return True
    except Exception as e:
        print(f"✗ Obstacle avoidance failed: {e}")
        return False

def test_uav_overwatch():
    """Test UAV overwatch module"""
    try:
        from Challenge3_Main import UAVOverwatch
        from UAVVisionIntegration import VisionCoordinator
        
        vision = VisionCoordinator(
            calibration_file="calibration_chessboard.yaml",
            marker_size=0.1,
            use_zed=False
        )
        overwatch = UAVOverwatch(vision)
        vision.close()
        
        print("✓ UAV overwatch module working")
        return True
    except Exception as e:
        print(f"✗ UAV overwatch failed: {e}")
        return False

def run_all_tests():
    """Run all Challenge 3 verification tests"""
    print("="*70)
    print("CHALLENGE 3 SETUP VERIFICATION")
    print("="*70)
    print()
    
    results = []
    
    # Test required files
    print("[1/6] Checking Required Files...")
    print("-" * 70)
    results.append(test_file_exists("Challenge3_Main.py", "Challenge 3 main script"))
    results.append(test_file_exists("Challenge2_Main.py", "Challenge 2 (prerequisite)"))
    results.append(test_file_exists("UAVVision.py", "Vision module"))
    results.append(test_file_exists("UAVVisionIntegration.py", "Integration module"))
    results.append(test_file_exists("UGVComms.py", "Communication module"))
    results.append(test_file_exists("UGVConnect.py", "UGV connection module"))
    results.append(test_file_exists("calibration_chessboard.yaml", "Calibration file"))
    print()
    
    # Test Python dependencies
    print("[2/6] Checking Python Dependencies...")
    print("-" * 70)
    results.append(test_import("cv2", "OpenCV"))
    results.append(test_import("numpy", "NumPy"))
    results.append(test_import("dronekit", "DroneKit"))
    results.append(test_import("pymavlink", "PyMAVLink"))
    print()
    
    # Test Challenge 2 works
    print("[3/6] Testing Challenge 2 (Prerequisite)...")
    print("-" * 70)
    results.append(test_challenge2_works())
    print()
    
    # Test obstacle avoidance
    print("[4/6] Testing Obstacle Avoidance Module...")
    print("-" * 70)
    results.append(test_obstacle_avoidance())
    print()
    
    # Test UAV overwatch
    print("[5/6] Testing UAV Overwatch Module...")
    print("-" * 70)
    results.append(test_uav_overwatch())
    print()
    
    # Test custom modules
    print("[6/6] Testing Custom Modules...")
    print("-" * 70)
    results.append(test_import("Challenge3_Main", "Challenge 3 Main"))
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
        print("✓✓✓ ALL TESTS PASSED! CHALLENGE 3 IS READY! ✓✓✓")
        print("="*70)
        print()
        print("Challenge 3 Features:")
        print("  ✓ Target detection (from Challenge 2)")
        print("  ✓ Coordinate relay (from Challenge 2)")
        print("  ✓ Obstacle detection and avoidance")
        print("  ✓ UAV overwatch and guidance")
        print("  ✓ UGV navigation without GPS")
        print()
        print("Next Steps:")
        print("1. Test Challenge 2 first: python Challenge2_Main.py")
        print("2. Then test Challenge 3: python Challenge3_Main.py")
        print("3. For live hardware: python Challenge3_Main.py --liverover")
        print()
        return True
    else:
        print("="*70)
        print("✗✗✗ SOME TESTS FAILED - FIX ISSUES BEFORE PROCEEDING ✗✗✗")
        print("="*70)
        print()
        print("Common Fixes:")
        print("- Make sure Challenge 2 is working first")
        print("- Ensure all dependencies installed")
        print("- Check that Challenge3_Main.py is created")
        print()
        return False

def show_challenge3_info():
    """Show Challenge 3 specific information"""
    print()
    print("="*70)
    print("CHALLENGE 3 REQUIREMENTS")
    print("="*70)
    print()
    print("New Features in Challenge 3:")
    print("  1. Obstacle Detection - UGV detects obstacles in path")
    print("  2. Obstacle Avoidance - UGV plans alternate routes")
    print("  3. UAV Overwatch - UAV monitors and guides UGV")
    print("  4. No GPS on UGV - Navigation via UAV guidance only")
    print("  5. Dynamic Path Planning - Real-time route adjustments")
    print()
    print("Hardware Options:")
    print("  --brainchip : Use BrainChip for obstacle detection")
    print("  --zed       : Use ZED camera for depth perception")
    print()
    print("Competition Requirements:")
    print("  • All Challenge 2 requirements PLUS:")
    print("  • UGV must avoid ground obstacles")
    print("  • UGV operates without GPS")
    print("  • UAV provides continuous guidance")
    print("  • Complete within 7 minutes")
    print("="*70)

if __name__ == "__main__":
    success = run_all_tests()
    
    if success:
        show_challenge3_info()
    
    sys.exit(0 if success else 1)
