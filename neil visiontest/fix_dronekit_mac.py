"""
Fix DroneKit for Python 3.11+ on macOS
Patches the collections.MutableMapping issue
"""
import sys

def fix_dronekit():
    """Patch DroneKit to work with Python 3.11+"""
    try:
        import dronekit
        import os
        
        print("="*60)
        print("DroneKit Python 3.11+ Compatibility Patcher")
        print("="*60)
        print()
        
        # Find dronekit __init__.py file
        dronekit_file = dronekit.__file__
        print(f"Found DroneKit at: {dronekit_file}")
        
        # Read the file
        with open(dronekit_file, 'r') as f:
            content = f.read()
        
        # Check if already patched
        if 'collections.abc.MutableMapping' in content:
            print("✓ DroneKit is already patched!")
            return True
        
        # Check if needs patching
        if 'collections.MutableMapping' not in content:
            print("✗ DroneKit file doesn't contain the expected code")
            print("  This may be a different version or already modified")
            return False
        
        print("\nPatching DroneKit...")
        print("  Replacing: collections.MutableMapping")
        print("  With:      collections.abc.MutableMapping")
        
        # Make backup
        backup_file = dronekit_file + '.backup'
        with open(backup_file, 'w') as f:
            f.write(content)
        print(f"✓ Backup created: {backup_file}")
        
        # Apply patch
        patched_content = content.replace(
            'collections.MutableMapping',
            'collections.abc.MutableMapping'
        )
        
        # Write patched file
        with open(dronekit_file, 'w') as f:
            f.write(patched_content)
        
        print("✓ DroneKit patched successfully!")
        print()
        print("Testing import...")
        
        # Test import
        try:
            # Force reload
            import importlib
            importlib.reload(dronekit)
            print("✓ DroneKit imports successfully!")
            return True
        except Exception as e:
            print(f"✗ Import test failed: {e}")
            print("  Restoring backup...")
            with open(backup_file, 'r') as f:
                original = f.read()
            with open(dronekit_file, 'w') as f:
                f.write(original)
            print("✓ Backup restored")
            return False
            
    except ImportError:
        print("✗ DroneKit not installed!")
        print("  Install with: pip install dronekit")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


def check_python_version():
    """Check if Python version needs the patch"""
    version = sys.version_info
    print(f"Python version: {version.major}.{version.minor}.{version.micro}")
    
    if version.major == 3 and version.minor >= 10:
        print("⚠ Python 3.10+ detected - DroneKit patch needed")
        return True
    else:
        print("✓ Python version compatible with DroneKit")
        return False


def main():
    print()
    needs_patch = check_python_version()
    print()
    
    if needs_patch:
        success = fix_dronekit()
        print()
        print("="*60)
        if success:
            print("✓✓✓ DRONEKIT FIXED! ✓✓✓")
            print("="*60)
            print()
            print("You can now run: python test_challenge2_setup.py")
        else:
            print("✗✗✗ PATCH FAILED ✗✗✗")
            print("="*60)
            print()
            print("Alternative solutions:")
            print("1. Use Python 3.9: conda create -n challenge2 python=3.9")
            print("2. Manual patch: Edit dronekit/__init__.py line 2689")
            print("3. Install older DroneKit: pip install dronekit==2.9.1")
        print()
    else:
        print("No patch needed - your Python version should work!")
        print()


if __name__ == "__main__":
    main()
