#!/bin/bash

# Challenge 2 Quick Run Script
# Makes it easy to run Challenge 2 with different configurations

echo "======================================================================"
echo "           CHALLENGE 2: TARGET DETECTION & UGV NAVIGATION            "
echo "======================================================================"
echo ""

# Function to show menu
show_menu() {
    echo "Select run mode:"
    echo ""
    echo "  1) Simulation (SITL) with Standard Camera"
    echo "  2) Live Hardware with Standard Camera"
    echo "  3) Live Hardware with ZED Camera"
    echo "  4) Test Setup (verify all systems)"
    echo "  5) Generate Markers"
    echo "  6) Test Vision System"
    echo "  7) Exit"
    echo ""
}

# Function to run simulation
run_simulation() {
    echo "======================================================================"
    echo "Running Challenge 2 in SIMULATION mode"
    echo "======================================================================"
    python3 Challenge2_Main.py
}

# Function to run live hardware
run_live() {
    echo "======================================================================"
    echo "Running Challenge 2 with LIVE HARDWARE"
    echo "======================================================================"
    echo ""
    echo "⚠️  WARNING: This will control real hardware!"
    echo "   Make sure all safety measures are in place."
    echo ""
    read -p "Continue? (y/n): " confirm
    if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
        python3 Challenge2_Main.py --liverover
    else
        echo "Aborted."
    fi
}

# Function to run with ZED
run_zed() {
    echo "======================================================================"
    echo "Running Challenge 2 with LIVE HARDWARE + ZED CAMERA"
    echo "======================================================================"
    echo ""
    echo "⚠️  WARNING: This will control real hardware!"
    echo "   Make sure all safety measures are in place."
    echo ""
    read -p "Continue? (y/n): " confirm
    if [ "$confirm" = "y" ] || [ "$confirm" = "Y" ]; then
        python3 Challenge2_Main.py --liverover --zed
    else
        echo "Aborted."
    fi
}

# Function to test setup
test_setup() {
    echo "======================================================================"
    echo "Running Setup Verification"
    echo "======================================================================"
    python3 test_challenge2_setup.py
}

# Function to generate markers
generate_markers() {
    echo "======================================================================"
    echo "Generating ArUco Markers"
    echo "======================================================================"
    python3 generate_aruco_markers.py
    echo ""
    echo "Markers generated! Print target_marker_0.png at 10cm x 10cm"
}

# Function to test vision
test_vision() {
    echo "======================================================================"
    echo "Testing Vision System"
    echo "======================================================================"
    python3 test_aruco_setup.py
}

# Main loop
while true; do
    show_menu
    read -p "Enter choice [1-7]: " choice
    echo ""
    
    case $choice in
        1)
            run_simulation
            ;;
        2)
            run_live
            ;;
        3)
            run_zed
            ;;
        4)
            test_setup
            ;;
        5)
            generate_markers
            ;;
        6)
            test_vision
            ;;
        7)
            echo "Exiting..."
            exit 0
            ;;
        *)
            echo "Invalid choice. Please select 1-7."
            ;;
    esac
    
    echo ""
    echo "======================================================================"
    echo ""
    read -p "Press Enter to continue..."
    clear
done
