#!/bin/bash
# Quick Start Script for State Estimation Testing

echo "================================================"
echo "State Estimation Testing Script"
echo "================================================"
echo ""

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 is not sourced!"
    echo "Please run: source /opt/ros/jazzy/setup.bash"
    exit 1
fi

echo "ROS2 Distribution: $ROS_DISTRO"
echo ""

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check dependencies
echo "Checking dependencies..."
MISSING_DEPS=false

if ! python3 -c "import numpy" 2>/dev/null; then
    echo "ERROR: numpy not found. Install with:"
    echo "  python3 -m pip install --break-system-packages numpy"
    MISSING_DEPS=true
fi

if ! python3 -c "import matplotlib" 2>/dev/null; then
    echo "ERROR: matplotlib not found. Install with:"
    echo "  python3 -m pip install --break-system-packages matplotlib"
    MISSING_DEPS=true
fi

if ! python3 -c "from transforms3d.euler import quat2euler" 2>/dev/null; then
    echo "ERROR: transforms3d not found. Install with:"
    echo "  python3 -m pip install --break-system-packages transforms3d"
    MISSING_DEPS=true
fi

if [ "$MISSING_DEPS" = true ]; then
    echo ""
    echo "Please install missing dependencies before continuing."
    exit 1
fi

echo "All dependencies found!"

echo ""
echo "================================================"
echo "Select testing mode:"
echo "================================================"
echo "1) Test with MOCK sensors (simulated circular motion)"
echo "2) Test with REAL sensors (requires robot/simulation running)"
echo "3) Just run state estimator (manual sensor setup)"
echo "4) Exit"
echo ""
read -p "Enter choice [1-4]: " choice

case $choice in
    1)
        echo ""
        echo "Starting MOCK sensor test..."
        echo "This will:"
        echo "  1. Start mock sensor publisher (simulates circular motion)"
        echo "  2. Start state estimation filter"
        echo "  3. Launch visualization"
        echo ""
        echo "Press Ctrl+C to stop all processes"
        echo ""
        
        # Start mock sensors in background
        echo "Starting mock sensor publisher..."
        python3 state_estimation/test_mock_sensors.py &
        MOCK_PID=$!
        sleep 2
        
        # Start state estimator in background
        echo "Starting state estimator..."
        python3 state_estimation/sensor_fusion_ekf.py &
        ESTIMATOR_PID=$!
        sleep 2
        
        # Start visualizer in foreground
        echo "Starting visualizer..."
        python3 state_estimation/visualize_state.py
        
        # Cleanup on exit
        echo "Stopping processes..."
        kill $MOCK_PID $ESTIMATOR_PID 2>/dev/null
        ;;
        
    2)
        echo ""
        echo "Starting REAL sensor test..."
        echo "Make sure your robot or simulation is running!"
        echo ""
        read -p "Press Enter to continue or Ctrl+C to cancel..."
        
        # Check if sensor topics exist
        echo "Checking for sensor topics..."
        if ! ros2 topic list | grep -q "imu_0/data"; then
            echo "Warning: IMU topic not found. Is the robot/simulation running?"
        fi
        
        # Start state estimator in background
        echo "Starting state estimator..."
        python3 state_estimation/sensor_fusion_ekf.py &
        ESTIMATOR_PID=$!
        sleep 2
        
        # Start visualizer in foreground
        echo "Starting visualizer..."
        python3 state_estimation/visualize_state.py
        
        # Cleanup
        echo "Stopping state estimator..."
        kill $ESTIMATOR_PID 2>/dev/null
        ;;
        
    3)
        echo ""
        echo "Starting state estimator only..."
        python3 state_estimation/sensor_fusion_ekf.py
        ;;
        
    4)
        echo "Exiting..."
        exit 0
        ;;
        
    *)
        echo "Invalid choice!"
        exit 1
        ;;
esac

echo ""
echo "Done!"
