#!/bin/bash
# ==============================================================================
# R1 Robot Startup Script for Raspberry Pi
# ==============================================================================

# 1. Dynamically source ROS 2 setup environment (e.g., humble, jazzy, iron)
for ros_setup in /opt/ros/jazzy/setup.bash; do
    if [ -f "$ros_setup" ]; then
        source "$ros_setup"
        break
    fi
done

# 2. Source the built local R1 workspace overlay
WORKSPACE_DIR="/home/rpi/ros2_ws/"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
else
    echo "Error: $WORKSPACE_DIR/install/setup.bash not found. Please run colcon build first."
    exit 1
fi

# 3. Wait slightly for serial/USB hardware devices to settle on boot
sleep 3

# 4. Launch the bringup package
echo "Starting R1 Joy Serial Bringup..."
ros2 launch joy_serial_esp joy_serial_bringup.launch.py
