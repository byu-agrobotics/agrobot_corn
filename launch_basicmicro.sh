#!/bin/bash

# Basicmicro ROS2 Driver Launch Script (Development)
# This script sets up the development environment and launches the driver

# Configuration
WORKSPACE_PATH="$HOME/ros2_ws"
USE_VIRTUAL_ENV=true
DEFAULT_PORT="/dev/ttyAMA0"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# Check for port argument
if [ "$#" -gt 0 ]; then
    PORT="$1"
else
    PORT="$DEFAULT_PORT"
fi

echo -e "${GREEN}[INFO]${NC} Launching Basicmicro ROS2 driver on port: $PORT"

# Setup environment
cd "$WORKSPACE_PATH"

# Source ROS2 environment
if [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]; then
    source "/opt/ros/$ROS_DISTRO/setup.bash"
elif [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source "/opt/ros/jazzy/setup.bash"
else
    echo -e "${RED}[ERROR]${NC} ROS2 not found"
    exit 1
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo -e "${RED}[ERROR]${NC} Workspace not built. Please run: colcon build --packages-select basicmicro_driver"
    exit 1
fi

# Activate virtual environment if used
if [ "$USE_VIRTUAL_ENV" = true ] && [ -d "venv" ]; then
    source venv/bin/activate
    echo -e "${GREEN}[INFO]${NC} Virtual environment activated"
fi

# Launch the driver
echo -e "${GREEN}[INFO]${NC} Starting Basicmicro ROS2 driver..."
ros2 run basicmicro_ros2 basicmicro_node.py --ros-args -p port:=$PORT




