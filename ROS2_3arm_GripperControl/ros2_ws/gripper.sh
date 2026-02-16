#!/bin/bash

# Set script to exit if any command fails
set -e

#Go to workspace root
cd /home/ros2_ws

# Clean previous build, install, and log directories
echo "Cleaning previous build, install, and log directories..."
rm -rf build/ install/ log/
find . -name '*.egg-info' -exec rm -rf {} +

echo " Building workspace..."
colcon build --symlink-install

if [ $? -ne 0 ]; then
    echo " Build failed. Aborting."
  exit 1
fi

# Source ROS 2 and workspace setup
echo "Sourcing setup.bash..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Launching wsg50_gripper control driver..."
ros2 launch wsg50_driver_pkg gripper.launch.py