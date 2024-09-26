#!/bin/bash

# Build the workspace
colcon build --packages-select vi_agent

# Source the setup file
source install/setup.bash

# Run the ROS2 node
ros2 run vi_agent vi_agent
