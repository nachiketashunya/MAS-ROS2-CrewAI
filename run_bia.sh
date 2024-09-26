#!/bin/bash

# Build the workspace
colcon build --packages-select bi_agent_package

# Source the setup file
source install/setup.bash

# Run the ROS2 node
ros2 run bi_agent_package bi_agent_node
