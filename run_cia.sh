#!/bin/bash

# Build the workspace
colcon build --packages-select ci_agent_package

# Source the setup file
source install/setup.bash

# Run the ROS2 node
ros2 run ci_agent_package ci_agent_node
