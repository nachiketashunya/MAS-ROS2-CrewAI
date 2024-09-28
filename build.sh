# Build the workspace
colcon build --packages-select ci_agent_package

# Build the workspace
colcon build --packages-select bi_agent_package

# Build the workspace
colcon build --packages-select visitor_agent_package

# Build the workspace
colcon build --packages-select multi_agent_launch_package