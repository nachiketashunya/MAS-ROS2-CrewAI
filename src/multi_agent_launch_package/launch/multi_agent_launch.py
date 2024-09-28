from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

def generate_launch_description():
    return LaunchDescription([
        # Launch CI Agent Node
        Node(
            package='ci_agent_package',  # Replace with your package name
            executable='ci_agent_node',  # Replace with the entry point of the CI node
            name='ci_agent_node',
            output='screen',
            emulate_tty=True
        ),
        
        # Launch BI Agent Node
        Node(
            package='bi_agent_package',  # Replace with your package name
            executable='bi_agent_node',  # Replace with the entry point of the BI node
            name='bi_agent_node',
            output='screen',
            emulate_tty=True
        ),
        
        # Launch VI Agent Node with delay
        TimerAction(
            period=30.0,  # Delay in seconds before launching VI agent
            actions=[
                Node(
                    package='visitor_agent_package',  # Replace with your package name
                    executable='vi_agent_node',  # Replace with the entry point of the VI node
                    name='vi_agent_node',
                    output='screen',
                    emulate_tty=True
                )
            ]
        )
    ])