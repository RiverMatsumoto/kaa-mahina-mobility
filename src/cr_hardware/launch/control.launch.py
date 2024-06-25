from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[{'joy_config': 'xbox'}],  # Ensure this path is correct
            output='screen',
            remappings=[
                ('/cmd_vel', '/cmd_vel')  # Change this as needed
            ]
        )
    ])