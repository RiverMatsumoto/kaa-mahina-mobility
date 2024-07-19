from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='cr_hardware',
            executable='joy_to_twist_node.py',
            name='joy_to_twist_node',
            output='screen',
            remappings=[
                ('/cmd_vel', '/differential_drive_controller/cmd_vel')  # Change this as needed
            ]
        )
    ])