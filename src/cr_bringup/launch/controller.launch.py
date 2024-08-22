from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the joy_node from the joy package
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.1,
                'autorepeat_rate': 10.0,
            }]
        ),

        # Launch the joy_to_twist_node.py from the cr_hardware package
        Node(
            package='cr_hardware',
            executable='joy_to_twist_node.py',
            name='joy_to_twist_node',
            output='screen',
            parameters=[{
                # 'some_param_name': 'some_value',  # Example parameter if needed
            }]
        ),
    ])
