from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trial_recorder_node = Node(
        package='cr_bag',
        executable='record_trial.py',
        name='record_trial',
        output='screen'
    )

    return LaunchDescription([
        trial_recorder_node
    ])
    