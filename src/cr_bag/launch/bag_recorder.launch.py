from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    trial_recorder_node = Node(
        package='cr_bag',
        executable='record_trial.py',
        name='record_trial',
        output='screen'
    )

    trial_menu_node = Node(
        package='cr_bag',
        executable='trial_menu_and_runner.py',
        name='bag_recorder',
        output='screen'
    )

    trial_executor_node = Node(
        package='cr_bag',
        executable='trial_executor.py',
        name='trial_executor',
        output='screen'
    )

    return LaunchDescription([
        trial_recorder_node,
        trial_executor_node,
        trial_menu_node
    ])
    