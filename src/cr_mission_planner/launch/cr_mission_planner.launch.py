from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [Node(package="cr_mission_planner", executable="cr_mission_planner_node", output="screen")]
    )
