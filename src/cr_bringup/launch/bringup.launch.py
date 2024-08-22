from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    vicon_client_launch_dir = PathJoinSubstitution([FindPackageShare('vicon_client'), 'launch'])

    vicon_client_launch = IncludeLaunchDescription(
        PathJoinSubstitution([vicon_client_launch_dir, 
                              'client.launch.py']))

    return LaunchDescription([
        vicon_client_launch
    ])
