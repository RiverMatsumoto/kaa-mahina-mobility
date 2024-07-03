from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    cr_hardware_launch_dir = PathJoinSubstitution([FindPackageShare('cr_hardware'), 'launch'])

    diff_drive_launch = IncludeLaunchDescription(
        PathJoinSubstitution([cr_hardware_launch_dir, 
                              'differential_drive.launch.py']))

    imu_launch = IncludeLaunchDescription(
        PathJoinSubstitution([cr_hardware_launch_dir, 
                              'bno055.launch.py']))

    return LaunchDescription([
        diff_drive_launch,
        imu_launch
    ])
