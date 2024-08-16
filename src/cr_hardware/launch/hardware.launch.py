from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    cr_hardware_dir = get_package_share_directory('cr_hardware')

    # Include the bno055 launch file
    bno055_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cr_hardware_dir, 'launch', 'bno055.launch.py'))
    )

    # Include the differential_drive launch file
    differential_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(cr_hardware_dir, 'launch', 'differential_drive.launch.py'))
    )

    return LaunchDescription([
        bno055_launch,
        differential_drive_launch
    ])
