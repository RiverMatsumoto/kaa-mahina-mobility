
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():
    bno055_calibration_file = PathJoinSubstitution([FindPackageShare('cr_hardware'), 'config', 'bno055_calibration.yaml'])

    bno055_calib_launch_arg = DeclareLaunchArgument(
        'bno055_calibration_file',
        default_value=bno055_calibration_file,
        description='Path to bno055 calibration file'
    )

    bno055_node = Node(
        package='cr_hardware',
        executable='bno055_driver_node',
        name='bno055_driver_node',
        output='screen',
        parameters=[LaunchConfiguration('bno055_calibration_file')]
    )

    launch_description = LaunchDescription([
        bno055_calib_launch_arg,
        bno055_node
    ])
    return launch_description
    