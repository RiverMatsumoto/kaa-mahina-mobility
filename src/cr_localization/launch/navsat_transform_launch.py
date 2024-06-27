from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/ekf_launch.py'])
    )

    navsat_transform_yaml_path = PathJoinSubstitution([FindPackageShare('cr_localization'),
                                                    'config',
                                                    'navsat_transform_config.yaml'])

    navsat_transform_node = Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            output='screen',
            parameters=[navsat_transform_yaml_path],
            remappings=[
                ('/gps/fix', '/gps/fix'),  # Adjust these topics as necessary
                ('/imu', '/imu'),
                ('/odometry/filtered', '/odometry/filtered')
            ]
        )
    
    return LaunchDescription([
        ekf_launch,
        navsat_transform_node
    ])
