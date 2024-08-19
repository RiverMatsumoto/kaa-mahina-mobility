from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    ekf_local_config_path = PathJoinSubstitution([FindPackageShare('cr_localization'),
                                            'config', 
                                            'ekf_local.yaml'])
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_localization_node',
        output='screen',
        parameters=[{'use_sim_time': False}, ekf_config_path]
    )
    
    return LaunchDescription([
        ekf_node
    ])