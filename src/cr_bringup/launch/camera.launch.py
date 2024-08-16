from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node_cam1',
            namespace='rpi1',
            parameters=[{
                'video_device': '/dev/video0',
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'image_size': [640, 480],
                'time_per_frame': [1, 30],
                'use_v4l2_buffer_timestamps': True,
                'timestamp_offset': 0,
                # Add more camera control parameters as needed
                # 'brightness': 128,
                # 'contrast': 32,
                # etc.
            }],
            output='screen',
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
