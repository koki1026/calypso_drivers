# src/launch_controller/launch/lowrance_rtsp_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lowrance_driver',
            executable='rtsp_image_publisher',
            name='rtsp_image_publisher',
            output='screen'
        )
    ])