from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='ID of the Camera Device (e.g., 0 for /dev/video0)'
        ),

        Node(
            package='lowrance_publisher_pkg',
            executable='lowrance_publisher',
            name='lowrance_publisher',
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),}]
        )
    ])
