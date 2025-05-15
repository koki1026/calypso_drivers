from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'device_id',
            default_value='4',
            description='Device ID for the thermal camera'
        ),

        Node(
            package='thermal_camera_driver',
            executable='thermal_camera_publisher',
            name='thermal_camera_publisher',
            parameters=[{
                'device_id': LaunchConfiguration('device_id')
            }],
            output='screen'
        )
    ])
