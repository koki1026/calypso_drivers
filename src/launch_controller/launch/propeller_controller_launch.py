from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='propeller_controller',
            executable='propeller_controller_node',
            name='propeller_controller_node',
            output='screen'
        )
    ])
