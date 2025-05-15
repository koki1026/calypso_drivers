from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path

def generate_launch_description():
    # Declare launch argument for environment variable
    return LaunchDescription([
        DeclareLaunchArgument(
            'stdout_line_buffered',
            default_value='1',
            description='Set RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED env var'
        ),

        # Set environment variable dynamically
        SetEnvironmentVariable(
            'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED',
            LaunchConfiguration('stdout_line_buffered')
        ),

        # Include the original driver node
        Node(
            package='bluespace_ai_xsens_mti_driver',
            executable='xsens_mti_node',
            name='xsens_mti_node',
            output='screen',
            parameters=[Path(
                get_package_share_directory('bluespace_ai_xsens_mti_driver')) / 'param' / 'xsens_mti_node.yaml'],
                remappings=[
                    ('/gnss', '/gps/fix')  #change topic name
                ]
        )
    ])