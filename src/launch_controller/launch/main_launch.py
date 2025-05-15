from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments (必要に応じて追加)
        DeclareLaunchArgument('stdout_line_buffered', default_value='1'),
        
        # Include each sublaunch

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_controller'),
                    'launch',
                    'xsens_launch.py'
                ])
            ]),
            launch_arguments={
                'stdout_line_buffered': LaunchConfiguration('stdout_line_buffered')
            }.items()
        ),
        # 他のlaunchも同様に追加
    ])