from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments (必要に応じて追加)
        DeclareLaunchArgument('stdout_line_buffered', default_value='1'),
        DeclareLaunchArgument('serial', default_value="'17550665'",),
        
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('launch_controller'), 'launch', 'camera_launch.py'])
            ]),
            launch_arguments={
                'camera_name': 'flir_camera',
                'camera_type': 'blackfly_s',
                'serial': "'17550665'", # シリアル番号は適宜変更
                'parameter_file': ''
            }.items()
        ),
        # 他のlaunchも同様に追加
    ])