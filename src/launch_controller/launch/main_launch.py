from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument('stdout_line_buffered', default_value='1'),
        DeclareLaunchArgument('serial', default_value="'17550665'",),
        DeclareLaunchArgument('device_id', default_value='4'),
        DeclareLaunchArgument('device_ip', default_value='192.168.1.201'),
        DeclareLaunchArgument('port', default_value='2370'),

        # Include each sublaunch

        # Include Xsens launch
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
        
        # Include camera launch
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

        # Include thermal camera launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_controller'),  # ← thermal_camera_launch.py があるパッケージ名
                    'launch',                            # ← thermal_camera_launch.py のディレクトリ名
                    'thermal_camera_launch.py'
                ])
            ]),
            launch_arguments={
                'device_id': LaunchConfiguration('device_id')  # ← 渡す
            }.items()
        ),

        # Include Velodyne launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_controller'),  # ← velodyne_launch.py があるパッケージ名
                    'launch',                            # ← velodyne_launch.py のディレクトリ名
                    'velodyne_launch.py'
                ])
            ]),
            launch_arguments={
                'device_ip': LaunchConfiguration('device_ip'),
                'port': LaunchConfiguration('port')
            }.items()
        ),
        
        # Include RTSP image publisher launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_controller'),  # ← lowrance_driver があるパッケージ名
                    'launch',                            # ← lowrance_driver の launch ディレクトリ名
                    'lowrance_rtsp_launch.py'
                ])
            ])
        )

        # Include propeller control launch
        , IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('launch_controller'),  # ← propeller_teleop があるパッケージ名
                    'launch',                            # ← propeller_teleop の launch ディレクトリ名
                    'propeller_controller_launch.py'
                ])
            ])
        )
    ])