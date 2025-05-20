from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        # propeller_driver パッケージの launch ファイルを Include
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('propeller_teleop'),
                    'launch',
                    'propeller_teleop_launch.py'
                ])
            )
        ),
    ])
