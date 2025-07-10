from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace

def generate_launch_description():
    camera_name = LaunchConfiguration('camera_name')
    return LaunchDescription([
        # Declare arguments that can be passed from main_launch.py
        DeclareLaunchArgument(
            'camera_name',
            default_value='flir_camera',
            description='ROS node name for the camera'
        ),
        DeclareLaunchArgument(
            'camera_type',
            default_value='grasshopper',
            description='Type of camera (e.g., blackfly_s, chameleon, grasshopper, etc.)'
        ),
        DeclareLaunchArgument(
            'serial',
            default_value="'17550665'",
            description='Serial number of the FLIR camera (in quotes!)'
        ),
        DeclareLaunchArgument(
            'parameter_file',
            default_value='',
            description='Optional: path to parameter file. Overrides camera type defaults.'
        ),

        # Include the upstream driver_node.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('spinnaker_camera_driver'),
                    'launch',
                    'driver_node.launch.py'
                ])
            ),
            launch_arguments={
                'camera_name': LaunchConfiguration('camera_name'),
                'camera_type': LaunchConfiguration('camera_type'),
                'serial': LaunchConfiguration('serial'),
                'parameter_file': LaunchConfiguration('parameter_file'),
            }.items()
        ),
         # image_proc ノードのみを namespace 付きで追加
        GroupAction([
            PushRosNamespace(camera_name),
            Node(
                package='image_proc',
                executable='image_proc',
                name='image_proc',
                output='screen',
                remappings=[
                    ('image', 'image_raw'),
                    ('camera_info', 'camera_info')
                ]
            )
        ])
    ])