# src/launch_controller/velodyne_launch.py

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1) launch 引数を宣言
    args = [
        DeclareLaunchArgument('device_ip', default_value='192.168.100.201'),
        DeclareLaunchArgument('gps_time', default_value='false'),
        DeclareLaunchArgument('time_offset', default_value='0.0'),
        DeclareLaunchArgument('enabled', default_value='true'),
        DeclareLaunchArgument('read_once', default_value='false'),
        DeclareLaunchArgument('read_fast', default_value='false'),
        DeclareLaunchArgument('repeat_delay', default_value='0.0'),
        DeclareLaunchArgument('frame_id', default_value='velodyne'),
        # DeclareLaunchArgument('model', default_value='32C'),
        DeclareLaunchArgument('model', default_value='VLP16'),
        DeclareLaunchArgument('rpm', default_value='600.0'),
        # DeclareLaunchArgument('port', default_value='2368'),
        DeclareLaunchArgument('port', default_value='2370'),
        DeclareLaunchArgument('timestamp_first_packet', default_value='false'),
    ]

    # 2) 実際にノードを組み立てる関数
    def launch_setup(context, *args, **kwargs):
        # --- A) driver ノード用パラメータ YAML 読み込み + 上書き ---
        pkg_share = get_package_share_directory('velodyne_driver')
        # yaml_path = os.path.join(pkg_share, 'config', 'VLP32C-velodyne_driver_node-params.yaml')
        yaml_path = os.path.join(pkg_share, 'config', 'VLP16-velodyne_driver_node-params.yaml')
        with open(yaml_path, 'r') as f:
            all_params = yaml.safe_load(f)

        # 上書きしたいフィールドだけ取り出し
        p = all_params['velodyne_driver_node']['ros__parameters']
        p['device_ip']            = LaunchConfiguration('device_ip').perform(context)
        p['gps_time']             = context.launch_configurations['gps_time'] == 'true'
        p['time_offset']          = float(context.launch_configurations['time_offset'])
        p['enabled']              = context.launch_configurations['enabled'] == 'true'
        p['read_once']            = context.launch_configurations['read_once'] == 'true'
        p['read_fast']            = context.launch_configurations['read_fast'] == 'true'
        p['repeat_delay']         = float(context.launch_configurations['repeat_delay'])
        p['frame_id']             = LaunchConfiguration('frame_id').perform(context)
        p['model']                = LaunchConfiguration('model').perform(context)
        p['rpm']                  = float(LaunchConfiguration('rpm').perform(context))
        p['port']                 = int(LaunchConfiguration('port').perform(context))
        p['timestamp_first_packet']= context.launch_configurations['timestamp_first_packet']=='true'

        # driver ノード
        driver_node = Node(
            package='velodyne_driver',
            executable='velodyne_driver_node',
            name='velodyne_driver_node',
            output='screen',
            parameters=[p],
        )

        # --- B) transform ノード ---
        # calib = os.path.join(
        #     get_package_share_directory('velodyne_pointcloud'),
        #     'params',
        #     'VeloView-VLP-32C.yaml'
        # )
        calib = os.path.join(
            get_package_share_directory('velodyne_pointcloud'),
            'params',
            'VLP16db.yaml'
        )
        transform_node = Node(
            package='velodyne_pointcloud',
            executable='velodyne_transform_node',
            name='velodyne_transform_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id').perform(context),
                'model':    LaunchConfiguration('model').perform(context),
                'rpm':      float(LaunchConfiguration('rpm').perform(context)),
                'calibration': calib,
            }],
            remappings=[
                ('velodyne_packets', 'velodyne_packets'),
                ('velodyne_points',  'velodyne_points'),
            ]
        )

        # --- C) laserscan ノード ---
        laserscan_node = Node(
            package='velodyne_laserscan',
            executable='velodyne_laserscan_node',
            name='velodyne_laserscan_node',
            output='screen',
            parameters=[{
                'frame_id': LaunchConfiguration('frame_id').perform(context),
            }],
            remappings=[
                ('velodyne_points', 'velodyne_points'),
                ('scan',            'scan'),
            ]
        )

        return [driver_node, transform_node, laserscan_node]

    # 3) OpaqueFunction で実行時に launch_setup を呼び出す
    return LaunchDescription(args + [
        OpaqueFunction(function=launch_setup)
    ])
