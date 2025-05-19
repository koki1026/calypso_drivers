# launch_controller

## xsens_launch
### base node
bluespace_ai_xsens_mti_driver -> xsens_mti_node
### param
```
DeclareLaunchArgument('stdout_line_buffered', default_value='1')
```
### change
```
remappings=[
                    ('/gnss', '/gps/fix')  #change topic name
                ]
```

## camera_launch
### base launch
spinnaker_camera_driver -> driver_node.launch.py
### param
```
DeclareLaunchArgument('serial', default_value="'17550665'")
```

### add
```
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
```

## thermal_camera_launch
### base node
thermal_camera_driver -> thermal_camera_publisher
### param
DeclareLaunchArgument( 'device_id', default_value='4'),

## velodyne_launch
### base launch
velodyne_driver -> velodyne_driver_node
### add
```
        # --- B) transform ノード ---
        calib = os.path.join(
            get_package_share_directory('velodyne_pointcloud'),
            'params',
            'VeloView-VLP-32C.yaml'
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
```
### param
DeclareLaunchArgument('device_ip', default_value='192.168.201.1')
DeclareLaunchArgument( 'port', default_value='2370')
