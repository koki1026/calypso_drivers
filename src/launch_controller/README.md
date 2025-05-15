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
