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

## 