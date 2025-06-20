#!/bin/bash
source /home/calypso/calypso/calypso_drivers/install/setup.bash
ros2 launch launch_controller main_launch.py device_id:=0
