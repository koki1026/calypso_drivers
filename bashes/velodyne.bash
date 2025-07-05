#!/bin/bash
source /home/calypso/calypso/calypso_drivers/install/setup.bash
ros2 launch launch_controller velodyne_launch.py port:=2370
