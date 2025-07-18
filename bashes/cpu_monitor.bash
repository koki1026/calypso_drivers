#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/calypso/calypso_drivers/install/setup.bash
ros2 launch cpu_monitor monitor.launch.py
