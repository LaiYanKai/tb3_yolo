#!/bin/bash
cd $HOME/tb3_yolo
source install/setup.bash
ROS_DOMAIN_ID=$1 ros2 launch tb3_yolo_bringup run.launch.py
source stop_robot.sh