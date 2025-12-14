#!/bin/bash
cd $HOME/tb3_yolo
source install/setup.bash
ROS_DOMAIN_ID=$1 ros2 run tb3_yolo_camera tb3_yolo_capture $2 $3