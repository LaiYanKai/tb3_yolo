#!/bin/bash
cd $HOME/tb3_yolo
source install/setup.bash 
ROS_DOMAIN_ID=$1 ros2 run tb3_yolo_camera tb3_yolo_viewer --ros-args -p topic:=/camera/image_raw/compressed -p window:=view_cam