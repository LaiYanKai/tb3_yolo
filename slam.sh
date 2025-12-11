#!/bin/bash
cd $HOME/tb3_yolo
source install/setup.bash
export TURTLEBOT3_MODEL=burger 
ROS_DOMAIN_ID=$1 ros2 launch tb3_yolo_bringup slam.launch.py resolution:=0.02