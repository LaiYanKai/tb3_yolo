#!/bin/bash
cd $HOME/tb3_yolo
TURTLEBOT3_MODEL=burger ROS_DOMAIN_ID=$1 ros2 run turtlebot3_teleop teleop_keyboard