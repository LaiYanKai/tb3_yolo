#!/bin/bash
cd $HOME/tb3_yolo
ROS_DOMAIN_ID=$1 ros2 run nav2_map_server map_saver_cli -f $HOME/tb3_yolo_pc/src/tb3_yolo_bringup/maps/map
colcon build --symlink-install