#!/bin/bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}}"