```bash
sudo apt install ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz ros-jazzy-turtlebot3-gazebo ros-jazzy-turtlebot3-teleop ros-jazzy-turtlebot3-cartographer ros-jazzy-nav2-map-server ros-jazzy-turtlebot3-navigation2 ros-jazzy-nav2-route -y
cd $HOME
git clone https://github.com/laiyankai/tb3_yolo
cd $HOME/tb3_yolo
colcon build --symlink-install
chmod +x *.sh
echo $ROS_DOMAIN_ID
```

```bash
ssh ece@10.42.0.
```

```bash
rm -rf $HOME/tb3_yolo
wget https://raw.githubusercontent.com/LaiYanKai/Misc/main/tb3_jazzy_c1_cam3/flash.sh
wget https://raw.githubusercontent.com/LaiYanKai/Misc/main/tb3_jazzy_c1_cam3/robot.sh
chmod +x *.sh
echo $ROS_DOMAIN_ID
```