#!/usr/bin/env bash

# Update application icon
mv serp.desktop serp.desktop-bak
sed -e "s,Icon=.*,Icon=$PWD/icon.png,g" serp.desktop-bak > serp.desktop
rm serp.desktop-bak
export SERP_PROJECT_PATH=~/catkin_ws/src/serp/
. ~/catkin_ws/devel/setup.bash
roscore &
rosrun serp graphical_interface &
rosrun serp rpi_camera_node & 
rosrun serp arduino_bridge_node
