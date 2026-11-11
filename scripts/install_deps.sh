#!/usr/bin/env bash
set -e
sudo apt update
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-xacro \
  ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
  ros-humble-robot-localization ros-humble-twist-mux ros-humble-cv-bridge \
  ros-humble-teleop-twist-keyboard python3-colcon-common-extensions
/usr/bin/python3 -m pip install --user "numpy==1.26.4" "opencv-python==4.10.0.84" ultralytics websockets aiohttp psutil
