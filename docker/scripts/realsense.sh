#!/bin/bash

if ! [[ "$USE_REALSENSE" == "ON" ]]; then
  exit 0;
fi

sudo su $ROSUSER

sudo apt-get update      
sudo apt-get install ros-${ROS2_DISTRO}-realsense2-camera

mkdir -p /home/${ROSUSER}/ros2_ws/src/sensors
cd /home/${ROSUSER}/ros2_ws/src/sensors
git clone --depth 1 --branch `git ls-remote --tags https://github.com/IntelRealSense/realsense-ros.git | grep -Po "(?<=tags/)3.\d+\.\d+" | sort -V | tail -1` https://github.com/IntelRealSense/realsense-ros.git
cd /

sudo su root
rm -rf /var/lib/apt/lists/*

