#!/bin/bash

sudo apt-get update      
sudo apt-get install -y ros-${ROS2_DISTRO}-realsense2-camera ros-${ROS2_DISTRO}-librealsense2

rm -rf /var/lib/apt/lists/*
