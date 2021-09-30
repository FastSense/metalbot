#!/bin/bash


sudo apt-get update      
sudo apt-get install ros-${ROS2_DISTRO}-realsense2-camera

rm -rf /var/lib/apt/lists/*
