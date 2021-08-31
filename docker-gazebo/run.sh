#!/bin/bash

image_name=rosbot2-image:latest
container_name=rosbot2

docker run -it --privileged --net=host \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --name $container_name \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $(pwd)/..:/home/user/ros2_ws/src:rw \
    -v /home/${USER}/data:/home/user/data:ro \
    -e DISPLAY=$DISPLAY \
    -e ROS_HOSTNAME="localhost" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e QT_X11_NO_MITSHM=1 $image_name zsh
