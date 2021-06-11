#!/bin/bash

image_name=rosbot2-image
container_name=rosbot2

docker run -it -d --privileged --net=host \
    --name $container_name \
    --runtime=nvidia \
    -v /dev/bus/usb:/dev/bus/usb \
    --device-cgroup-rule='c 189:* rmw' \
    --gpus=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $(pwd)/..:/home/user/ros2_ws/src:rw \
    -v /home/${USER}/data:/home/user/data:ro \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_VISIBLE_DEVICES="all" \
    -e NVIDIA_DRIVER_CAPABILITIES="all" \
    -e ROS_HOSTNAME="localhost" \
    -e ROS_MASTER_URI="http://localhost:11311" \
    -e QT_X11_NO_MITSHM=1 $image_name zsh
