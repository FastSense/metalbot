#!/bin/bash

echo $image
echo $container

case $1 in
  push)
    docker push $image ;;
  pull)
    docker pull $image ;;
  gpu)
    docker run -it --privileged --net=host \
        --name $container \
        --runtime=nvidia \
        -v /dev/bus/usb:/dev/bus/usb \
        --device-cgroup-rule='c 189:* rmw' \
        --gpus=all \
        -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
        -v $(pwd)/../ros2_ws:/home/user/ros2_ws/src:rw \
        -v $(pwd)/../ros1_ws:/home/user/ros1_ws/src:rw \
        -v /home/${USER}/data:/home/user/data:ro \
        -e ROS_MASTER_URI=http://localhost:11311 \
        -e DISPLAY=$DISPLAY \
        -e NVIDIA_VISIBLE_DEVICES="all" \
        -e NVIDIA_DRIVER_CAPABILITIES="all" \
        -e ROS_DOMAIN_ID=0 \
        -e QT_X11_NO_MITSHM=1 $image zsh ;;
  *)
  docker run -it --privileged --net=host \
      -v /dev/bus/usb:/dev/bus/usb \
      --device-cgroup-rule='c 189:* rmw' \
      --name $container \
      -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
      -v $(pwd)/../ros2_ws:/home/user/ros2_ws/src:rw \
      -v $(pwd)/../ros1_ws:/home/user/ros1_ws/src:rw \
      -v /home/${USER}/data:/home/user/data:ro \
      -e ROS_MASTER_URI=http://localhost:11311 \
      -e DISPLAY=$DISPLAY \
      -e ROS_DOMAIN_ID=0 \
      -e QT_X11_NO_MITSHM=1 $image zsh ;;
esac
