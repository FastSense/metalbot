#!/bin/bash

echo "Image: " $image
echo "Container: " $container

echo "USE_X_SERVER: " $USE_X_SERVER

echo "USE_ROS1: " $USE_ROS1
echo "USE_ROS2: " $USE_ROS2
echo "USE_GAZEBO: " $USE_GAZEBO
echo "USE_TORCH: " $USE_TORCH
echo "USE_CUDA: " $USE_CUDA
echo "USE_OPENCV: " $USE_OPENCV
echo "USE_REALSENSE: " $USE_REALSENSE
echo "USE_OAKD: " $USE_OAKD
echo "
" 


build_docker()  {
    echo "Building ..."
    docker build . -t $image \
      --build-arg USE_ROS1=$USE_ROS1 \
      --build-arg USE_ROS2=$USE_ROS2 \
      --build-arg USE_GAZEBO=$USE_GAZEBO \
      --build-arg USE_TORCH=$USE_TORCH \
      --build-arg USE_CUDA=$USE_CUDA \
      --build-arg USE_OPENCV=$USE_OPENCV \
      --build-arg USE_REALSENSE=$USE_REALSENSE \
      --build-arg USE_OAKD=$USE_OAKD 
}

run_docker()  {
    cuda_args=()
    volumes_args=()
    environment_args=()

    volumes_args+=( "-v /dev/bus/usb:/dev/bus/usb " )
    environment_args+=( "-v /dev/bus/usb:/dev/bus/usb " )

    if [[ "$USE_CUDA" == "ON" ]]; then
        cuda_args+=( "--gpus=all" )
        cuda_args+=( "--runtime=nvidia" )
    fi

    if [[ "$USE_X_SERVER" == "ON" ]]; then
        volumes_args+=( "-v /tmp/.X11-unix:/tmp/.X11-unix:ro" )
        environment_args+=( "-e DISPLAY=$DISPLAY" )
        environment_args+=( "-e QT_X11_NO_MITSHM=1" )
    fi

    if [[ "$USE_ROS1" == "ON" ]]; then
        local path="/home/user/ros1_ws/src" 
        local master_uri="localhost" 
        volumes_args+=( "-v $(pwd)/../ros1_src:${path}:rw" )
        environment_args+=( "-e ROS_MASTER_URI=http://${master_uri}:11311 " )
    fi

    if [[ "$USE_ROS2" == "ON" ]]; then
        local path="/home/user/ros2_ws/src" 
        environment_args+=( "-e ROS_DOMAIN_ID=0 " )
        volumes_args+=( "-v $(pwd)/../ros2_src/:${path}:rw" )
    fi

    docker run -it --privileged --net=host \
        --name $container \
        --device-cgroup-rule='c 189:* rmw' \
        ${cuda_args[@]} \
        ${environment_args[@]} \
        ${volumes_args[@]} \
        $image zsh
}

case $1 in
  build)
    build_docker
  ;;
  run)
     run_docker 
  ;;
  push)
    echo "Pushing ..."
    docker push $image 
  ;;
  pull)
    echo "Pulling ..."
    docker pull $image 
  ;;
  *)
    echo "drun doesn't know such command :(" "
    " $1 
    echo "Try [build, run, push, pull] and don't forget to set environment variables " ;;
esac
