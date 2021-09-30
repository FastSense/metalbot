#!/bin/bash

echo "Image: " $image
echo "Container: " $container

echo "Gazebo: " $USE_GAZEBO
echo "Realsense: " $USE_REALSENSE
echo "OpenCV: " $USE_OPENCV
echo "OAKD: " $USE_OAKD
echo "CUDA: " $USE_CUDA
echo "USE_X_SERVER" $USE_X_SERVER

echo "
" 


build_docker()  {
    echo "Building ..."
    docker build . -t $image \
      --build-arg USE_GAZEBO=$USE_GAZEBO \
      --build-arg USE_REALSENSE=$USE_REALSENSE \
      --build-arg USE_OAKD=$USE_OAKD \
      --build-arg USE_CUDA=$USE_CUDA \
      --build-arg USE_OPENCV=$USE_OPENCV 
}

run_docker()  {
    cuda_args=()
    volumes_args=()
    environment_args=()

    volumes_args+=( "-v /dev/bus/usb:/dev/bus/usb " )
    environment_args+=( "-v /dev/bus/usb:/dev/bus/usb " )


    case $USE_CUDA in
      ON)
        cuda_args+=( "--gpus=all" )
        cuda_args+=( "--runtime=nvidia" )
        ;;
      esac 

    case $USE_ROS1 in
      ON)
        volumes_args+=( "-v $(pwd)/../ros1_ws:/home/user/ros1_ws/src:rw" )
        environment_args+=( "-e ROS_MASTER_URI=http://localhost:11311 " )
        ;;
      esac 

    case $USE_ROS2 in
      ON)
        volumes_args+=( "-v $(pwd)/../ros2_ws:/home/user/ros2_ws/src:rw" )
        environment_args+=( "-e ROS_DOMAIN_ID=0 " )
        ;;
      esac 

    case $USE_X_SERVER in
      ON)
        volumes_args+=( "-v /tmp/.X11-unix:/tmp/.X11-unix:ro" )
        environment_args+=( "-e DISPLAY=$DISPLAY" )
        environment_args+=( "-e QT_X11_NO_MITSHM=1" )
        ;;
      esac 

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
  push)
    echo "Pushing ..."
    docker push $image 
    ;;
  pull)
    echo "Pulling ..."
    docker pull $image 
    ;;
  run)
     run_docker ;;
  *)
    echo "drun doesn't know such command :(" "
    " $1 
    echo "Try [build, run, push, pull] and don't forget to set environment variables " ;;
esac


