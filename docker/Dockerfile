FROM ros:foxy-ros-base

ARG USE_ROS1
ARG USE_ROS2
ARG USE_GAZEBO
ARG USE_TORCH
ARG USE_CUDA
ARG USE_OPENCV
ARG USE_OAKD
ARG USE_REALSENSE

ENV ROS1_DISTRO noetic
ENV ROS2_DISTRO foxy


# Create User
ENV ROSUSER="user"
RUN adduser --home /home/${ROSUSER} --shell /bin/zsh --gecos '' --disabled-password ${ROSUSER} && \
    usermod -a -G root,sudo,video,tty,dialout ${ROSUSER} && \
    echo "${ROSUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${ROSUSER}

# Ubuntu setup
RUN apt-get update && apt-get install -y --no-install-recommends \
    dialog apt-utils zsh && \
    echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections && \
    apt-get install -y -q && \
    rm -rf /var/lib/apt/lists/*

# CUDA
ENV CUDA_VERSION 11.2.2
ENV CUDNN_VERSION 8.1.1.33
ENV NCCL_VERSION 2.8.4
ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES all
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.2 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 driver>=450,driver<451"
COPY scripts/cuda.sh .
RUN if [ "$USE_CUDA" = "ON" ]; then \
      sh ./cuda.sh \
    ;fi && \
    rm cuda.sh 

# OpenCV

COPY scripts/opencv.sh .
RUN if [ "$USE_OPENCV" = "ON" ]; then \
      sh ./opencv.sh \
    ;fi && \
    rm opencv.sh 

# OAKD
COPY scripts/oakd.sh .
RUN if [ "$USE_OAKD" = "ON" ]; then \
      sh ./oakd.sh \
    ;fi && \
    rm oakd.sh 


# REALSENSE
COPY scripts/realsense.sh .
RUN if [ "$USE_REALSENSE" = "ON" ]; then \
    sh ./realsense.sh \
  ;fi && \
  rm realsense.sh 

# ROS 2
RUN if [ "$USE_ROS2" = "ON" ]; then \
    apt-get update && \
    apt-get install -y \
    apt-utils debconf-utils gnupg2 lsb-release && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654  && \
    echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list && \
    rosdep update && \
    apt-get update && \
    RTI_NC_LICENSE_ACCEPTED=yes apt-get install rti-connext-dds-5.3.1 \
  ;fi

# ROS 1
RUN if [ "$USE_ROS1" = "ON" ]; then \
    echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1-latest.list \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 \
  ;fi

# Gazebo
COPY scripts/gazebo.sh .
RUN if [ "$USE_GAZEBO" = "ON" ]; then \
    sh ./gazebo.sh \
  ;fi && \
  rm gazebo.sh 


# Python
RUN apt-get update && \
    apt-get install -y \
       python3-tk \
       python3-pip && \
       rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    if [ "$USE_TORCH" = "ON" ]; then \
      pip3 install torch \
    ;fi

RUN pip3 install --upgrade pip && \
    if [ "$USE_CUDA" = "ON" ]; then \
      pip3 install onnxruntime-gpu \
    else \
      pip3 install onnxruntime \
    ;fi

# Python Packages
RUN pip3 install --upgrade pip && \
    pip3 install \
        rospkg \
        numpy \
        pandas \
        numba \
        matplotlib \
        wandb \
        seaborn \
        tqdm \
        yaml-1.3 \
        nnio \
        pynput \
        uvicorn \
        vidgear \
        starlette \
        aiofiles \
        aiohttp \
        aiortc \
        scikit-build \
        cmake \
        netifaces \
        opencv-python \
        opencv-contrib-python \
        msgpack_numpy \
        jinja2 \
        conan \
        filterpy


# ROS2 Packages dependencies
RUN if [ "$USE_ROS2" = "ON" ]; then \
    apt-get update && \
    apt-get install -y \
    qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev libpcl-dev \
  ;fi

# ROS 2 Packages
RUN if [ "$USE_ROS2" = "ON" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-rviz2 \
    ros-foxy-rqt \
    && rm -rf /var/lib/apt/lists/* \
  ;fi

RUN if [ "$USE_ROS2" = "ON" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-foxy-ros-core \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-geometry2 \
    ros-foxy-filters \
    ros-foxy-octomap \
    ros-foxy-octomap-msgs \
    ros-foxy-perception-pcl \
    && rm -rf /var/lib/apt/lists/* \
  ;fi

# ROS 1 Packages
RUN if [ "$USE_ROS1" = "ON" ]; then \
    apt-get update && apt-get install -y --no-install-recommends \
    ros-noetic-ros-comm \
    ros-noetic-grid-map \
    ros-noetic-trajectory-msgs \
    ros-noetic-tf-conversions \
    ros-noetic-pcl-ros \
    ros-noetic-eigen-conversions \
    ros-noetic-octomap \
    ros-noetic-octomap-msgs \
    && rm -rf /var/lib/apt/lists/* \
  ;fi 

# Utils 
RUN apt-get update && \
    apt-get install -y \
        tmux \
        ranger \
        wget \
        neovim \
        nano \
        htop \
        iputils-ping && \
        rm -rf /var/lib/apt/lists/*

# Scripts
user $ROSUSER

RUN mkdir -p /home/$ROSUSER/scripts
RUN mkdir -p ~/ros2_ws/src
RUN mkdir -p ~/ros1_ws/src

WORKDIR /home/$ROSUSER/scripts

COPY scripts/utils-setup.sh .
RUN ./utils-setup.sh

COPY scripts/ros-setup.sh .
RUN ./ros-setup.sh

WORKDIR /home/$ROSUSER
RUN rm -rf scripts

ENTRYPOINT []
