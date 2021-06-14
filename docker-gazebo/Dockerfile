FROM osrf/ros:foxy-desktop

RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

##################################################################
# ------------------------------ CUDA -------------------------- #
##################################################################

#############
# CUDA Base #
#############
RUN apt-get update && apt-get install -y --no-install-recommends \
    gnupg2 curl ca-certificates && \
    curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
    echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
    echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list \
    && rm -rf /var/lib/apt/lists/*

ENV CUDA_VERSION 11.2.2

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-cudart-11-2=11.2.152-1 \
    cuda-compat-11-2 \
    && ln -s cuda-11.2 /usr/local/cuda && \
    rm -rf /var/lib/apt/lists/*

# Required for nvidia-docker v1
RUN echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf \
    && echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

ENV PATH /usr/local/nvidia/bin:/usr/local/cuda/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES all
ENV NVIDIA_DRIVER_CAPABILITIES compute,utility
ENV NVIDIA_REQUIRE_CUDA "cuda>=11.2 brand=tesla,driver>=418,driver<419 brand=tesla,driver>=440,driver<441 driver>=450,driver<451"

################
# CUDA Runtime #
################
ENV NCCL_VERSION 2.8.4

RUN apt-get update && apt-get install -y --no-install-recommends \
    cuda-libraries-11-2=11.2.2-1 \
    libnpp-11-2=11.3.2.152-1 \
    cuda-nvtx-11-2=11.2.152-1 \
    libcublas-11-2=11.4.1.1043-1 \
    libcusparse-11-2=11.4.1.1152-1 \
    libnccl2=$NCCL_VERSION-1+cuda11.2 \
    && rm -rf /var/lib/apt/lists/*

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
RUN apt-mark hold libcublas-11-2 libnccl2


##########
# CUDDNN #
##########
ENV CUDNN_VERSION 8.1.1.33

LABEL com.nvidia.cudnn.version="${CUDNN_VERSION}"

RUN apt-get update && apt-get install -y --no-install-recommends \
    libcudnn8=$CUDNN_VERSION-1+cuda11.2 \
    && apt-mark hold libcudnn8 && \
    rm -rf /var/lib/apt/lists/*


##################################################################
# ------------------------------ END -------------------------- #
##################################################################



# >>>>>>>>>>> ROS2 dependencies
RUN apt-get update && \
    apt-get install -y apt-utils debconf-utils gnupg2 lsb-release && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list && \
    rosdep update && \
    apt-get update 
RUN RTI_NC_LICENSE_ACCEPTED=yes apt-get install rti-connext-dds-5.3.1
# <<<<<<<<<<<< ROS2 dependencies

# >>>>>>>>>>>> Python packages
RUN apt-get update && \
    apt-get install -y \
       python3-tk \
       python3-pip && \
       rm -rf /var/lib/apt/lists/*

RUN pip3 install --upgrade pip && \
    pip3 install \
       torch 

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
        onnxruntime-gpu \
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
        opencv-contrib-python

# <<<<<<<<<<<<< Python packages


# >>>>>>>>>>>> Gazebo
# Install Gazebo
RUN curl -sSL http://get.gazebosim.org | sh
# <<<<<<<<<<<< Gazebo

# >>>>>>>>>>>> ROS2_packages
RUN apt-get update -y
RUN apt-get upgrade -y
RUN apt install -y ros-foxy-gazebo-ros-pkgs
RUN apt install -y ros-foxy-ros-core
RUN apt install -y ros-foxy-navigation2
RUN apt install -y ros-foxy-nav2-bringup
RUN apt install -y ros-foxy-turtlebot3
RUN apt install -y ros-foxy-geometry2
# <<<<<<<<<<<< ROS2_packages


## >>>>>>>>>>>> OpenCV
RUN apt-get install -y libgstreamer1.0-0 \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    gstreamer1.0-doc \
    gstreamer1.0-tools \
    gstreamer1.0-x \
    gstreamer1.0-alsa \
    gstreamer1.0-gl \
    gstreamer1.0-gtk3 \
    gstreamer1.0-qt5 \
    gstreamer1.0-pulseaudio \
    libgstreamer-plugins-base1.0-dev 


# RUN git clone https://github.com/opencv/opencv.git --branch=4.5.0
# RUN git clone https://github.com/opencv/opencv_contrib.git --branch=4.5.0

# RUN mkdir opencv/build
# RUN cd opencv/build && \
#     ls && \
#     cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DWITH_GSTREAMER=ON   # -DBUILD_opencv_python3=ON .. && \
#   make -j3 && \
#   make install
## <<<<<<<<<<<< OpenCV


# >>>>>>>>>>>> OAK-D camera utils
RUN apt-get update && apt-get install -y wget build-essential cmake pkg-config libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran
RUN wget http://docs.luxonis.com/_static/install_dependencies.sh && chmod +x install_dependencies.sh && ./install_dependencies.sh
RUN wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2
RUN tar xf libusb-1.0.24.tar.bz2
RUN cd libusb-1.0.24 && \
    ./configure --disable-udev && \
    make -j4 && make install

RUN pip install --extra-index-url https://www.piwheels.org/simple/ opencv-python

RUN git clone https://github.com/luxonis/depthai-python /depthai-python
RUN mkdir -p /depthai-python/build
RUN cd /depthai-python && \
    git clone https://github.com/luxonis/depthai-core && \
    git submodule update --init --recursive && \
    cd build && \
    cmake .. && \
    make -j4
ENV PYTHONPATH=/depthai-python/build
# <<<<<<<<<<<< OAK-D camera utils



# >>>>>>>>>>>> Utils 
RUN apt-get update && \
    apt-get install -y \
        tmux \
        ranger \
        wget \
        neovim && \
        rm -rf /var/lib/apt/lists/*
# <<<<<<<<<<<< Utils






# >>>>>>>>>>>> Create User
ENV ROSUSER="user"

RUN adduser --home /home/${ROSUSER} --shell /bin/zsh --gecos '' --disabled-password ${ROSUSER} && \
    usermod -a -G root,sudo,video ${ROSUSER} && \
    echo "${ROSUSER} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${ROSUSER}
user $ROSUSER
# <<<<<<<<<<<< Create User


# >>>>>>>>>>>> OpenCV
COPY scripts/build_opencv.sh .
RUN ./build_opencv.sh
# <<<<<<<<<<<< OpenCV

# >>>>>>>>>>>> Scripts
RUN mkdir -p /home/$ROSUSER/scripts
WORKDIR /home/$ROSUSER/scripts

COPY scripts/utils-setup.sh .
RUN ./utils-setup.sh

COPY scripts/ros-setup.sh .
RUN ./ros-setup.sh


WORKDIR /home/$ROSUSER
RUN rm -rf scripts
# <<<<<<<<<<<< Scripts


# >>>>>>>>>>>> OAK-D repo for testing
RUN git clone https://github.com/luxonis/depthai
RUN git clone https://github.com/luxonis/depthai-python
# <<<<<<<<<<<< OAK-D repo for testing
