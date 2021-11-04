#!/bin/bash

##################################################################
# ------------------------------ CUDA -------------------------- #
##################################################################

#############
# CUDA Base #
#############
sudo apt-get update

sudo apt-get install -y --no-install-recommends \
      gnupg2 curl ca-certificates && \
      curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub | apt-key add - && \
      echo "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list && \
      echo "deb https://developer.download.nvidia.com/compute/machine-learning/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/nvidia-ml.list 

sudo apt-get update

sudo apt-get install -y --no-install-recommends \
     cuda-cudart-11-2=11.2.152-1 \
     cuda-cudart-dev-11-2=11.2.152-1 \
     cuda-compat-11-2 \
     && ln -s cuda-11.2 /usr/local/cuda

# Required for nvidia-docker v1
sudo echo "/usr/local/nvidia/lib" >> /etc/ld.so.conf.d/nvidia.conf
sudo echo "/usr/local/nvidia/lib64" >> /etc/ld.so.conf.d/nvidia.conf

sudo apt-get install -y --no-install-recommends \
     cuda-libraries-11-2=11.2.2-1 \
     cuda-libraries-dev-11-2=11.2.2-1 \
     libnpp-11-2=11.3.2.152-1 \
     libnpp-dev-11-2=11.3.2.152-1 \
     cuda-nvtx-11-2=11.2.152-1 \
     libcublas-11-2=11.4.1.1043-1 \
     libcublas-dev-11-2=11.4.1.1043-1 \
     libcusparse-11-2=11.4.1.1152-1 \
     libcusparse-dev-11-2=11.4.1.1152-1 \
     libnccl2=$NCCL_VERSION-1+cuda11.2 \
     cuda-nvcc-11-2

# apt from auto upgrading the cublas package. See https://gitlab.com/nvidia/container-images/cuda/-/issues/88
sudo apt-mark hold libcublas-11-2 libcublas-dev-11-2 libnccl2


##########
# CUDDNN #
##########
sudo apt-get install -y --no-install-recommends \
    libcudnn8=$CUDNN_VERSION-1+cuda11.2 \
    libcudnn8-dev=$CUDNN_VERSION-1+cuda11.2 \
    && apt-mark hold libcudnn8 

sudo rm -rf /var/lib/apt/lists/*


##################################################################
# ------------------------------ END -------------------------- #
##################################################################
