#!/bin/bash

sudo apt-get update && apt-get install -y wget build-essential cmake pkg-config libjpeg-dev libtiff5-dev libavcodec-dev libavformat-dev libswscale-dev libv4l-dev libxvidcore-dev libx264-dev libgtk2.0-dev libgtk-3-dev libatlas-base-dev gfortran
sudo wget http://docs.luxonis.com/_static/install_dependencies.sh && chmod +x install_dependencies.sh && ./install_dependencies.sh
sudo wget https://github.com/libusb/libusb/releases/download/v1.0.24/libusb-1.0.24.tar.bz2
sudo tar xf libusb-1.0.24.tar.bz2
sudo cd libusb-1.0.24 && \
    ./configure --disable-udev && \
    make -j4 && make install

sudo pip install --extra-index-url https://www.piwheels.org/simple/ opencv-python

# depthai API
sudo git clone https://github.com/luxonis/depthai-python /depthai-python
sudo mkdir -p /depthai-python/build
sudo cd /depthai-python && \
    git clone https://github.com/luxonis/depthai-core && \
    git submodule update --init --recursive && \
    cd build && \
    cmake .. && \
    make -j4

sudo PYTHONPATH=/depthai-python/build

sudo su $ROSUSER

cd ~
git clone https://github.com/luxonis/depthai
git clone https://github.com/luxonis/depthai-python

cd /

sudo su root
