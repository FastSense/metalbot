#!/bin/bash

sudo su $ROSUSER
# opencv with Gstreamer
cd /home/$ROSUSER
git clone https://github.com/opencv/opencv.git -b 4.5.0
git clone https://github.com/opencv/opencv_contrib.git -b 4.5.0
mkdir opencv/build
cd opencv/build
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DWITH_GSTREAMER=ON -DBUILD_opencv_python3=ON ..
make -j4
sudo make install

sudo su root
