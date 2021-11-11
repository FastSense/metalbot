#!/bin/bash

# requirements
sudo apt-get update
sudo apt-get install -y build-essential cmake pkg-config unzip yasm git checkinstall
sudo apt-get install -y libjpeg-dev libpng-dev libtiff-dev
sudo apt-get install -y libgtk-3-dev
sudo apt-get install -y libtbb-dev
sudo apt-get install -y libatlas-base-dev gfortran

# opencv with Gstreamer and CUDA
cd /usr/local/src
sudo git clone https://github.com/opencv/opencv.git -b 4.5.2
sudo git clone https://github.com/opencv/opencv_contrib.git -b 4.5.2
sudo mkdir opencv/build
cd opencv/build
sudo cmake \
    -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local/opencv4.5+cuda \
    -D WITH_TBB=ON \
    -D ENABLE_FAST_MATH=1 \
    -D CUDA_FAST_MATH=1 \
    -D WITH_CUBLAS=1 \
    -D WITH_CUDA=ON \
    -D BUILD_opencv_cudacodec=OFF \
    -D WITH_V4L=ON \
    -D WITH_QT=OFF \
    -D WITH_OPENGL=ON \
    -D WITH_GSTREAMER=ON \
    -D OPENCV_GENERATE_PKGCONFIG=ON \
    -D OPENCV_PC_FILE_NAME=opencv.pc \
    -D OPENCV_ENABLE_NONFREE=ON \
    -D BUILD_opencv_python3=ON \
    -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D INSTALL_C_EXAMPLES=OFF \
    -D BUILD_EXAMPLES=OFF ..
sudo make -j4
sudo make install

# create bash script to activate opencv 4.5
# echo "\
# export PATH=\"/usr/local/opencv4.5+cuda/bin:\$PATH\" \n\
# export PKG_CONFIG_PATH=\"/usr/local/opencv4.5+cuda/lib/pkgconfig:\$PKG_CONFIG_PATH\" \n\
# export LD_LIBRARY_PATH=\"/usr/local/opencv4.5+cuda/lib:\$LD_LIBRARY_PATH\"" > /home/${ROSUSER}/activate_opencv4.5.bash

# add opencv 4.5 libraries to ldconfig
echo /usr/local/opencv4.5+cuda/lib > /home/${ROSUSER}/opencv4.5+cuda.conf
sudo mv /home/${ROSUSER}/opencv4.5+cuda.conf /etc/ld.so.conf.d
sudo ldconfig

# modify .zshrc to activate opencv 4.5
# echo "\n\
# export PATH=\"/usr/local/opencv4.5+cuda/bin:\$PATH\" \n\
# export PKG_CONFIG_PATH=\"/usr/local/opencv4.5+cuda/lib/pkgconfig:\$PKG_CONFIG_PATH\" \n\
# export LD_LIBRARY_PATH=\"/usr/local/opencv4.5+cuda/lib:\$LD_LIBRARY_PATH\"" >> /home/${ROSUSER}/.zshrc

