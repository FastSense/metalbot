# opencv with Gstreamer

cd /home/user
pwd
ls
sudo apt-get install -y libgstreamer1.0-0 gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-doc gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio libgstreamer-plugins-base1.0-dev 

git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
pwd
ls
mkdir opencv/build
cd opencv/build
pwd
ls
cmake -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules -DWITH_GSTREAMER=ON -DBUILD_opencv_python3=ON ..
make -j4
sudo make install