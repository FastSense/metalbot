mkdir -p ~/ros2_ws/src && \

echo "alias n='source /opt/ros/noetic/setup.zsh'" >> ~/.zshrc && \
echo "source ~/ros2_ws/install/setup.zsh" >> ~/.zshrc && \

echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> ~/.zshrc && \
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.zshrc
# echo "export ROS_LOCALHOST_ONLY=1" >> ~/.zshrc
# echo "export PYTHONPATH="/usr/local/lib/python3.8/dist-packages/cv2/python-3.8/"" >> ~/.zshrc
