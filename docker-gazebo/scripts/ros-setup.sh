mkdir -p ~/ros2_ws/src && \
echo "source /opt/ros/foxy/setup.zsh" >> ~/.zshrc && \
echo "source ~/ros2_ws/install/setup.zsh" >> ~/.zshrc && \
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> ~/.zshrc && \
echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11/models" >> ~/.zshrc
