echo "alias n='source /opt/ros/noetic/setup.zsh'" >> ~/.zshrc && \
echo "alias f='source /opt/ros/foxy/setup.zsh'" >> ~/.zshrc && \

echo "alias colcon_build='colcon build --symlink-install --packages-skip ros1_bridge'" >> ~/.zshrc && \
echo "alias colcon_build_bridge='colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure '" >> ~/.zshrc && \

echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> ~/.zshrc && \
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.zshrc
