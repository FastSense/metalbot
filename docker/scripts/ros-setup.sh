grid_map_pkgs="grid_map grid_map_demos grid_map_pcl grid_map_cmake_helpers grid_map_filters grid_map_ros grid_map_core grid_map_loader grid_map_rviz_plugin grid_map_costmap_2d grid_map_msgs grid_map_sdf grid_map_cv grid_map_octomap grid_map_visualization" 
realsense_pkgs="realsense2_camera realsense2_camera_msgs realsense2_description"
basic_ignore_pkgs="${grid_map_pkgs} ${realsense_pkgs} ros1_bridge groot oakd rplidar_ros rosbot_description"

echo "alias r1='source /opt/ros/noetic/setup.zsh && source ~/ros1_ws/devel/setup.zsh'" >> ~/.zshrc
echo "alias r2='source /opt/ros/foxy/setup.zsh && source ~/ros2_ws/install/setup.zsh'" >> ~/.zshrc

echo "alias cb_selected='colcon_build_selected_release'" >> ~/.zshrc

echo "
function colcon_build_selected_release {
  colcon build --packages-select "\$\@" --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release 
} " >> ~/.zshrc

echo "alias cb_basic='colcon build --symlink-install --packages-skip ${basic_ignore_pkgs}'" >> ~/.zshrc && \
echo "alias cb_grid_map='colcon build --symlink-install --packages-select ${grid_map_pkgs}'" >> ~/.zshrc
echo "alias cb_oakd='colcon build --symlink-install --packages-select oakd'" >> ~/.zshrc
echo "alias cb_realsense='colcon build --packages-select ${realsense_pkgs}'" >> ~/.zshrc
echo "alias cb_oakd='colcon build --symlink-install --packages-select oakd'" >> ~/.zshrc
echo "alias cb_rplidar='colcon build --symlink-install --packages-select rplidar_ros'" >> ~/.zshrc
echo "alias cb_bridge='colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure '" >> ~/.zshrc
echo "alias cb_gazebo='colcon build --symlink-install --packages-select rosbot_description'" >> ~/.zshrc

echo "export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11" >> ~/.zshrc
echo "export TURTLEBOT3_MODEL=waffle" >> ~/.zshrc
