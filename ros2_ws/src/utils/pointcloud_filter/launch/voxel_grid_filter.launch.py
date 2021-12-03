from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

import os
import yaml

def generate_launch_description():
    filter_pkg = get_package_share_directory('pointcloud_filter')
    print(filter_pkg)
    default_config_path = os.path.join(filter_pkg, 'config', 'realsense_d455.yaml')
    config_file = LaunchConfiguration('config_file', default=default_config_path)

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config_path),
        Node(
            package='pointcloud_filter',
            executable='voxel_grid_filter',
            output='screen',
            parameters=[
                config_file
            ]
        )
    ])