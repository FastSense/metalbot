from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

state_estimation_path = get_package_share_directory('state_estimation_2d')
rviz_path = os.path.join(state_estimation_path, 'rviz', 'default.rviz')

def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output='screen',
            arguments=['-d'+str(rviz_path)]
        ),
    ])