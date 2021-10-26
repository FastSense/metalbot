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

enable_viz = 'true'

def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'viz',
            default_value=enable_viz,
            description='update rate'
        ),
        Node(
            package="state_estimation_2d",
            executable="state_estimation_2d",
            output='screen',
            emulate_tty=True,
            parameters=[{"use_sim_time": False}],
        ),
        Node(
            package="path_visualizer",
            executable="path_visualizer",
            condition=IfCondition(launch.substitutions.LaunchConfiguration("viz")),
        ),
    ])