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

def generate_launch_description():
#Teleop
    default_update_rate = '20'
    update_rate = launch.substitutions.LaunchConfiguration(
        'update_rate',
        default=default_update_rate
    )

    keyboard_listener_dir = get_package_share_directory('teleop')
    keyboard_listener_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    keyboard_listener_dir + '/keyboard_listener.launch.py'
            ),
            launch_arguments = {'output_path': update_rate}.items()
    )

    return LaunchDescription([
        
        launch.actions.DeclareLaunchArgument(
            'update_rate',
            default_value=default_update_rate,
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
            executable="path_visualizer"
        ),
        # launh Teleop node
        Node(
            package='rosbot_controller',
            executable='rosbot_teleop',
            name='rosbot_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"update_rate": update_rate},
                {'keyboard_topic': "/keyboard"},
                {'control_topic': "/cmd_vel"},
                {'joystick_topic': "/joy"},
                {'movable_camera': "False"},
                {'v_limit': "0.5"},
                {'w_limit': "2.5"},
                {'lin_a': "0.1"},
                {'ang_a': "0.25"},
            ]
        ),
    ])