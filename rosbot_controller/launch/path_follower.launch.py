import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    return LaunchDescription([


        Node(
            package='rosbot_controller',
            executable='path_follower',
            name='path_follower',
            output='screen',
            emulate_tty=True,
        ),

    ])
