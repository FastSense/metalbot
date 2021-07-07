from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    return LaunchDescription([


        Node(
            package='rosbot_controller',
            executable='model_runner',
            name='model_runner',
            output='screen',
            parameters=[
            ]
        ),

    ])
