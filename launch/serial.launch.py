import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    rosbot_description = get_package_share_directory('rosbot_description')

    rosserial = launch_ros.actions.Node(
        package='rosbot_description',
        executable='rosserial_node.py',
        output='screen',
        parameters=[
            rosbot_description + '/config/rosserial.yaml'
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        rosserial
    ])


if __name__ == '__main__':
    generate_launch_description()
