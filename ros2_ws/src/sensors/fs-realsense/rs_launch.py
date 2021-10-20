from launch import LaunchDescription
from launch_ros.actions import Node

DEFAULT_CONFIG_PATH = "/home/user/ros2_ws/src/sensors/fs-realsense/config/d455.yaml"

def generate_launch_description():
    config_path = DEFAULT_CONFIG_PATH
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='rs',
            parameters=[f'{config_path}']
        )
    ])
