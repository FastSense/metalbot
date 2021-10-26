from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    launch_pkg = get_package_share_directory('realsense2_camera')
    launch_path =  launch_pkg + '/launch/rs_launch.py'

    config_pkg = get_package_share_directory('fs-realsense')
    config_path = config_pkg + '/config/d455.yaml'

    config = LaunchConfiguration('config_file')
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=f"'{config_path}'"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ launch_path ])
        )
    ])

