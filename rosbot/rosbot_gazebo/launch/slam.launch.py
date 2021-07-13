from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('rosbot')
    gz_pkg = get_package_share_directory('rosbot_gazebo')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_pkg, '/launch/bringup.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_pkg, '/launch/nav2.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_pkg, '/launch/slam_toolbox.launch.py']),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name="rviz2",
            arguments=['-d', pkg + "/config/rviz2.rviz"],
        ),
    ])
