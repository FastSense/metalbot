from launch import LaunchDescription

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')
    use_slam = LaunchConfiguration('use_slam')

    pkg = get_package_share_directory('rosbot')
    gz_pkg = get_package_share_directory('rosbot_gazebo')
    return LaunchDescription([
        DeclareLaunchArgument('use_gui', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_slam', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_pkg, '/launch/bringup.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_pkg, '/launch/nav2.launch.py']),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([gz_pkg, '/launch/slam_toolbox.launch.py']), condition = IfCondition(use_slam),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([pkg, '/launch/rviz.launch.py']), condition=IfCondition(use_rviz),
        ),
    ])
