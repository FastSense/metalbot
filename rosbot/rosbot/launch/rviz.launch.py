import os

from launch import LaunchDescription

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('rosbot')
    config=os.path.join(pkg, 'rviz', 'rviz2.rviz'),

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', config],
        output='screen')

    return LaunchDescription([
        rviz_node
    ])
