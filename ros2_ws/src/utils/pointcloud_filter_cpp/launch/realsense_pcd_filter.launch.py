from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
    Node(
        package='pointcloud_filter_cpp',
        executable='voxel_grid_filter',
        output='screen',
        parameters=[
            {'input_topic': 'camera/depth/color/points'},
            {'output_topic': 'points_filtered'},
            {'resolution': 0.02},
            {'verbose': False}
        ]
        ),
    ])