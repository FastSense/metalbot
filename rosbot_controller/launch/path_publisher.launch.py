from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    default_traj_type = "1.0sin1.0"

    traj_type = LaunchConfiguration('traj_type', default=default_traj_type)

    return LaunchDescription([

        DeclareLaunchArgument('traj_type',
                              default_value=default_traj_type,
                              description='1.0sin2.0 / 2.0spiral / polygon'
                              ),

        Node(
            package='rosbot_controller',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
            parameters=[
                {"traj_type": traj_type}
            ]
        ),

    ])
