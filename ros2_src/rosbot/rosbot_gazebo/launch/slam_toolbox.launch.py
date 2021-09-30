from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg = FindPackageShare('rosbot_gazebo')
    config = PathJoinSubstitution([pkg, 'config', 'slam_toolbox_sim.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),

            
        Node(
            parameters=[ config ],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
