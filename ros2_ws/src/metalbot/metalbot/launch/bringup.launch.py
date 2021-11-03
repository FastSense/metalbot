from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    robot_pkg        = FindPackageShare('metalbot')
    robot_state_publisher_launch = PathJoinSubstitution([robot_pkg, 'launch', 'robot_state_publisher.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument(name='use_sim_time', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true', 
            description='Set "true" to increase messages written to terminal.'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_publisher_launch))
    ])

if __name__ == '__main__':
    generate_launch_description()

    
