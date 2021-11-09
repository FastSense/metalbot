import launch_ros.actions
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    serial = LaunchConfiguration('serial', default='/dev/ttyACM0')

    robot_pkg                    = FindPackageShare('metalbot')
    robot_state_publisher_launch = PathJoinSubstitution([robot_pkg, 'launch', 'robot_state_publisher.launch.py'])
    micro_ros_launch             = PathJoinSubstitution([robot_pkg, 'launch', 'micro_ros.launch.py'])

    return LaunchDescription([
        DeclareLaunchArgument('serial', default_value='/dev/ttyACM0', description='Serial port'),
        DeclareLaunchArgument('use_tf_static', default_value='true', description='Use static transforms'),
        DeclareLaunchArgument(name='use_sim_time', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true', 
            description='Set "true" to increase messages written to terminal.'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(robot_state_publisher_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(micro_ros_launch))
    ])

if __name__ == '__main__':
    generate_launch_description()

    
