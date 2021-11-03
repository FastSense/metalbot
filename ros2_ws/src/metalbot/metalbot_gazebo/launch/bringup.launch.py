from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_gui       = LaunchConfiguration('gui')

    gazebo_ros_pkg   = FindPackageShare('gazebo_ros')
    gazebo_robot_pkg = FindPackageShare('metalbot_gazebo')

    gz_server_launch = PathJoinSubstitution([gazebo_ros_pkg,     'launch',  'gzserver.launch.py'])
    gz_client_launch = PathJoinSubstitution([gazebo_ros_pkg,     'launch',  'gzclient.launch.py'])
    spawn_launch     = PathJoinSubstitution([gazebo_robot_pkg,   'launch',  'spawn.launch.py'])
    world            = PathJoinSubstitution([gazebo_robot_pkg,    'worlds',  'willow_garage.world'])

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=[world, ''], description='SDF world file'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true',description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('gdb', default_value='false', description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('state', default_value='true', description='Set "false" not to load "libgazebo_ros_state.so"'),
        DeclareLaunchArgument('use_tf_static', default_value='true', description='Use static transforms'),

        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_server_launch)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(gz_client_launch), condition=IfCondition(use_gui)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(spawn_launch))
    ])

if __name__ == '__main__':
    generate_launch_description()
