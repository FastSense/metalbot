from launch import LaunchDescription
import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    
    default_control_topic = "/cmd_vel"
    default_odom_topic = "/odom"
    defaut_v_max = "2.5"
    defaut_w_max = "2.5"
    default_cmd_freq = "30.0"
    default_kill_follower = "True"

    rosbot_controller = get_package_share_directory('rosbot_controller')

    path_publisher = IncludeLaunchDescription(PythonLaunchDescriptionSource(rosbot_controller + '/path_publisher.launch.py'))
    path_follower = IncludeLaunchDescription(PythonLaunchDescriptionSource(rosbot_controller + '/path_follower.launch.py'))

    return LaunchDescription([
        DeclareLaunchArgument('control_topic', default_value=default_control_topic, description='Topic in which we publish a control information'),
        DeclareLaunchArgument('odom_topic', default_value=default_odom_topic, description="Topic in which information about the current position is published"),
        DeclareLaunchArgument('v_max', default_value=defaut_v_max,description='Maximum forward speed'),
        DeclareLaunchArgument('w_max', default_value=defaut_w_max, description='Maximum value of a rotation speed around axe z'),
        DeclareLaunchArgument('cmd_freq', default_value=default_cmd_freq, description='Frequency of publishing control of a rosbot'),
        DeclareLaunchArgument('kill_follower', default_value=default_kill_follower, description='Signal, that we have to kill the follower process'),
        path_follower,
        path_publisher,
    ])


if __name__ == '__main__':
    generate_launch_description()
