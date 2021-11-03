from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    serial = LaunchConfiguration('serial', default='/dev/ttyACM0')

    micro_ros = ExecuteProcess(
             cmd=['ros2', 'run', 'micro_ros_agent', 'micro_ros_agent','serial', '--dev', serial],
             output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('serial', default_value='/dev/ttyACM0', description='Serial port'),
        micro_ros,
    ])

