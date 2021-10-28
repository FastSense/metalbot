from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time = LaunchConfiguration(
        'use_sim_time', default='false')
    rosbot_pkg = get_package_share_directory('rosbot')

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        Node(
            package='rosbot_controller',
            executable='publish_rosbot_tf',
            name='publish_rosbot_tf',
            output='screen',
            emulate_tty=True,
        ),

    ])


if __name__ == '__main__':
    generate_launch_description()
