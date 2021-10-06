from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    default_update_rate = '10'
    new_update_rate = LaunchConfiguration(
        'rosbot_update_rate',
        default=default_update_rate
    )

    rosbot_controller_dir = get_package_share_directory('rosbot_controller')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbot_update_rate',
            default_value=default_update_rate,
            description='rosbot update_rate'
        ),

        Node(
            package='rosbot_controller',
            executable='spawn_rosbot',
            name='spawn_rosbot',
            output='screen',
            parameters=[{"rosbot_update_rate":new_update_rate},],
        ),

        Node(
            package='rosbot_controller',
            executable='publish_rosbot_tf',
            name='publish_rosbot_tf',
            output='screen',
            emulate_tty=True,
        ),
    ])
