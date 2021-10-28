from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    default_update_rate = '10'
    odom_update_rate = LaunchConfiguration(
        'odom_update_rate',
        default=default_update_rate
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'odom_update_rate',
            default_value=default_update_rate,
            description=''
        ),

        Node(
            package='metalbot_gazebo',
            executable='spawn_robot.py',
            name='spawn_robot',
            output='screen',
            parameters=[{"odom_update_rate":odom_update_rate},],
        ),

        Node(
            package='metalbot',
            executable='publish_tf.py',
            name='publish_tf',
            output='screen',
            emulate_tty=True,
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
