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
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dummy'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.2', '0', '0.8', '0', '0', '0', 'base_link', 'camera_rgb_frame'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_fl'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_fr'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_rl'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_rr'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.03', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_link', 'laser'],
            parameters=[{'use_sim_time': use_sim_time}],
            ),
    ])
