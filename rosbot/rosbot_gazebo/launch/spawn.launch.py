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

    rosbot_description_dir = get_package_share_directory('rosbot_description')

    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbot_update_rate',
            default_value=default_update_rate,
            description='rosbot update_rate'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame'],
            parameters=[rosbot_description_dir + '/config/static_tf.yaml'],
            ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dummy'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0.2', '0', '0.8', '0', '0', '0', 'base_link', 'camera_rgb_frame'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_fl'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_fr'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_rl'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'range_rr'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top'],
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['-0.03', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
            parameters=[rosbot_description_dir + '/config/static_tf.yaml']
            ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0.07', '0', '0', '0', 'base_link', 'laser'],
            parameters=[rosbot_description_dir + '/config/static_tf_sim.yaml']
            ),
        Node(
            package='rosbot_controller',
            executable='spawn_rosbot',
            name='spawn_rosbot',
            output='screen',
            parameters=[{"rosbot_update_rate":new_update_rate},],
        ),
    ])
