from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rosbot = get_package_share_directory('rosbot')

    camera_link_d455_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['0.16', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
        parameters=[
    		rosbot + '/config/static_tf.yaml'
        ]
    )

    camera_link_t265_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['-0.14', '-0.01', '-0.215', '0.0', '0.0', '0.0', 'camera_pose_frame', 'base_link'],
        parameters=[
    		rosbot + '/config/static_tf.yaml'
        ]
    )

    odom_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['0.14', '0.01', '0.215', '0.0', '0', '0.0', 'odom', 'odom_frame'],
        parameters=[
    		rosbot + '/config/static_tf.yaml'
    	],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        camera_link_d455_tf,
        camera_link_t265_tf,
        odom_tf,
    ])

if __name__ == '__main__':
    generate_launch_description()

    
