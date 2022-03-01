from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    camera_link_t265_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['-0.14', '-0.01', '-0.215', '0.0', '0.', '0.0', 'camera_pose_frame', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    odom_tf = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='log',
        arguments=['0.14', '0.01', '0.215', '0.0', '0.', '0.0', 'odom', 'odom_frame'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='false'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        camera_link_t265_tf,
        odom_tf,
    ])

if __name__ == '__main__':
    generate_launch_description()

    
