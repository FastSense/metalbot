import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    rosbot = get_package_share_directory('rosbot')
    # rplidar_ros = get_package_share_directory('rplidar_ros')

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

    rosserial = launch_ros.actions.Node(
        package='rosbot',
        executable='rosserial_node.py',
        output='screen',
        parameters=[
    		rosbot + '/config/rosserial.yaml'
        ]
    )

    rosbot_tf = launch_ros.actions.Node(
        package='rosbot',
        executable='rosbot_tf',
        output='log',
    )

    # rp_lidar = launch.actions.IncludeLaunchDescription(
    #     launch.launch_description_sources.PythonLaunchDescriptionSource(
    #         os.path.join(rplidar_ros, 'launch', 'rplidar.launch.py'))
    # )

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
        rosserial,
        rosbot_tf,
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.08', '0.1', '0', '0', '0', '0', 'base_link', 'front_left_wheel'],
        #     ),        
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0.08', '-0.1', '0', '0', '0', '0', 'base_link', 'front_right_wheel'],
        #     ),
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['-0.08', '0.1', '0', '0', '0', '0', 'base_link', 'rear_left_wheel'],
        #     ),        
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['-0.08', '-0.1', '0', '0', '0', '0', 'base_link', 'rear_right_wheel'],
        #     ),
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'top'],
        # ),
        # launch_ros.actions.Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     output='screen',
        #     arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'camera_link'],
        # ),

    ])

if __name__ == '__main__':
    generate_launch_description()

    
