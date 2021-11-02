from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

import xacro
import os

def generate_launch_description():
    robot_name = 'metalbot'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    default_update_rate = '10'
    odom_update_rate = LaunchConfiguration(
        'odom_update_rate',
        default=default_update_rate
    )

    models_pkg = get_package_share_directory('metalbot_gazebo')

    # Xacro
    xacro_file = os.path.join(models_pkg, 'urdf', 'rosbot.xacro')
    xacro_xml = xacro.process_file(xacro_file).toxml()

    # Sdf or Urdf 
    sdf = os.path.join(models_pkg, 'models', 'metalbot.sdf')
    sdf_xml = open(sdf, 'r').read()
    sdf_xml = sdf_xml.replace('"', '\\"')
    spawn_args = '{name: \"robot\", xml: \"'  +  sdf_xml + '\" }'

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': xacro_xml},
            {'use_sim_time': use_sim_time},
            {'use_tf_static': True},
            {'publish_frequency': 20.0}]
    )

    spawn_process = ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_args], 
            output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'odom_update_rate',
            default_value=default_update_rate,
            description=''
        ),
        robot_state_publisher,
        spawn_process
    ])
