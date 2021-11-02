from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# If using xacro
# import xacro

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
    # xacro_file = os.path.join(rosbot_description_dir, 'urdf', 'rosbot.xacro')
    # xml = xacro.process_file(xacro_file).toxml()

    # Sdf or Urdf 
    urdf = os.path.join(models_pkg, 'models', 'metalbot.sdf')
    xml = open(urdf, 'r').read()

    # Common part
    xml = xml.replace('"', '\\"')
    spawn_args = '{name: \"Robot\", xml: \"'  +  xml + '\" }'

    description = {'robot_description': xml}
    use_sim_time_param = {'use_sim_time': use_sim_time}
    use_tf_static = {'use_tf_static': True}
    publish_frequency = {'publish_frequency': 20.0}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[description, use_sim_time_param, use_tf_static, publish_frequency]
    )


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
        ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', spawn_args], output='screen'),
        robot_state_publisher,
    ])
