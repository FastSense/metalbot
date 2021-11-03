from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_tf_static = LaunchConfiguration('use_tf_static', default='true')

    description_pkg = get_package_share_directory('metalbot_description')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': description},
            {'use_sim_time': use_sim_time},
            {'use_tf_static': use_tf_static},
            {'publish_frequency': 20.0}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher,
    ])

