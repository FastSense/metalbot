from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

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
from launch_ros.actions import Node

def generate_launch_description():
# ROSBOT
    world_file_name =  'willow_garage.world'

    urdf_file_name = 'urdf/rosbot.urdf'
    urdf = os.path.join(
        get_package_share_directory('rosbot_description'),
        urdf_file_name)
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    gazebo_ros = get_package_share_directory('gazebo_ros')
    rosbot_description = get_package_share_directory('rosbot_description')

    gazebo_client = launch.actions.IncludeLaunchDescription(
    launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
                condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )

    spawn_rosbot = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(rosbot_description, 'launch', 'rosbot_spawn.launch.py'))
    )

    rviz_path = "/home/user/ros2_ws/src/state_estimation_2d/rviz/default.rviz"

#Teleop
    default_update_rate = '20'
    update_rate = launch.substitutions.LaunchConfiguration(
        'update_rate',
        default=default_update_rate
    )

    keyboard_listener_dir = get_package_share_directory('teleop')
    keyboard_listener_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    keyboard_listener_dir + '/keyboard_listener.launch.py'
            ),
            launch_arguments = {'output_path': update_rate}.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(rosbot_description, 'worlds', world_file_name), ''],
          description='SDF world file'),
        DeclareLaunchArgument(
            name='gui',
            default_value='true'
        ),
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value='true'
        ),
        DeclareLaunchArgument('verbose', default_value='true',
                              description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('gdb', default_value='false',
                              description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('state', default_value='true',
                                description='Set "false" not to load "libgazebo_ros_state.so"'),
        
        launch.actions.DeclareLaunchArgument(
            'update_rate',
            default_value=default_update_rate,
            description='update rate'
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            output='screen',
            arguments=['-d'+str(rviz_path)]
        ),
        Node(
            package="odom_noiser",
            executable="odom_noiser",
            output='screen',
            emulate_tty=True
        ),
        # Node(
        #     package="state_estimation_25d",
        #     executable="state_estimation_25d",
        #     output='screen',
        #     emulate_tty=True,
        #     parameters=[{"use_sim_time": True}],
        # ),
        Node(
            package="path_visualizer",
            executable="path_visualizer"
        ),
        Node(
            package="metric_calculator",
            executable="metric_calculator",
            output='screen',
            emulate_tty=True
        ),
        # launh Teleop node
        Node(
            package='rosbot_controller',
            executable='rosbot_teleop',
            name='rosbot_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"update_rate": update_rate},
                {'keyboard_topic': "/keyboard"},
                {'control_topic': "/cmd_vel"},
                {'joystick_topic': "/joy"},
                {'movable_camera': "False"},
                {'v_limit': "0.5"},
                {'w_limit': "2.5"},
                {'lin_a': "0.1"},
                {'ang_a': "0.25"},
            ]
        ),
        Node(
            package="robot_localization",
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            emulate_tty=True,
            parameters=[{os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')},
                        {"use_sim_time": True}],
        ),

        # launh listener node
        gazebo_server,
        gazebo_client,
        spawn_rosbot,
        keyboard_listener_launch

    ])