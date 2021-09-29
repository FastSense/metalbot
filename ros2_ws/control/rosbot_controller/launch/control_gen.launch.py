import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    # Declare 
    defaut_mode = "periodic"
    defaut_num_of_subs = "1"
    defaut_control_topic = "/cmd_vel"
    defaut_pub_rate = "30"
    defaut_max_time = "10"
    defaut_period_lin = "5"
    defaut_period_ang = "5"
    defaut_v_min = "0.0"
    defaut_v_max = "1.5"
    defaut_w_min = "0.0"
    defaut_w_max = "2.5"
    defaut_a_lin = "0.25"
    defaut_a_ang = "0.25"
    defaut_file_path = ""

    mode = LaunchConfiguration('mode', default=defaut_mode)
    num_of_subs = LaunchConfiguration('num_of_subs', default=defaut_num_of_subs)
    control_topic = LaunchConfiguration('control_topic', default=defaut_control_topic)
    pub_rate = LaunchConfiguration('pub_rate', default=defaut_pub_rate)
    max_time = LaunchConfiguration('Tmax', default=defaut_max_time)
    period_lin = LaunchConfiguration('period_lin', default=defaut_period_lin)
    period_ang = LaunchConfiguration('period_ang', default=defaut_period_ang)
    v_min = LaunchConfiguration('v_min', default=defaut_v_min)
    v_max = LaunchConfiguration('v_max', default=defaut_v_max)
    w_min = LaunchConfiguration('w_min', default=defaut_w_min)	
    w_max = LaunchConfiguration('w_max', default=defaut_w_max)
    a_lin = LaunchConfiguration('a_lin', default=defaut_a_lin)
    a_ang = LaunchConfiguration('a_ang', default=defaut_a_ang)
    file_path = LaunchConfiguration('file_path', default=defaut_file_path)


    return LaunchDescription([

        DeclareLaunchArgument('mode', default_value=defaut_mode, description='periodic / from_file'),
        DeclareLaunchArgument('num_of_subs', default_value=defaut_num_of_subs, description='Number of desired subscribers'),
        DeclareLaunchArgument('control_topic', default_value=defaut_control_topic, description='Control topic name'),
        DeclareLaunchArgument('pub_rate', default_value=defaut_pub_rate, description='Control publication frequency'),
        DeclareLaunchArgument('Tmax', default_value=defaut_max_time, description='control generator running time'),
        DeclareLaunchArgument('period_lin', default_value=defaut_period_lin, description='Linear velocity change period'),
        DeclareLaunchArgument('period_ang', default_value=defaut_period_ang, description='Angular velocity change period'),
        DeclareLaunchArgument('v_min', default_value=defaut_v_min, description='Minimum linear speed'),
        DeclareLaunchArgument('v_max', default_value=defaut_v_max, description='Maximum linear speed'),
        DeclareLaunchArgument('w_min', default_value=defaut_w_min, description='Minimum angular speed'),
        DeclareLaunchArgument('w_max', default_value=defaut_w_max, description='Maximum angular speed'),
        DeclareLaunchArgument('a_lin', default_value=defaut_a_lin, description='Linear acceleration'),
        DeclareLaunchArgument('a_ang', default_value=defaut_a_ang, description='Angular acceleration'),
        DeclareLaunchArgument('file_path', default_value=defaut_file_path, description='Path to file with control'),

        # launh Logger node
        Node(
            package='rosbot_controller',
            executable='control_gen',
            name='control_gen',
            output='screen',
            # emulate_tty=True,
            parameters=[
                {"mode":mode},
                {"num_of_subs":num_of_subs},
                {"control_topic":control_topic},
                {"pub_rate":pub_rate},
                {"Tmax": max_time},
                {"period_lin":period_lin},
                {"period_ang":period_ang},
                {"v_min":v_min},
                {"v_max":v_max},
                {"w_min":w_min},
                {"w_max":w_max},
                {"a_lin":a_lin},
                {"a_ang":a_ang},
                {"file_path":file_path},
            ]
        ),

    ])