import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Declare 
    defaut_mode = "periodic"
    defaut_num_of_subs = "1"
    defaut_control_topic = "/cmd_vel"
    defaut_pub_rate = "30"
    defaut_Tmax = "20"
    defaut_period_lin = "5"
    defaut_period_ang = "5"
    defaut_v_min = "0.0"
    defaut_v_max = "1.5"
    defaut_w_min = "0.0"
    defaut_w_max = "2.5"
    defaut_a_lin = "0.25"
    defaut_a_ang = "-0.25"
    defaut_file_path = ""

    mode = launch.substitutions.LaunchConfiguration(
        'mode',
        default=defaut_mode
    )

    num_of_subs = launch.substitutions.LaunchConfiguration(
        'num_of_subs',
        default=defaut_num_of_subs
    )

    control_topic = launch.substitutions.LaunchConfiguration(
        'control_topic',
        default=defaut_control_topic
    )

    pub_rate = launch.substitutions.LaunchConfiguration(
        'pub_rate',
        default=defaut_pub_rate
    )

    Tmax = launch.substitutions.LaunchConfiguration(
        'Tmax',
        default=defaut_Tmax
    )

    period_lin = launch.substitutions.LaunchConfiguration(
        'period_lin',
        default=defaut_period_lin
    )
    
    period_ang = launch.substitutions.LaunchConfiguration(
        'period_ang',
        default=defaut_period_ang
    )
    
    v_min = launch.substitutions.LaunchConfiguration(
        'v_min',
        default=defaut_v_min
    )
    
    v_max = launch.substitutions.LaunchConfiguration(
        'v_max',
        default=defaut_v_max
    )
    
    w_min = launch.substitutions.LaunchConfiguration(
        'w_min',
        default=defaut_w_min
    )
    
    w_max = launch.substitutions.LaunchConfiguration(
        'w_max',
        default=defaut_w_max
    )
    
    a_lin = launch.substitutions.LaunchConfiguration(
        'a_lin',
        default=defaut_a_lin
    )
    
    a_ang = launch.substitutions.LaunchConfiguration(
        'a_ang',
        default=defaut_a_ang
    )
    
    file_path = launch.substitutions.LaunchConfiguration(
        'file_path',
        default=defaut_file_path
    )


    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'mode',
            default_value=defaut_mode,
            description='periodic / from_file'
        ),

        launch.actions.DeclareLaunchArgument(
            'num_of_subs',
            default_value=defaut_num_of_subs,
            description='Number of desired subscribers'
        ),

        launch.actions.DeclareLaunchArgument(
            'control_topic',
            default_value=defaut_control_topic,
            description='Control topic name'
        ),

        launch.actions.DeclareLaunchArgument(
            'pub_rate',
            default_value=defaut_pub_rate,
            description='Control publication frequency'
        ),

        launch.actions.DeclareLaunchArgument(
            'Tmax',
            default_value=defaut_Tmax,
            description='control generator running time'
        ),

        launch.actions.DeclareLaunchArgument(
            'period_lin',
            default_value=defaut_period_lin,
            description='Linear velocity change period'
        ),

        launch.actions.DeclareLaunchArgument(
            'period_ang',
            default_value=defaut_period_ang,
            description='Angular velocity change period'
        ),

        launch.actions.DeclareLaunchArgument(
            'v_min',
            default_value=defaut_v_min,
            description='Minimum linear speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'v_max',
            default_value=defaut_v_max,
            description='Maximum linear speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'w_min',
            default_value=defaut_w_min,
            description='Minimum angular speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'w_max',
            default_value=defaut_w_max,
            description='Maximum angular speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'a_lin',
            default_value=defaut_a_lin,
            description='Linear acceleration'
        ),

        launch.actions.DeclareLaunchArgument(
            'a_ang',
            default_value=defaut_period_ang,
            description='Angular acceleration'
        ),

        launch.actions.DeclareLaunchArgument(
            'file_path',
            default_value=defaut_file_path,
            description='Path to file with control'
        ),

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
                {"Tmax": Tmax},
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