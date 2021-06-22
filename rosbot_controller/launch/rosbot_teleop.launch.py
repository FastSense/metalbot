import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    default_update_rate = '20'
    update_rate = launch.substitutions.LaunchConfiguration(
        'update_rate',
        default=default_update_rate
    )

    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'update_rate',
            default_value=default_update_rate,
            description='update rate'
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
                {'v_limit': "1.5"},
                {'w_limit': "2.5"},
                {'lin_a': "0.25"},
                {'ang_a': "0.25"},
            ]
        ),

    ])
