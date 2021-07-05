from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    defaut_control_topic = "/cmd_vel"
    defaut_v_max = "2.5"
    defaut_w_max = "2.5"

    control_topic = LaunchConfiguration(
        "control_topic", default=defaut_control_topic)
    v_max = LaunchConfiguration('v_max', default=defaut_v_max)
    w_max = LaunchConfiguration('w_max', default=defaut_w_max)

    return LaunchDescription([


        Node(
            package='rosbot_controller',
            executable='path_follower',
            name='path_follower',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"control_topic": control_topic},
                {"v_max": v_max},
                {"w_max": w_max},
            ]
        ),

    ])