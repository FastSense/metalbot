from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():


    defaut_control_topic = "/cmd_vel"
    default_parent_topic = "/odom"
    defaut_v_max = "2.5"
    defaut_w_max = "2.5"

    control_topic = LaunchConfiguration(
        "control_topic", default=defaut_control_topic)
    parent_topic = LaunchConfiguration(
        "parent_topic", default=default_parent_topic)
    
    v_max = LaunchConfiguration('v_max', default=defaut_v_max)
    w_max = LaunchConfiguration('w_max', default=defaut_w_max)

    return LaunchDescription([

        DeclareLaunchArgument('control_topic',
                              default_value=defaut_control_topic,
                              description='Topic in which we publish a control information'
                              ),
        DeclareLaunchArgument('parent_topic',
                              default_value=default_parent_topic,
                              description="Topic in which information about the current position is published"
                              ),
        DeclareLaunchArgument('v_max',
                              default_value=defaut_v_max,
                              description='Maximum forward speed'
                              ),

        DeclareLaunchArgument('w_max',
                              default_value=defaut_w_max,
                              description='Maximum value of a rotation speed around axe z'
                              ),

        Node(
            package='rosbot_controller',
            executable='path_follower',
            name='path_follower',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"control_topic": control_topic},
                {"parent_topic": parent_topic},
                {"v_max": v_max},
                {"w_max": w_max},
            ]
        ),

    ])
