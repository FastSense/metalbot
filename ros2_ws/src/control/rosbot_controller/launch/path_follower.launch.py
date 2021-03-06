from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    default_control_topic = "/cmd_vel"
    default_odom_topic = "/odom"
    default_v_max = "2.5"
    default_w_max = "2.5"
    default_vel_coeff = "1.0"
    default_ang_vel_coeff = "1.0" 
    default_cmd_freq = "30.0"
    default_kill_follower = "True"
    default_use_odom = "True"
    default_parent_frame = "odom_frame"
    default_robot_frame = "camera_pose_frame"

    control_topic = LaunchConfiguration("control_topic", default=default_control_topic)
    odom_topic = LaunchConfiguration("odom_topic", default=default_odom_topic)
    v_max = LaunchConfiguration('v_max', default=default_v_max)
    w_max = LaunchConfiguration('w_max', default=default_w_max)
    vel_coeff = LaunchConfiguration('vel_coeff', default=default_vel_coeff)
    ang_vel_coeff = LaunchConfiguration('ang_vel_coeff', default=default_ang_vel_coeff)
    cmd_freq = LaunchConfiguration('cmd_freq', default=default_cmd_freq)
    kill_follower = LaunchConfiguration('kill_follower', default=default_kill_follower)
    use_odom = LaunchConfiguration('use_odom', default=default_use_odom)
    parent_frame = LaunchConfiguration('parent_frame', default=default_parent_frame)
    robot_frame = LaunchConfiguration('robot_frame', default=default_robot_frame)

    return LaunchDescription([

        DeclareLaunchArgument('control_topic', default_value=default_control_topic, description='Topic in which we publish a control information'),
        DeclareLaunchArgument('odom_topic', default_value=default_odom_topic, description="Topic in which information about the current position is published"),
        DeclareLaunchArgument('v_max', default_value=default_v_max, description='Maximum forward speed'),
        DeclareLaunchArgument('w_max', default_value=default_w_max, description='Maximum value of a rotation speed around axe z'),
        DeclareLaunchArgument('vel_coeff', default_value=default_vel_coeff, description='linear velocity coefficient'),
        DeclareLaunchArgument('ang_vel_coeff', default_value=default_ang_vel_coeff, description='angular velocity coefficient'),
        DeclareLaunchArgument('cmd_freq', default_value=default_cmd_freq, description='Frequency of publishing control of a rosbot'),
        DeclareLaunchArgument('kill_follower', default_value=default_kill_follower, description='Signal, that we have to kill the follower process'),
        DeclareLaunchArgument('use_odom', default_value=default_use_odom, description='If true - use /odom topic for position, else use - tf topic'),
        DeclareLaunchArgument('parent_frame', default_value=default_parent_frame, description='Static TF frame'),
        DeclareLaunchArgument('robot_frame', default_value=default_robot_frame, description='Robot TF frame'),

        Node(
            package='rosbot_controller',
            executable='path_follower',
            name='path_follower',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"control_topic": control_topic},
                {"odom_topic": odom_topic},
                {"v_max": v_max},
                {"w_max": w_max},
                {"cmd_freq": cmd_freq},
                {"vel_coeff": vel_coeff},
                {"ang_vel_coeff": ang_vel_coeff},
                {"kill_follower": kill_follower},
                {"use_odom": use_odom},
                {"parent_frame": parent_frame},
                {"robot_frame": robot_frame},
            ]
        ),

    ])
