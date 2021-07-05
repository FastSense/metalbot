from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    default_traj_type = "1.0sin1.0"
    default_move_plan = ""
    default_num_of_subs = 1

    traj_type = LaunchConfiguration('traj_type', default=default_traj_type)
    move_plan = LaunchConfiguration('move_plan', default=default_move_plan)
    num_of_subs = LaunchConfiguration('num_of_subs', default=default_num_of_subs)
    
    return LaunchDescription([

        DeclareLaunchArgument('traj_type',
                              default_value=default_traj_type,
                              description='1.0sin2.0 / 2.0spiral / polygon / from_file'
                              ),
        DeclareLaunchArgument('move_plan',
                              default_value=default_move_plan,
                              description="Path to file with a plan of movement"
                              ),

        Node(
            package='rosbot_controller',
            executable='path_publisher',
            name='path_publisher',
            output='screen',
            parameters=[
                {"traj_type": traj_type},
                {"move_plan": move_plan},
                {"num_of_subs": num_of_subs},
            ]
        ),

    ])
