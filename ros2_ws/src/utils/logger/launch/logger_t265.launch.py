import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    I can't use get_package_share_directory, cause it will return
    'ros2_ws/install/logger/share/logger', BUT when using 'ros2 launch' 
    ROS launches from its workspace so I can easy get desired directory

    * I checked it works well when launch from any directory *
    """
    output_dir = os.path.join(os.getcwd(), 'src/logger/output_data/')
    output_path = launch.substitutions.LaunchConfiguration(
        'output_path',
        default=output_dir
    )

    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'output_path',
            default_value=output_dir,
            description='output_folder path'
        ),

        # launh Logger node
        Node(
            package='logger',
            executable='logger',
            name='logger',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"output_path": output_path},
                {"control_topic": "/cmd_vel"},
                {"tf_topic": "/tf"},
                {"parent_frame": "odom"},
                {"robot_frame": "base_link"},
                {"kinetic_model_frame": "model_link"},
                {"nn_model_frame": "nn_model_link"},
                {"use_odom_topic": False}
            ]
        ),

    ])
