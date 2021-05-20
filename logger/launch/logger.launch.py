import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix

def generate_launch_description():

    logger_dir = os.path.join(os.getcwd(), 'src/logger/output_data/') 

    output_path = launch.substitutions.LaunchConfiguration(
        'output_path',
        default=logger_dir
    )
    
    # output_folder = launch.substitutions.LaunchConfiguration(
    #     'output_folder',
    #     default='output_data'
    # )
               
    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'output_path',
            default_value=logger_dir,
            description='output_folder path'
        ),

        # launch.actions.DeclareLaunchArgument(
        #     'output_folder',
        #     default_value='output_data',
        #     description='The name of the directory with the collected data'
        # ),

        # launh Logger node
        Node(
            package='logger',
            executable='logger',
            name='logger',
            output='screen',
            emulate_tty=True,
            parameters=[
                {"output_path": output_path},
                # {"output_folder": output_folder},
                {"control_topic": "/cmd_vel"},
                {"tf_topic": "/tf"},
                {"parent_frame": "odom"},
                {"robot_frame": "base_link"},
                {"kinetic_model_frame": "model_link"},
                {"nn_model_frame": "nn_model_link"}
            ]
        ),

        # launch Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
        )
    ])