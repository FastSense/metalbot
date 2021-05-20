import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    output_dir = os.path.join(os.getcwd(), 'src/logger/output_data/') 
    rosbot_description_dir = get_package_share_directory('rosbot_description')
    
    rosbot_sim_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    rosbot_description_dir + '/launch/rosbot_sim.launch.py'))


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
                {"nn_model_frame": "nn_model_link"}
            ]
        ),

        # launch Rviz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='log',
        ),

        rosbot_sim_launch
    ])