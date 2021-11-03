from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess


import os

def generate_launch_description():
    model_pkg = get_package_share_directory('metalbot_gazebo')

    # Model
    model_path = os.path.join(model_pkg, 'models', 'metalbot.sdf')
    model_file = open(model_path, 'r').read()
    model_file = model_file.replace('"', '\\"')
    model = '{name: \"robot\", xml: \"'  +  model_file + '\" }'

    spawn_service_call = ExecuteProcess(cmd=['ros2', 'service', 'call', '/spawn_entity', 'gazebo_msgs/SpawnEntity', model], 
            output='screen')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),

        spawn_service_call
    ])
