import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    world_file_name = 'willow_garage.world'
    default_update_rate = '10'
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    new_update_rate = LaunchConfiguration('rosbot_update_rate',default=default_update_rate)

    gazebo_ros = get_package_share_directory('gazebo_ros')
    rosbot_description = get_package_share_directory('rosbot_description')
    rosbot_controller = get_package_share_directory('rosbot_controller')

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )    
    spawn_rosbot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(rosbot_controller, 'rosbot_spawn.launch.py')),
        launch_arguments = {'rosbot_update_rate': new_update_rate}.items()
    )
    
    return LaunchDescription([

        DeclareLaunchArgument('world',
            default_value=[os.path.join(rosbot_description, 'worlds', world_file_name), ''],
            description='SDF world file'
        ),
        DeclareLaunchArgument(name='gui', default_value='true'),
        DeclareLaunchArgument(name='use_sim_time', default_value='true'),
        DeclareLaunchArgument('verbose', default_value='true', description='Set "true" to increase messages written to terminal.'),
        DeclareLaunchArgument('gdb', default_value='false', description='Set "true" to run gzserver with gdb'),
        DeclareLaunchArgument('state', default_value='true', description='Set "false" not to load "libgazebo_ros_state.so"'),
        DeclareLaunchArgument('rosbot_update_rate', default_value=default_update_rate, description='rosbot update_rate'),
        
        gazebo_server,
        gazebo_client,
        spawn_rosbot,
    ])

if __name__ == '__main__':
    generate_launch_description()
