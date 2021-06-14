import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    update_rate = '10'
    rosbot_update_rate = launch.substitutions.LaunchConfiguration(
        'rosbot_update_rate',
        default=update_rate
    )

    # print("update_rate_new = {}".format(update_rate_new))
                  
    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'rosbot_update_rate',
            default_value='10',
            description='update_rate'
        ),

        # launh Logger node
        Node(
            package='rosbot_controller',
            executable='spawn_rosbot',
            name='spawn_rosbot',
            output='screen',
            parameters=[
                {"rosbot_update_rate":rosbot_update_rate},
            ],
            # arguments=[rosbot_update_rate]
        ),

    ])