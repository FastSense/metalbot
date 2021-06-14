from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()
    
    state_estimation_2d = Node(
        package="state_estimation_2d",
        executable="state_estimation_2d"
    )
    ld.add_action(state_estimation_2d)

    return ld