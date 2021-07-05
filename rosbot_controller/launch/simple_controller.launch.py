from launch import LaunchDescription
import launch
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rosbot_controller = get_package_share_directory('rosbot_controller')

    path_publisher = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            rosbot_controller + '/path_publisher.launch.py')
    )
    path_follower = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            rosbot_controller + '/path_follower.launch.py')
    )

    return LaunchDescription([
        path_follower,
        path_publisher,
    ])


if __name__ == '__main__':
    generate_launch_description()
