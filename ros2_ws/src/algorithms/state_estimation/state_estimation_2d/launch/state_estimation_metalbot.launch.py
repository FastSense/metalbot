from launch import LaunchDescription
from launch_ros.actions import Node
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory

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

enable_viz = 'true'

def generate_launch_description():
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'viz',
            default_value=enable_viz,
            description='enable_visualization'
        ),
        Node(
            package="state_estimation_2d",
            executable="state_estimation_2d",
            output='screen',
            emulate_tty=True,
            parameters=[
                {"use_sim_time": False},
                {"odom_topic": "velocity"},
                {"imu_topic": "imu"},
                {"imu_accel_topic": "/camera/accel/sample"},
                {"imu_gyro_topic": "/camera/gyro/sample"},
                {"cmd_topic": "cmd_vel"},
                {"odom_sim_topic": "odom_noised"},
                {"publish_topic": "odom_filtered"},
                {"imu_frame": "camera_gyro_optical_frame"},
                {"robot_base_frame": "base_link"},
                {"path_to_nn_model": "http://192.168.194.51:8345/ml-control/gz-rosbot/new_model_dynamic_batch.onnx"},
                {"time_step": 0.1},
                {"odom_sim_covariance": [0.5, 0,
                                         0, 0.1]},
                {"imu_sim_covariance": [0.5, 0,
                                         0, 0.1]},
                {"gyro_robot_covariance": 200.0},
                {"accel_robot_covariance": 200.0},
            ],
        ),
        Node(
            package="path_visualizer",
            executable="path_visualizer",
            condition=IfCondition(launch.substitutions.LaunchConfiguration("viz")),
        ),
    ])