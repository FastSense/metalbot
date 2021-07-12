from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    default_control_topic = "/cmd_vel"
    default_parent_frame = "odom"
    default_model_type = "kinematic"
    default_cmd_freq = "60.0"
    default_nn_model_path = "http://192.168.194.51:8345/ml-control/gz-rosbot/rosbot_sim_gazebo11.onnx"

    control_topic = LaunchConfiguration("control_topic", default=default_control_topic)
    parent_frame = LaunchConfiguration("parent_frame", default=default_parent_frame)
    model_type = LaunchConfiguration("model_type", default=default_model_type)
    cmd_freq = LaunchConfiguration("cmd_freq", default=default_cmd_freq)
    nn_model_path = LaunchConfiguration("nn_model_path", default=default_nn_model_path)
    
    return LaunchDescription([

        DeclareLaunchArgument('control_topic', default_value=default_control_topic, description='Topic from which we read a control information'),
        DeclareLaunchArgument('parent_frame', default_value=default_parent_frame,description='Parent frame'),
        DeclareLaunchArgument('model_type', default_value=default_model_type, description='Type of model: kinematic / nn_model'),
        DeclareLaunchArgument('cmd_freq', default_value=default_cmd_freq, description='Frequency of an updating state of a control of a rosbot'),
        DeclareLaunchArgument('nn_model_path', default_value=default_nn_model_path, description='Path to a weights of neural network'),
        
        Node(
            package='rosbot_controller',
            executable='model_runner',
            name='model_runner',
            output='screen',
            parameters=[
                {"control_topic": control_topic},
                {"parent_frame": parent_frame},
                {"model_type": model_type},
                {"cmd_freq": cmd_freq},
                {"nn_model_path": nn_model_path}
            ]
        ),

    ])
