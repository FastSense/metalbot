import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # declare default parameters for control_generator
    defaut_pub_rate = "30"
    defaut_max_time = "10"
    defaut_period_lin = "5"
    defaut_period_ang = "5"
    defaut_v_min = "0.0"
    defaut_v_max = "1.5"
    defaut_w_min = "0.0"
    defaut_w_max = "2.5"
    defaut_a_lin = "0.25"
    defaut_a_ang = "0.25"

    pub_rate = LaunchConfiguration('pub_rate', default=defaut_pub_rate)
    max_time = LaunchConfiguration('Tmax', default=defaut_max_time)
    period_lin = LaunchConfiguration('period_lin', default=defaut_period_lin) 
    period_ang = LaunchConfiguration('period_ang', default=defaut_period_ang)
    v_min = LaunchConfiguration('v_min', default=defaut_v_min)
    v_max = LaunchConfiguration('v_max', default=defaut_v_max)
    w_min = LaunchConfiguration('w_min', default=defaut_w_min)
    w_max = LaunchConfiguration('w_max', default=defaut_w_max)
    a_lin = LaunchConfiguration('a_lin', default=defaut_a_lin)
    a_ang = LaunchConfiguration('a_ang', default=defaut_a_ang)

    # Include control generator
    rosbot_controller_dir = get_package_share_directory('rosbot_controller')  
    control_gen_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(rosbot_controller_dir + '/control_gen.launch.py'),
            launch_arguments = {
                'pub_rate': pub_rate, 
                "Tmax":max_time,
                "period_lin": period_lin,
                "period_ang": period_ang,
                "v_min": v_min,
                "v_max": v_max,
                "w_min": w_min,
                "w_max": w_max,
                "a_lin": a_lin,
                "a_ang": a_ang
            }.items()
    )

    # decalre default parameters for logger
    """
    I can't use get_package_share_directory, cause it will return
    'ros2_ws/install/logger/share/logger', BUT when using 'ros2 launch' 
    ROS launches from its workspace so I can easy get desired directory
    
    * I checked it works well when launch from any directory *
    """
    output_dir = os.path.join(os.getcwd(), 'src/logger/output_data/') 
    output_path = launch.substitutions.LaunchConfiguration('output_path', default=output_dir)

    # Include logger
    logger_dir = get_package_share_directory('logger')
    logger_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(logger_dir + '/logger.launch.py'),
            launch_arguments = {'output_path': output_path}.items()
    )

    return LaunchDescription([

        DeclareLaunchArgument('output_path', default_value=output_dir, description='Logger node output dir'),
        DeclareLaunchArgument('pub_rate', default_value=defaut_pub_rate, description='Control publication frequency'),
        DeclareLaunchArgument('Tmax', default_value=defaut_max_time, description='control generator running time'),
        DeclareLaunchArgument('period_lin', default_value=defaut_period_lin, description='Linear velocity change period'),
        DeclareLaunchArgument('period_ang', default_value=defaut_period_ang, description='Angular velocity change period'),
        DeclareLaunchArgument('v_min', default_value=defaut_v_min, description='Minimum linear speed'),
        DeclareLaunchArgument('v_max', default_value=defaut_v_max, description='Maximum linear speed'),
        DeclareLaunchArgument('w_min', default_value=defaut_w_min, description='Minimum angular speed'),
        DeclareLaunchArgument('w_max', default_value=defaut_w_max, description='Maximum angular speed'),
        DeclareLaunchArgument('a_lin', default_value=defaut_a_lin, description='Linear acceleration'),
        DeclareLaunchArgument('a_ang', default_value=defaut_a_ang, description='Angular acceleration'),

        control_gen_launch,
        logger_launch

    ]) 
