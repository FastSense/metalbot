import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # declare default parameters for control_generator
    defaut_pub_rate = "30"
    defaut_Tmax = "10"
    defaut_period_lin = "5"
    defaut_period_ang = "5"
    defaut_v_min = "0.0"
    defaut_v_max = "1.5"
    defaut_w_min = "0.0"
    defaut_w_max = "2.5"
    defaut_a_lin = "0.25"
    defaut_a_ang = "0.25"

    pub_rate = launch.substitutions.LaunchConfiguration(
        'pub_rate',
        default=defaut_pub_rate
    )
    Tmax = launch.substitutions.LaunchConfiguration(
        'Tmax',
        default=defaut_Tmax
    )
    period_lin = launch.substitutions.LaunchConfiguration(
        'period_lin',
        default=defaut_period_lin
    ) 
    period_ang = launch.substitutions.LaunchConfiguration(
        'period_ang',
        default=defaut_period_ang
    )
    v_min = launch.substitutions.LaunchConfiguration(
        'v_min',
        default=defaut_v_min
    )
    v_max = launch.substitutions.LaunchConfiguration(
        'v_max',
        default=defaut_v_max
    )
    w_min = launch.substitutions.LaunchConfiguration(
        'w_min',
        default=defaut_w_min
    )
    w_max = launch.substitutions.LaunchConfiguration(
        'w_max',
        default=defaut_w_max
    )
    a_lin = launch.substitutions.LaunchConfiguration(
        'a_lin',
        default=defaut_a_lin
    )
    a_ang = launch.substitutions.LaunchConfiguration(
        'a_ang',
        default=defaut_a_ang
    )

    # decalre default parameters for logger
    output_dir = os.path.join(os.getcwd(), 'src/logger/output_data/') 
    output_path = launch.substitutions.LaunchConfiguration(
        'output_path',
        default=output_dir
    )

    # Include rosbot gazebo sim
    rosbot_description_dir = get_package_share_directory('rosbot_description')  
    rosbot_sim_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    rosbot_description_dir + '/launch/rosbot_sim.launch.py'
            ),
            launch_arguments = {'gui': 'true'}.items()
    )

    # Include control generator
    rosbot_controller_dir = get_package_share_directory('rosbot_controller')  
    control_gen_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    rosbot_controller_dir + '/control_gen.launch.py'
            ),
            launch_arguments = {
                'pub_rate': pub_rate, 
                "Tmax":Tmax,
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

    # Include logger
    logger_dir = get_package_share_directory('logger')
    logger_launch = launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                    logger_dir + '/logger.launch.py'
            ),
            launch_arguments = {'output_path': output_path}.items()
    )

    # launch Rviz2
    # Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='log',
    # ),

    return LaunchDescription([

        launch.actions.DeclareLaunchArgument(
            'output_path',
            default_value=output_dir,
            description='Logger node output dir'
        ),

        launch.actions.DeclareLaunchArgument(
            'pub_rate',
            default_value=defaut_pub_rate,
            description='Control publication frequency'
        ),

        launch.actions.DeclareLaunchArgument(
            'Tmax',
            default_value=defaut_Tmax,
            description='control generator running time'
        ),

        launch.actions.DeclareLaunchArgument(
            'period_lin',
            default_value=defaut_period_lin,
            description='Linear velocity change period'
        ),

        launch.actions.DeclareLaunchArgument(
            'period_ang',
            default_value=defaut_period_ang,
            description='Angular velocity change period'
        ),

        launch.actions.DeclareLaunchArgument(
            'v_min',
            default_value=defaut_v_min,
            description='Minimum linear speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'v_max',
            default_value=defaut_v_max,
            description='Maximum linear speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'w_min',
            default_value=defaut_w_min,
            description='Minimum angular speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'w_max',
            default_value=defaut_w_max,
            description='Maximum angular speed'
        ),

        launch.actions.DeclareLaunchArgument(
            'a_lin',
            default_value=defaut_a_lin,
            description='Linear acceleration'
        ),

        launch.actions.DeclareLaunchArgument(
            'a_ang',
            default_value=defaut_a_ang,
            description='Angular acceleration'
        ),


        # rosbot_sim_launch,
        control_gen_launch,
        logger_launch

    ]) 