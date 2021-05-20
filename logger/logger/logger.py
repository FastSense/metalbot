import rclpy
from rclpy.node import Node

import os
import pandas as pd

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from logger.utils import euler_from_quaternion, convert_ros2_time_to_float
from logger.create_graphs import build_general_graph_for_rosbot

class Logger(Node):
    """
    Class for logging the state of the rosbot
    """
    def __init__(self):
        """

        """
        super().__init__('logger')
      
        self.init_parameters()
        self.get_parametes()
        self.init_subs()
        self.init_containers()
        
        self.frist_tick = True
        self.init_time = None
        self.prev_tf_callback_time = None
        self.curr_control = list()

        rclpy.get_default_context().on_shutdown(self.on_shutdown)
       

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameter('output_path', "")
        self.declare_parameter('output_folder', 'output_data')
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('kinetic_model_frame', 'model_link')
        self.declare_parameter('nn_model_frame', 'nn_model_link')

    def get_parametes(self):
        """
        Gets node parameters
        """
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        # self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.tf_topic = self.get_parameter('tf_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.kinetic_model_frame = self.get_parameter('kinetic_model_frame').get_parameter_value().string_value
        self.nn_model_frame = self.get_parameter('nn_model_frame').get_parameter_value().string_value

    def init_containers(self):
        """
        Declares containers for logged data
        """
        self.robot_state = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z',
            ]
        )
        self.kinetic_model_state = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z',
            ]
        )
        self.nn_model_state = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z',
            ]
        )
        self.robot_control = pd.DataFrame(
            columns=[
                'x', 'yaw'
            ]
        )
        self.time = list()

    def init_subs(self):
        """
        Declares node subscribers
        """
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            1
        )
        self.control_sub = self.create_subscription(
            Twist,
            self.control_topic,
            self.control_callback,
            1
        )

        # prevent unused variable warning
        self.control_sub
        self.odom_sub

    def odom_callback(self, odom_msg):
        """
        Args:
            :odom_msg: (Odometry): odom msg from 
        """

        if (len(self.curr_control) == 0):
            return  

        curr_time = convert_ros2_time_to_float(
            self.get_clock().now().seconds_nanoseconds()
        )   
        # update time container
        self.time.append(curr_time - self.init_time)
        # update control container
        self.robot_control.loc[len(self.robot_control)] = self.curr_control
        # update robot_state container
        rosbot_pose = odom_msg.pose.pose
        rosbot_velocities = odom_msg.twist.twist
        x, y, z = rosbot_pose.position.x, rosbot_pose.position.y, rosbot_pose.position.z
        rpy = euler_from_quaternion(
            rosbot_pose.orientation.x,
            rosbot_pose.orientation.y,
            rosbot_pose.orientation.z,
            rosbot_pose.orientation.w
        )

        v_x = rosbot_velocities.linear.x # Linear velocity
        v_y = rosbot_velocities.linear.y
        v_z = rosbot_velocities.linear.z

        w_x = rosbot_velocities.angular.x
        w_y = rosbot_velocities.angular.y
        w_z = rosbot_velocities.angular.z # YAW velocity

        last_row = len(self.robot_state)
        self.robot_state.loc[last_row] = [x,y,z] + rpy + [v_x, v_y, v_z, w_x, w_y, w_z]

    def control_callback(self, control):
        """

        """
        if self.frist_tick:
            self.frist_tick = False
            self.init_time = convert_ros2_time_to_float(
                self.get_clock().now().seconds_nanoseconds()
            )
            self.prev_tf_callback_time = self.init_time

        self.curr_control = [control.linear.x, control.angular.z]

    def save_collected_data_to_csv(self):
        """

        """
        if not os.path.exists(self.output_path):
            os.makedirs(self.output_path)
        self.robot_state.to_csv(
            path_or_buf=os.path.join(self.output_path, "rosbot_state.csv"),
            sep=' ',
            index=False
        )

        self.kinetic_model_state.to_csv(
            path_or_buf=os.path.join(self.output_path, "kinematic_model_state.csv"),
            sep=' ',
            index=False
        )

        self.nn_model_state.to_csv(
            path_or_buf=os.path.join(self.output_path, "nn_model_state.csv"),
            sep=' ',
            index=False
        )

        self.robot_control.to_csv(
            path_or_buf= os.path.join(self.output_path,"control.csv"),
            sep=' ',
            index=False
        )

        pd.DataFrame(data=self.time, columns=['t']).to_csv(
            path_or_buf= os.path.join(self.output_path, "time.csv"),
            sep=' ',
            index=False
        )

    def on_shutdown(self):
        """
        
        """
        build_general_graph_for_rosbot(
            robot_state_df=self.robot_state,
            control_df=self.robot_control,
            time_list=self.time
        )
        self.save_collected_data_to_csv()

        self.get_logger().warn("Output path = {}".format(self.output_path))

def main():
    """

    """
    rclpy.init()

    logger = Logger()

    try:
        rclpy.spin(logger)
    except:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



