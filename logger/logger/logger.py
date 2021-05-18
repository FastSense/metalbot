import rclpy
from rclpy.node import Node

import pandas as pd
import numpy as np

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

from logger.utils import euler_from_quaternion, convert_ros2_time_to_float
from logger.utils import calculate_rosbot_velocities
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

    def get_parametes(self):
        """
        Gets node parameters
        """
        self.output_path = self.get_parameter('output_path').get_parameter_value().string_value
        self.output_folder = self.get_parameter('output_folder').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.tf_topic = self.get_parameter('tf_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        self.kinetic_model_frame = self.get_parameter('kinetic_model_frame').get_parameter_value().string_value

    def init_containers(self):
        """
        Declares containers for logged data
        """
        self.robot_state = pd.DataFrame(
            columns=[
                'x', 'y', 'roll', 'pitch', 'yaw', 'v', 'w'
            ]
        )
        self.kinetic_model_state = pd.DataFrame(
            columns=[
                'x', 'y', 'roll', 'pitch', 'yaw', 'v', 'w'
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
        # TODO try use odom from gazebo
        # self.tf_sub = self.create_subscription(
        #     TFMessage,
        #     self.tf_topic,
        #     self.tf_callback,
        #     1
        # )

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
        # self.tf_sub

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
        x, y = odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y
        rpy = euler_from_quaternion(
            odom_msg.pose.pose.orientation.x,
            odom_msg.pose.pose.orientation.y,
            odom_msg.pose.pose.orientation.z,
            odom_msg.pose.pose.orientation.w
        )
        v, w = odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z
        self.robot_state.loc[len(self.robot_state)] = [x,y] + rpy + [v, w]

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
        self.robot_state.to_csv(
            path_or_buf="/home/user/ros2_ws/src/logger/test_state.csv",
            sep=' ',
            index=False
        )

        self.robot_control.to_csv(
            path_or_buf="/home/user/ros2_ws/src/logger/test_control.csv",
            sep=' ',
            index=False
        )

        pd.DataFrame(data=self.time, columns=['t']).to_csv(
            path_or_buf="/home/user/ros2_ws/src/logger/test_time.csv",
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



