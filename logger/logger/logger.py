import rclpy
from rclpy.node import Node

import pandas as pd
import numpy as np

from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage

from logger.utils import euler_from_quaternion, convert_ros2_time_to_float

class Logger(Node):
    """

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

       

    def init_parameters(self):
        """

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

        """
        self.tf_sub = self.create_subscription(
            TFMessage,
            self.tf_topic,
            self.tf_callback,
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
        self.tf_sub

    def tf_callback(self, msg):
        """
        
        """
        for transform in msg.transforms:
            if (
                transform.header.frame_id == self.parent_frame and 
                transform.child_frame_id == self.robot_frame and 
                len(self.curr_control) > 0
            ):  

                curr_time = convert_ros2_time_to_float(
                    self.get_clock().now().seconds_nanoseconds()
                )   

                # update time container
                self.time.append(curr_time - self.init_time)
                dt = curr_time - self.prev_tf_callback_time
                self.prev_tf_callback_time = curr_time

                # update control container
                self.robot_control.loc[len(self.robot_control)] = self.curr_control

                x = transform.transform.translation.x
                y = transform.transform.translation.y
                rpy = euler_from_quaternion(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )

                if len(self.robot_control) > 1:
                    # default 
                    last_row = self.robot_state.iloc[-1]
                    x_prev, y_prev = last_row['x'], last_row['y'] 
                    rpy_prev = [last_row['roll'], last_row['pitch'], last_row['yaw']]
                    v, w = caclulate_rosbot_velocities(x, y, rpy, x_prev, y_prev, rpy_prev, dt)
                else:
                    # first tf_callback
                    v, w = 0
                    pass

                self.robot_state.loc[len(self.robot_state)] = [x,y] + rpy + [v, w]


    def control_callback(self, control):
        """

        """
        if self.frist_tick:
            self.frist_tick = False
            self.init_time = convert_ros2_time_to_float(
                self.get_clock().now().seconds_nanoseconds()
            )
            # TODO test
            self.prev_tf_callback_time = self.init_time

        self.curr_control = [control.linear.x, control.angular.z]


def main():
    """

    """
    rclpy.init()

    logger = Logger()

    rclpy.spin(logger)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)

    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



