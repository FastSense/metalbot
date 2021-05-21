import time
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist


class ControlGenerator(Node):
    """
    """
    def __init__(self):
        """

        """
        super().__init__('control_generator')

        self.init_parameters()
        self.get_parametes()

        self.cmd_pub = self.create_publisher(Twist, self.control_topic, 10)

        timer_period = 0.033  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        """
        print(time.time())
    
    def wait_for_subs(self):
        """

        """
        # TODO tyr ROS2 sleep
        # loop_rate = self.create_rate(0.1, self.get_clock())
        subs = self.count_subscribers(self.control_topic)
        while subs < self.num_of_subs:
            # loop_rate.sleep()
            self.get_logger().warn(
                "Wait for subscribers, current num of subscribers {} / {}".format(subs, self.num_of_subs)
            )
            time.sleep(1)
            subs = self.count_subscribers(self.control_topic)

    def run(self):
        """
        """
        self.get_logger().info("Start")
        self.wait_for_subs()

        if self.mode == 'periodic':
            self.v_seq, self.w_seq = self.generate_control_seq()
        elif self.mode == "from_file":
            self.v_seq, self.w_seq = self.parse_control_from_file()

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameter('mode', 'periodic')  # periodic / from_file
        self.declare_parameter('num_of_subs', 1)
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('pub_rate', 30)
        # periodic
        self.declare_parameter('Tmax', 10)          
        self.declare_parameter('period_lin', 5)
        self.declare_parameter('period_ang', 5)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('w_min', 0.0)
        self.declare_parameter('w_max', 2.5)
        self.declare_parameter('a_lin', 0.5)
        self.declare_parameter('a_ang', 0.5)
        # from_file
        self.declare_parameter('file_path', "")

    def get_parametes(self):
        """
        Gets node parameters
        """
        self.mode = self.get_parameter('mode').get_parameter_value().string_value
        self.num_of_subs = self.get_parameter('num_of_subs').get_parameter_value().integer_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        pub_rate = self.get_parameter('pub_rate').get_parameter_value().integer_value
        
        # periodic
        Tmax_sec = self.get_parameter('Tmax').get_parameter_value().integer_value
        period_lin_sec = self.get_parameter('period_lin').get_parameter_value().integer_value
        period_ang_sec = self.get_parameter('period_ang').get_parameter_value().integer_value
        self.v_min = self.get_parameter('v_min').get_parameter_value().double_value
        self.v_max = self.get_parameter('v_max').get_parameter_value().double_value
        self.w_min = self.get_parameter('w_min').get_parameter_value().double_value
        self.w_max = self.get_parameter('w_max').get_parameter_value().double_value
        self.a_lin = self.get_parameter('a_lin').get_parameter_value().double_value
        self.a_ang = self.get_parameter('a_ang').get_parameter_value().double_value
        # convert Hertz to seconds
        self.dt = round(1 / pub_rate, 4)
        # convert seconds to iterations
        self.Tmax_it = int(Tmax_sec / self.dt)
        self.period_lin_it = int(period_lin_sec / self.dt)
        self.period_ang_it = int(period_ang_sec / self.dt)

        # from_file
        self.file_path = self.get_parameter('file_path').get_parameter_value().string_value

    def generate_control_seq():
        """
        """
        pass

    def parse_control_from_file():
        """
        """


def main():
    """

    """
    rclpy.init()
    control_gen = ControlGenerator()
    control_gen.run()
    try:
        rclpy.spin(control_gen)
    except:
        pass

    control_gen.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()