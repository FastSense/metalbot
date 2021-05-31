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
        self.curr_index = 0
        self.start = False
        self.timer = self.create_timer(self.dt, self.pub_control)
        rclpy.get_default_context().on_shutdown(self.on_shutdown)
   
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
            self.v_seq, self.w_seq = self.generate_control_sequences()
        elif self.mode == "from_file":
            self.v_seq, self.w_seq = self.parse_control_from_file()

        self.v_seq = self.v_seq[:self.Tmax_it]
        self.w_seq = self.w_seq[:self.Tmax_it]
        # print(self.v_seq, self.w_seq )
        self.start = True
        # print("START!")

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameter('mode', 'periodic')  # periodic / from_file
        self.declare_parameter('num_of_subs', 1)
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('pub_rate', 30)
        # periodic
        self.declare_parameter('Tmax', 20)          
        self.declare_parameter('period_lin', 5)
        self.declare_parameter('period_ang', 5)
        self.declare_parameter('v_min', 0.0)
        self.declare_parameter('v_max', 1.5)
        self.declare_parameter('w_min', 0.0)
        self.declare_parameter('w_max', 2.5)
        self.declare_parameter('a_lin', 0.25)
        self.declare_parameter('a_ang', -0.25)
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

    def generate_control_sequences(self):
        """
        
        """
        v_seq, w_seq = np.array([0.0]), np.array([0.0])

        lin_iter_num = int(self.Tmax_it / self.period_lin_it*2) + 1
        ang_iter_num = int(self.Tmax_it / self.period_ang_it*2) + 1


        for _ in range(lin_iter_num):
            v_seq = np.concatenate([
                v_seq,
                self.generate_control_subsueqnces(
                    v = v_seq[-1],
                    period=self.period_lin_it,
                    acceleration=self.a_lin,
                    max=self.v_max,
                    min=self.v_min,
                    dt=self.dt
                )
            ])
            if len(v_seq) >= self.Tmax_it:
                break

        for _ in range(ang_iter_num):
            w_seq = np.concatenate([
                w_seq,
                self.generate_control_subsueqnces(
                    v = w_seq[-1],
                    period=self.period_ang_it,
                    acceleration=self.a_ang,
                    max=self.w_max,
                    min=self.w_min,
                    dt=self.dt
                )
            ])
            if len(w_seq) >= self.Tmax_it:
                break

        return v_seq, w_seq
        
    def generate_control_subsueqnces(
        self, 
        v, 
        period, 
        acceleration, 
        max, 
        min, 
        dt
    ):   
        seq=list() 
        k = -1 if acceleration < 0 else 1
        acceleration = abs(acceleration)
        for _ in range(period):
            v += acceleration * dt
            if v > max:
                v = max
            seq.append(v)

        for _ in range(period):
            v -= acceleration * dt
            if v < min:
                v = min
            seq.append(v)
        seq = np.array(seq) * k

        return seq

    def parse_control_from_file():
        """
        """
        pass

    def pub_control(self):
        """
        """
        if self.start:
            cmd_msg = Twist()
            if self.curr_index > self.Tmax_it-1:
                self.cmd_pub.publish(cmd_msg)
                self.timer.destroy()
                rclpy.try_shutdown()
            else:
                cmd_msg.linear.x = self.v_seq[self.curr_index]
                cmd_msg.angular.z = self.w_seq[self.curr_index]
                self.curr_index += 1
                self.cmd_pub.publish(cmd_msg)

    def on_shutdown(self):
        """
        A function that is executed when a node shutdown.
        """
        pass




def main():
    """

    """
    rclpy.init()
    control_gen = ControlGenerator()
    control_gen.run()
    rclpy.spin(control_gen)
 
    # control_gen.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()