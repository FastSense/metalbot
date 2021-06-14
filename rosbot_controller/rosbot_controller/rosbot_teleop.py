import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


class RosbotTeleop(Node):
    """
    """
    def __init__(self):
        """
        """
        super().__init__('rosbot_teleop')
        self.curr_cmd = Twist()
        self.init_parameters()
        self.get_node_parametes()
        self.init_subs()
        self.keyboard_sub = self.create_subscription(
            String,
            self.keyboard_topic,
            self.keyboard_callback,
            1
        )
        self.cmd_pub = self.create_publisher(Twist, self.control_topic, 10)


    def init_subs(self):
        """
        """
        self.keyboard_sub = self.create_subscription(
            String,
            self.keyboard_topic,
            self.keyboard_callback,
            1
        )

        self.joystick_sub = self.create_subscription(
            Joy,
            self.joystick_topic,
            self.joystick_callback,
            1
        )


    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameters(
            namespace="",
            parameters=[
                ('update_rate', 20),
                ('keyboard_topic', "/keyboard"),
                ('control_topic', "/cmd_vel"),
                ('joystick_topic', "/joy"),
                ('movable_camera', False),
                ('v_limit', 0.5),
                ('w_limit', 2.5),
                ('lin_a', 0.1),
                ('ang_a', 0.5),
            ]
        )

    def get_node_parametes(self):
        """
        Gets node parameters
        """
        update_rate = self.get_parameter('update_rate').get_parameter_value().integer_value
        self.keyboard_topic = self.get_parameter('keyboard_topic').get_parameter_value().string_value
        self.control_topic = self.get_parameter('control_topic').get_parameter_value().string_value
        self.joystick_topic = self.get_parameter('joystick_topic').get_parameter_value().string_value
        self.movable_camera = self.get_parameter('movable_camera').get_parameter_value().bool_value
        self.v_limit = float(self.get_parameter('v_limit').get_parameter_value().string_value)
        self.w_limit = float(self.get_parameter('w_limit').get_parameter_value().string_value)
        self.lin_a = float(self.get_parameter('lin_a').get_parameter_value().string_value)
        self.ang_a = float(self.get_parameter('ang_a').get_parameter_value().string_value)
        self.dt = 1 / update_rate

    def keyboard_callback(self, keyboard_msg):
        """
        """
        # print(keyboard_msg.data)

        keys = keyboard_msg.data.split(" ")

        lin_vel = self.curr_cmd.linear.x
        ang_vel = self.curr_cmd.angular.z
        k = 1
        if 'shift' in keys:
            k = 2

        if 'w' in keys:    # forward
            lin_vel += self.lin_a * self.dt  * k
        elif 's' in keys:  # backward
            lin_vel -= self.lin_a * self.dt  * k
        else:
            lin_vel /= 1.5
            lin_vel = 0.0 if abs(lin_vel) < 0.001 else lin_vel

        if 'a' in keys:    # left
            ang_vel = 0.0 if ang_vel < 0.0 else ang_vel
            ang_vel += self.ang_a * self.dt  * k
        elif 'd' in keys:  # right
            ang_vel = 0.0 if ang_vel > 0.0 else ang_vel
            ang_vel -= self.ang_a * self.dt  * k
        else:
            ang_vel /= 1.5
            ang_vel = 0.0 if abs(ang_vel) < 0.001 else ang_vel

        if 'space' in keys:
            lin_vel = 0
            ang_vel = 0

        # print(self.movable_camera, self.v_limit, self.w_limit, self.lin_a, self.ang_a, self.dt)
        lin_vel, ang_vel = self.clip_velocities(lin_vel, ang_vel)

        # print(lin_vel, ang_vel)
        self.curr_cmd.linear.x = lin_vel
        self.curr_cmd.angular.z = ang_vel
        self.cmd_pub.publish(self.curr_cmd)


    # (c)pizheno https://github.com/FastSense/tankbot-rc/blob/refactoring_to_class/tankbot_joystick.py#L54
    def joystick_callback(self, msg):
        """
        Callback for joystick input
        """

        # Reducing sensitivity of angular velocity control by joystick
        JOYSTICK_ANGULAR_SCALER = 0.6
        JOYSTICK_AXIS_LINEAR_VEL = 2     # 1
        JOYSTICK_AXIS_ANGULAR_VEL = 1    # 0
        JOYSTICK_AXIS_HEAD_YAW = 3       # 2
        JOYSTICK_AXIS_HEAD_PITCH = 0     # 3

        # Linear velocity axis
        # lin = msg.axes[1]
        lin_vel = -msg.axes[JOYSTICK_AXIS_LINEAR_VEL]
        # Angular velocity axis
        # ang = msg.axes[0] * JOYSTICK_ANGULAR_SCALER
        ang_vel = msg.axes[JOYSTICK_AXIS_ANGULAR_VEL] * JOYSTICK_ANGULAR_SCALER

        # Exponential scaling for input
        kl = 1
        ka = 1
        if lin_vel < 0:
            lin = abs(lin_vel)
            kl = -1
        if ang_vel < 0:
            ang = abs(ang_vel)
            ka = -1

        # Exponential scaling for inputs
        lin_vel = kl * (np.exp(lin) - 1) / (np.e - 1)
        ang_vel = ka * (np.exp(ang) - 1) / (np.e - 1)
        lin_vel, ang_vel = self.clip_velocities(lin_vel, ang_vel)

        self.curr_cmd.linear.x = lin_vel
        self.curr_cmd.angular.z = ang_vel
        return self.curr_cmd
        # with self.head_cmd_lock:
        #     self.head_cmd.angular.x = (msg.axes[JOYSTICK_AXIS_HEAD_YAW] + 1) / 2
        #     self.head_cmd.angular.y = (-msg.axes[JOYSTICK_AXIS_HEAD_PITCH] + 1) / 2



    def clip_velocities(self, lin_vel, ang_vel):
        """
        """

        lin_vel = np.clip(
            lin_vel,
            a_min=-self.v_limit,
            a_max=self.v_limit
        )
        ang_vel= np.clip(
            ang_vel,
            a_min=-self.w_limit, 
            a_max=self.w_limit
        )
        lin_vel = round(lin_vel, 3)
        ang_vel = round(ang_vel, 3)

        return lin_vel, ang_vel


def main():
    """
    """
    rclpy.init()
    teleop = RosbotTeleop()
    rclpy.spin(teleop)
    rclpy.shutdown()


if __name__ == '__main__':
    main()