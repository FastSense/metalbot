import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
import tf2_ros
import time
from rclpy.node import Node
import math
from RobotHelp import Rosbot, RobotState, RobotControl, Goal
from scipy.spatial.transform import Rotation
import signal
import sys
from nav_msgs.msg import Odometry
import numpy as np


def signal_handler(sig, frame):
    print('You pressed Ctrl+C!')
    sys.exit(0)


class TrajFollower():
    """
    TrajFollower tries to follow given path 
    Trajectory should be defined as array of (x, y, quaternion) points
    It computes control to follow the path
    """

    def __init__(self, node_name) -> None:
        self.node_name = node_name
        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)  # создали ноду
        self.odom_frame = "odom"
        self.robot_frame = "base_link"

        self.robot = Rosbot()
        self.robot_state = RobotState()  # обновляемое положение робота
        self.current_goal = Goal()

        self.cmd_freq = 30.0
        self.dt = 1.0 / self.cmd_freq
        # self.rate = self.node.create_rate(self.cmd_freq)

        self.goal_queue = []
        self.path = []

        self.path_index = 0
        self.got_path = False
        # паблишер для управления роботом
        self.cmd_pub = self.node.create_publisher(
            Twist, "/cmd_vel", 5)

        self.init_subs()

        # rclpy.spin_once(self.node)

        rclpy.spin(self.node)
        print("spin end")
        rclpy.get_default_context().on_shutdown(self.on_shutdown)
        # rclpy.shutdown()

    def init_subs(self):
        # подписчик для получения траектории
        self.path_sub = self.node.create_subscription(
            Path, '/path', self.path_callback, 10)

        # подписчик для положения робота
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        self.path_sub
        self.odom_sub

    def odom_callback(self, odom_msg: Odometry):
        rosbot_pose = odom_msg.pose.pose
        x, y, z = rosbot_pose.position.x, rosbot_pose.position.y, rosbot_pose.position.z
        rpy = Rotation.from_quat([
            np.float(rosbot_pose.orientation.x),
            np.float(rosbot_pose.orientation.y),
            np.float(rosbot_pose.orientation.z),
            np.float(rosbot_pose.orientation.w)]
        ).as_euler('xyz')
        yaw = rpy[2]
        # обновили состояние робота
        self.robot_state = RobotState(x, y, yaw)  
        # print(self.robot_state.to_str())

    def path_callback(self, msg: Path):  # получили сообщение

        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.goal_queue.append(Goal(x, y))
            self.path.append((x, y))

        self.got_path = True
        # print(f"self.got_path = {self.got_path}")

    def publish_control(self, control: RobotControl):
        """
        :param control: control vector of RobotControl type
        """
        twist_cmd = Twist()
        twist_cmd.linear.x = control.v
        twist_cmd.linear.y, twist_cmd.linear.z = 0.0, 0.0
        twist_cmd.angular.x, twist_cmd.angular.y = 0.0, 0.0
        twist_cmd.angular.z = control.w
        self.cmd_pub.publish(twist_cmd)

    def wait_for_path(self):
        """
        Wait for receiving coordinates of path
        """
        print("Waiting for path")
        while rclpy.ok():
            if self.got_path:
                print("Got path")
                break
            time.sleep(0.5)

    def run(self):
        t0 = self.node.get_clock().now()
        print(f"t0 = {t0}")
        # rclpy.init(args=None)
        print("start run")

        while rclpy.ok():
            # состояние обновляется в odom_callback
            print("from run: " + self.robot_state.to_str())
            if self.robot_state is None:
                print("robot_state is None")
                continue
            self.robot.set_state(self.robot_state)

            if self.robot.goal_reached(self.current_goal):
                if self.goal_queue:
                    self.current_goal = self.goal_queue.pop(0)
                    self.path_index += 1
                    time.sleep(0.5)
                else:
                    # конец пути
                    self.publish_control(RobotControl())  # тормозим
                    # self.rate.sleep()
                    break
            control = self.robot.calculate_contol(self.current_goal)
            self.publish_control(control)
            time.sleep(0.5)
            
            # self.rate.sleep()

        t1 = self.node.get_clock().now()
        print("This is the end of run")
        
        return

    def on_shutdown(self):
        """
        A function that is executed when a node shutdown.
        """
        pass


def main(args=None):
    traj_follower = TrajFollower("trajectory_follower")
    traj_follower.wait_for_path()
    traj_follower.run()


if __name__ == '__main__':
    main()
