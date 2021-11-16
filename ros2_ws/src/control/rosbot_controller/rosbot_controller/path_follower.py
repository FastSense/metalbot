import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Path
from rosbot_controller.rosbot_2D import Goal, Rosbot, RobotState, RobotControl

from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
import numpy as np


class TrajFollower(Node):
    """
    TrajFollower tries to follow given path.
    Trajectory should be defined as array of (x, y, quaternion) points.
    It computes control to follow the path.
    Args:
        _name: name of
    Args of a command line:
        control_topic: name of a control topic (default = /cmd_vel).
        v_max:         maximum forward speed.
        w_max:         maximum value of a rotation speed around axe z.
    """

    def __init__(self, _name):

        rclpy.init(args=None)

        super().__init__(_name)

        self.declare_and_get_parametrs()
        self.robot = Rosbot(self.v_max, self.w_max)
        self.current_goal = Goal()
        self.path = []
        self.dt = 1.0 / self.cmd_freq
        self.path_deviation = 0.0
        self.path_index = 0
        self.wait_for_path = True

        self.init_subs_pubs()

    def declare_and_get_parametrs(self):
        """
        Initializing of the parameters from a command line

        """
        self.declare_parameter('use_odom', True)
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('parent_frame', 'odom_frame')
        self.declare_parameter('robot_frame', 'camera_pose_frame')
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('v_max', 2.0)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('vel_coeff', 1.0)
        self.declare_parameter('ang_vel_coeff', 1.0)
        self.declare_parameter('cmd_freq', 30.0)
        self.declare_parameter('kill_follower', True)

        self.path_topic = self.get_parameter(
            'path_topic').get_parameter_value().string_value
        self.use_odom = self.get_parameter(
            'use_odom').get_parameter_value().bool_value
        self.odom_topic = self.get_parameter(
            'odom_topic').get_parameter_value().string_value
        self.tf_topic = self.get_parameter(
            'tf_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter(
            'parent_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter(
            'robot_frame').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter(
            'control_topic').get_parameter_value().string_value
        self.v_max = self.get_parameter(
            'v_max').get_parameter_value().double_value
        self.w_max = self.get_parameter(
            'w_max').get_parameter_value().double_value
        self.vel_coeff = self.get_parameter(
            'vel_coeff').get_parameter_value().double_value
        self.ang_vel_coeff = self.get_parameter(
            'ang_vel_coeff').get_parameter_value().double_value
        self.cmd_freq = self.get_parameter(
            'cmd_freq').get_parameter_value().double_value
        self.kill_follower = self.get_parameter(
            'kill_follower').get_parameter_value().bool_value

    def init_subs_pubs(self):
        """
        Initializing 2 subscriptions on /path and /odom
        and 1 publisher to a cmd_topic (\cmd_vel)

        """
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 1)
        self.path_sub = self.create_subscription(
            Path, 'path', self.path_callback, 1)

        if self.use_odom:
            self.odom_sub = self.create_subscription(
                Odometry, self.odom_topic, self.odom_callback, 1)
            self.odom_sub
        else:
            self.tf_sub_ = self.create_subscription(
                TFMessage, self.tf_topic, self.tf_callback, 1)
            self.tf_sub_
        self.path_sub

    def tf_callback(self, tf_msg):
        """
        Receiving TF transform from /tf topic.
        """
        for item in tf_msg.transforms:
            if (
                item.header.frame_id == self.parent_frame and
                item.child_frame_id == self.robot_frame
            ):
                x, y = item.transform.translation.x, item.transform.translation.y
                yaw = Rotation.from_quat([
                    np.float(item.transform.rotation.x),
                    np.float(item.transform.rotation.y),
                    np.float(item.transform.rotation.z),
                    np.float(item.transform.rotation.w)]
                ).as_euler('xyz')[2]
                self.robot.set_state(RobotState(x, y, yaw))

    def odom_callback(self, odom_msg: Odometry):
        """
        Receiving coordinates from /odom and update state of the robot
        """
        robot_pose = odom_msg.pose.pose
        x, y = robot_pose.position.x, robot_pose.position.y
        yaw = Rotation.from_quat([
            np.float(robot_pose.orientation.x),
            np.float(robot_pose.orientation.y),
            np.float(robot_pose.orientation.z),
            np.float(robot_pose.orientation.w)]
        ).as_euler('xyz')[2]
        self.robot.set_state(RobotState(x, y, yaw))

    def path_callback(self, msg: Path):
        """
        Receiving message with coordinates of a path to follow
        """
        self.wait_for_path = False
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]

    def get_min_dist_to_path(self):

        lookback_index_dist = 10
        if self.path_index >= lookback_index_dist:
            path_slice = self.path[self.path_index -
                                   lookback_index_dist: self.path_index]
        else:
            path_slice = self.path[0: self.path_index]

        min_dist = 100
        for p in path_slice:
            dist = self.robot.dist_to_goal_L2(Goal(p[0], p[1]))
            if dist < min_dist:
                min_dist = dist
        return min_dist

    def publish_control(self, control: RobotControl):
        """
        :Args:
            :control: control vector of RobotControl type
        """
        twist_cmd = Twist()
        twist_cmd.linear.x = control.v
        twist_cmd.angular.z = control.w
        self.cmd_pub.publish(twist_cmd)

    def run(self):
        """
        Create timer which call calculate_publish_control every dt seconds
        and start spinning the .
        """
        self.timer = self.create_timer(
            self.dt, self.calculate_publish_control)
        rclpy.spin(self)

    def calculate_publish_control(self):
        """
        Calculate and publish control until the robot reaches the goal

        """
        if self.wait_for_path:
            return

        if self.robot.goal_reached(self.current_goal):
            if self.path_index < len(self.path):
                x, y = self.path[self.path_index][0], self.path[self.path_index][1]
                self.current_goal = Goal(x, y)
                self.path_index += 1
            else:
                self.publish_control(RobotControl())
                print(
                    f"Trajectory finished. Path deviation = {self.path_deviation}")
                if self.kill_follower:
                    rclpy.try_shutdown()
                return

        self.path_deviation += self.get_min_dist_to_path()
        control = self.robot.calculate_contol(
            self.current_goal,
            kv=self.vel_coeff,
            kw=self.ang_vel_coeff
        )
        self.publish_control(control)
        return


def main(args=None):
    traj_follower = TrajFollower("trajectory_follower")
    traj_follower.run()


if __name__ == '__main__':
    main()
