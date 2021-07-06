import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rosbot_controller.rosbot_2D import Rosbot, RobotState, RobotControl, Goal

from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
import numpy as np


class TrajFollower():
    """
    TrajFollower tries to follow given path.
    Trajectory should be defined as array of (x, y, quaternion) points.
    It computes control to follow the path.
    Args:
        node_name: name of node
    Args of a command line:
        control_topic: name of a control topic (default = /cmd_vel).
        v_max:         maximum forward speed.
        w_max:         maximum value of a rotation speed around axe z.
    """

    def __init__(self, node_name):
        self.node_name = node_name
        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.node.declare_parameter('parent_topic', '/odom')
        self.node.declare_parameter('control_topic', '/cmd_vel')
        self.node.declare_parameter('v_max', 2.5)
        self.node.declare_parameter('w_max', 2.5)

        self.parent_topic = self.node.get_parameter(
            'parent_topic').get_parameter_value().string_value
        self.cmd_topic = self.node.get_parameter(
            'control_topic').get_parameter_value().string_value
        self.v_max = self.node.get_parameter(
            'v_max').get_parameter_value().double_value
        self.w_max = self.node.get_parameter(
            'w_max').get_parameter_value().double_value

        self.robot = Rosbot(self.v_max, self.w_max)
        # state of the Robot that updated
        self.robot_state = RobotState()
        self.current_goal = Goal()

        self.cmd_freq = 30.0
        self.dt = 1.0 / self.cmd_freq

        self.goal_queue = []
        self.path = []

        self.path_deviation = 0.0
        self.path_index = 0
        self.got_path = False

        # singal that we begin to wait for the trajectory

        self.init_subs_pubs()
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def init_subs_pubs(self):
        """
        Initializing 2 subscriptions on /path and /odom 
        and 1 publisher to a cmd_topic (\cmd_vel)

        """
        # publisher to control the robot
        self.cmd_pub = self.node.create_publisher(
            Twist, self.cmd_topic, 1)
        # subscriber to get the trajectory
        self.path_sub = self.node.create_subscription(
            Path, '/path', self.path_callback, 10)

        # subscriber for the robot position
        self.odom_sub = self.node.create_subscription(
            Odometry, self.parent_topic, self.odom_callback, 10)

        self.path_sub
        self.odom_sub

    def odom_callback(self, odom_msg: Odometry):
        """
        Receiving coordinates from /odom and update state of the robot

        """
        rosbot_pose = odom_msg.pose.pose
        x, y, z = rosbot_pose.position.x, rosbot_pose.position.y, rosbot_pose.position.z
        rpy = Rotation.from_quat([
            np.float(rosbot_pose.orientation.x),
            np.float(rosbot_pose.orientation.y),
            np.float(rosbot_pose.orientation.z),
            np.float(rosbot_pose.orientation.w)]
        ).as_euler('xyz')
        yaw = rpy[2]
        # state of the robot is updated
        self.robot_state = RobotState(x, y, yaw)

    def path_callback(self, msg: Path):
        """
        Receiving message with coordinates of a path to follow

        """
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.goal_queue.append(Goal(x, y))
            self.path.append((x, y))

        self.got_path = True

    def publish_control(self, control: RobotControl):
        """
        Args:
            control: control vector of RobotControl type

        """
        twist_cmd = Twist()
        twist_cmd.linear.x = control.v
        twist_cmd.linear.y = 0.0
        twist_cmd.linear.z = 0.0
        twist_cmd.angular.x = 0.0
        twist_cmd.angular.y = 0.0
        twist_cmd.angular.z = control.w
        self.cmd_pub.publish(twist_cmd)

    def run(self):
        """
        Create timer which call calculate_publish_control every dt seconds
        and start spinning the node.

        """
        self.timer = self.node.create_timer(
            self.dt, self.calculate_publish_control)
        rclpy.spin(self.node)


    def calculate_publish_control(self):
        """
        Calculate and publish control until the robot reaches the goal

        """
        # if the path has not yet been received, then we do not execute
        if not self.got_path:
            return


        self.robot.set_state(self.robot_state)

        if self.robot.goal_reached(self.current_goal):
            if self.goal_queue:
                self.current_goal = self.goal_queue.pop(0)
                self.path_index += 1
            else:
                # end of the path, slow down
                self.publish_control(RobotControl())
                print(f"Trajectory finished")
                rclpy.shutdown()
                return

        control = self.robot.calculate_contol(self.current_goal)
        self.publish_control(control)
        return

    def on_shutdown(self):
        """
        A function that is executed when a node shutdown.

        """
        pass


def main(args=None):

    traj_follower = TrajFollower("trajectory_follower")
    traj_follower.run()


if __name__ == '__main__':
    main()
