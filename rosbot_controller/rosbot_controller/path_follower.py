import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path


from rosbot_controller.rosbot import Rosbot, RobotState, RobotControl, Goal
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
import numpy as np


class TrajFollower():
    """
    TrajFollower tries to follow given path 
    Trajectory should be defined as array of (x, y, quaternion) points
    It computes control to follow the path
    """

    def __init__(self, node_name) -> None:
        self.node_name = node_name
        rclpy.init(args=None)
        self.node = rclpy.create_node(node_name)
        self.odom_frame = "odom"
        self.robot_frame = "base_link"

        self.robot = Rosbot()
        self.robot_state = RobotState()  # обновляемое положение робота
        self.current_goal = Goal()

        self.cmd_freq = 30.0
        self.dt = 1.0 / self.cmd_freq

        self.goal_queue = []
        self.path = []

        self.path_index = 0
        self.got_path = False

        self.start_receive_path = False  # сигнал вывода получения пути

        self.init_subs_pubs()
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def init_subs_pubs(self):
        """
        Initializing subsciption and publisher
        """
        # паблишер для управления роботом
        self.cmd_pub = self.node.create_publisher(
            Twist, '/cmd_vel', 1)
        # подписчик для получения траектории
        self.path_sub = self.node.create_subscription(
            Path, '/path', self.path_callback, 10)

        # подписчик для положения робота
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # на всякий случай
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
        # обновили состояние робота
        self.robot_state = RobotState(x, y, yaw)

    def path_callback(self, msg: Path):
        """
        Receiving message with coordinates of path to follow
        """
        for p in msg.poses:
            x, y = p.pose.position.x, p.pose.position.y
            self.goal_queue.append(Goal(x, y))
            self.path.append((x, y))

        self.got_path = True

    def publish_control(self, control: RobotControl):
        """
        :param control: control vector of RobotControl type
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

        self.timer = self.node.create_timer(
            self.dt, self.calculate_publish_control)
        print("start spin")
        rclpy.spin(self.node)
        print("end spin")

    def calculate_publish_control(self):
        """
        Calculate and publish control 
        """

        # если путь ещё не получен, то не выполняем
        if not self.got_path:
            if not self.start_receive_path:
                print("Waiting for coordinates of path")
                self.start_receive_path = True
            return

        # состояние обновляется в odom_callback
        print("From run. My state: " + self.robot_state.to_str())

        self.robot.set_state(self.robot_state)

        if self.robot.goal_reached(self.current_goal):
            if self.goal_queue:
                self.current_goal = self.goal_queue.pop(0)
                self.path_index += 1
            else:
                # конец пути, тормозим
                self.publish_control(RobotControl())
                print("We got it")
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
