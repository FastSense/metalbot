from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pathlib
import numpy as np
import rclpy
import math
import time
from scipy.spatial.transform import Rotation


class TrajPublish():
    """
    TrajPublish creates a message of a type Path(), and publish it in path_topic.
    If path stores in the file, it should be defines as array of (x, y).

    Args:
        node_name: name of the node.
    Args of a command line:
        traj_type:   type of a trajectory (default = 1.0sin1.0; 1.0spiral, polygon, from_file).
        move_plan:   path to a file which stores a path (default = "").
        num_of_subs: number of subcribers to the path_topic which is necessary
                     to start publishing a message (default = 1).
        path_topic:  name of the path topic (default = /path).

    """

    def __init__(self, node_name):
        self.node_name = node_name
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.node_name)
        self.node.declare_parameter('traj_type')
        self.node.declare_parameter('move_plan')
        self.node.declare_parameter('num_of_subs', 1)
        self.node.declare_parameter('path_topic', '/path')

        self.path_topic = self.node.get_parameter(
            'path_topic').get_parameter_value().string_value
        self.traj_type = self.node.get_parameter(
            'traj_type').get_parameter_value().string_value
        self.move_plan = self.node.get_parameter(
            'move_plan').get_parameter_value().string_value
        self.num_of_subs = self.node.get_parameter(
            'num_of_subs').get_parameter_value().double_value

        self.path_pub = None

        # create empty message
        self.msg = Path()
        self.msg.header.frame_id = "odom"
        self.msg.poses = []
        self.step = 0.1  # step for generating the trajectory of sinus and spiral

    def generate_message(self):
        """
        Generating a message depending on its type

        """
        if "sin" in self.traj_type:
            amplitude, freq, reverse = self.parse_sin_traj()
            self.SinTrajGenerator(amplitude, freq, reverse)
        elif "spiral" in self.traj_type:
            amplitude = self.parse_spiral_traj()
            self.SpiralTrajGenerator(amplitude)
        elif self.traj_type == "polygon":
            self.PolygonTrajGenerator()
        elif self.traj_type == "from_file":
            self.FromFileTrajGenerator()

    def run(self):
        """
        Main function. Check the correctness of a type_traj, generate message
        and publish it in topic

        """
        if not self.is_valid_traj_type():
            self.node.get_logger().info("Not valid type of trajectory")
            return 1

        self.path_pub = self.node.create_publisher(Path, self.path_topic, 5)
        self.generate_message()

        # waiting for subs on our channel
        while self.path_pub.get_subscription_count() < self.num_of_subs:
            time.sleep(0.1)
        time.sleep(0.1)

        self.path_pub.publish(self.msg)

        time.sleep(0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def is_valid_traj_type(self):
        """
        Checking the validity of a trajectory type

        """
        return 'sin' in self.traj_type or self.traj_type in ('from_file', 'polygon')\
            or 'spiral' in self.traj_type

    def parse_plan(self):
        """
        Parse path from file
        Return:
            edges: list of (x, y) - points of trajectory

        """
        edges = list()
        if self.move_plan == '':
            print("Error file_name")
            return 1
        with (pathlib.Path(self.move_plan)).open() as f:
            for line in f:
                p = line.replace('\n', '')
                if p:
                    p = p.split(" ")
                    edges.append((float(p[0]), float(p[1])))

        return edges

    def edges_to_points(self, edges):
        """
        Creates array of points between the edges
        Args:
            edges: list of (x, y) - points of trajectory
        Return:
            points: big array of points - splitting edges into small edges

        """
        points = list()
        p1 = (0.0, 0.0)

        for edge in edges:
            p2 = edge
            k = (p2[1] - p1[1]) / (p2[0] - p1[0])
            b = (p1[1]*p2[0] - p2[1]*p1[0]) / (p2[0] - p1[0])
            x = p1[0]
            y = k*x + b
            points.append((x, y))
            step = abs(p2[0] - p1[0])/10
            if p2[0] > p1[0]:
                while (x < p2[0]):
                    x += step
                    if (x > p2[0]):
                        break
                    y = k*x + b
                    points.append((x, y))
            else:
                while (x > p2[0]):
                    x -= step
                    if (x < p2[0]):
                        break
                    y = k*x + b
                    points.append((x, y))
            p1 = p2

        return points

    def FromFileTrajGenerator(self):
        """
        Generating a trajectory from file

        """
        if self.move_plan is None:
            print("Move plan was not specified")
            return 1

        edges = self.parse_plan()
        points = self.edges_to_points(edges)

        for p in points:
            ps = PoseStamped()
            ps.header = self.msg.header
            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = 0.0
            self.msg.poses.append(ps)

    def SinTrajGenerator(self, a=1.0, f=1.0, reverse=False):
        """
        Generating a sinus trajectory (a*sin(f*t))
        Args:
            a: amplitude of a sinus
            f: phase of a sinus

        """
        K = -1 if reverse == True else 1
        x_ar = np.arange(0, 2*np.pi * K, self.step * K,
                         dtype=float)   # start,stop,step
        y_ar = float(a) * np.sin(float(f) * x_ar)
        yaw_arr = float(a) * float(f) * np.cos(float(f) * x_ar)

        for i in range(len(x_ar)):
            ps = PoseStamped()
            ps.header = self.msg.header
            ps.pose.position.x = x_ar[i]
            ps.pose.position.y = y_ar[i]
            ps.pose.position.z = 0.0
            goal_quaternion = list(Rotation.from_euler(
                'z', math.atan(yaw_arr[i]), degrees=False
            ).as_quat())
            ps.pose.orientation.x = goal_quaternion[0]
            ps.pose.orientation.y = goal_quaternion[1]
            ps.pose.orientation.z = goal_quaternion[2]
            ps.pose.orientation.w = goal_quaternion[3]
            self.msg.poses.append(ps)

    def PolygonTrajGenerator(self):
        """
        Generating a trajectory of the square

        """
        p_edges = [(2.0, -0.1), (2.1, 1.9),  (0.1, 2.0), (0, 0)]  # square
        points = self.edges_to_points(p_edges)

        for p in points:
            ps = PoseStamped()
            ps.header = self.msg.header

            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = 0.0
            goal_quaternion = list(Rotation.from_euler(
                'z', math.atan2(p[1], p[0]), degrees=False
            ).as_quat())
            ps.pose.orientation.x = goal_quaternion[0]
            ps.pose.orientation.y = goal_quaternion[1]
            ps.pose.orientation.z = goal_quaternion[2]
            ps.pose.orientation.w = goal_quaternion[3]
            self.msg.poses.append(ps)

    def SpiralTrajGenerator(self, amplitude):
        """
        Generating a spiral trajectory
        Args:
            amplitude: amplitude of spiral

        """
        key_points = []
        if amplitude > 0:
            k = 1
        else:
            k = -1
        amplitude = abs(amplitude)
        f = 0
        while 1:
            r = self.step * math.exp(f*0.1)
            x = k * r * math.cos(f)
            y = r * math.sin(f)
            if abs(x) > amplitude or abs(y) > amplitude:
                points = self.edges_to_points(key_points)
                for p in points:
                    ps = PoseStamped()
                    ps.header = self.msg.header
                    ps.pose.position.x = p[0]
                    ps.pose.position.y = p[1]
                    ps.pose.position.z = 0.0
                    self.msg.poses.append(ps)
                return
            else:
                key_points.append([x, y])
                f += 0.1

    def parse_sin_traj(self):
        """
        Parsing sinus trajectory

        """
        traj_type = self.traj_type.strip().split('sin')  # []
        period = float(traj_type[-1])
        amplitude = float(traj_type[0].split("_")[-1])
        reverse = traj_type[0].split("_")[0] == 'reverse'
        return amplitude, period, reverse

    def parse_spiral_traj(self):
        """
        Parsing spiral trajectory

        """
        coef = self.traj_type.split('spiral')
        amp = coef[0] if coef[0] != '' or coef[0] is None else 1.0
        return float(amp)


def main():
    traj_publ = TrajPublish("path_pub")
    traj_publ.run()
    return


if __name__ == '__main__':
    main()
