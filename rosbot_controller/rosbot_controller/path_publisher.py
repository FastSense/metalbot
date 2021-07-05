from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import pathlib
import numpy as np
import rclpy
import math
import time


class TrajPublish():
    """

    """

    def __init__(self, node_name) -> None:
        self.node_name = node_name
        rclpy.init(args=None)
        self.node = rclpy.create_node(self.node_name)
        self.node.declare_parameter('traj_type')
        self.node.declare_parameter('move_plan')
        self.node.declare_parameter('num_of_subs', 1)

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
        self.step = 0.1

    def generate_message(self):
        """
        Generating the message to publish
        """

        print("Parse ", self.traj_type)

        if "sin" in self.traj_type:
            amplitude, freq, reverse = self.parse_sin_traj()
            self.SinTrajGenerator(amplitude, freq, reverse)
        elif "spiral" in self.traj_type:
            amplitude = self.parse_spiral_traj()
            self.SpiralTrajGenerator(amplitude)
        elif self.traj_type == "polygon":
            self.PolygonTrajGenerator()
        elif self.traj_type == "from_file":
            print(f"file_name = {self.move_plan}")
            self.FromFileTrajGenerator()

    def run(self) -> None:
        """
        Runner
        """
        if not self.IsValidTrajType():
            self.node.get_logger().info("Not valid type of trajectory")
            return 1

        self.path_pub = self.node.create_publisher(Path, "/path", 5)
        self.generate_message()

        # waiting for subs on our channel
        while self.path_pub.get_subscription_count() < self.num_of_subs:
            time.sleep(0.5)
        time.sleep(0.1)

        self.path_pub.publish(self.msg)

        time.sleep(0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def IsValidTrajType(self):
        return 'sin' in self.traj_type or self.traj_type in ('from_file', 'polygon') or 'spiral' in self.traj_type

    def parse_plan(self) -> list:
        """
        Parse path from file
        """
        edges = list()
        if self.file_name == '':
            print("Error file_name")
            return 1
        with (pathlib.Path(self.file_name)).open() as f:
            for line in f:
                p = line.replace('\n', '')
                if p:
                    p = p.split(" ")
                    edges.append((float(p[0]), float(p[1])))

        return edges

    def edges_to_points(self, edges) -> list:
        """
        
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
            ps.pose.orientation = euler_to_quaternion(
                yaw=math.atan(yaw_arr[i]), roll=0, pitch=0)
            self.msg.poses.append(ps)

    def PolygonTrajGenerator(self) -> None:
        """

        """
        p_edges = [(2.0, -0.1), (2.1, 1.9),  (0.1, 2.0), (0, 0)]  # square
        points = self.edges_to_points(p_edges)

        for p in points:
            ps = PoseStamped()
            ps.header = self.msg.header

            ps.pose.position.x = p[0]
            ps.pose.position.y = p[1]
            ps.pose.position.z = 0.0
            ps.pose.orientation = euler_to_quaternion(
                yaw=math.atan2(p[1], p[0]), roll=0, pitch=0)
            self.msg.poses.append(ps)

    def SpiralTrajGenerator(self, amplitude) -> None:
        """
        Generate spiral trajectory
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
        Parsing sinus
        """
        traj_type = self.traj_type.strip().split('sin')  # []
        period = float(traj_type[-1])
        amplitude = float(traj_type[0].split("_")[-1])
        reverse = traj_type[0].split("_")[0] == 'reverse'
        return amplitude, period, reverse

    def parse_spiral_traj(self):
        """
        Parsing spiral
        """
        coef = self.traj_type.split('spiral')
        amp = coef[0] if coef[0] != '' or coef[0] is None else 1.0
        return float(amp)


def euler_to_quaternion(yaw, pitch, roll):
    """
    Args:
        yaw: yaw angle
        pitch: pitch angle
        roll: roll angle
    Return:
        quaternion [qx, qy, qz, qw]
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(
        pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(
        pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    traj_publ = TrajPublish("path_pub")
    traj_publ.run()
    return


if __name__ == '__main__':
    main()
