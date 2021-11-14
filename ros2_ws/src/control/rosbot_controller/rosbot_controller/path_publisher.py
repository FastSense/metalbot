from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
import pathlib
import numpy as np
import rclpy
import math
import time
from scipy.spatial.transform import Rotation
from .robot_trajectory import Trajectory


class TrajPublish(Node):
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

        rclpy.init(args=None)
        super().__init__(node_name)
        self.declare_and_get_parameters()

        self.trajectory = Trajectory(
            step=self.step_size,
            frame=self.path_frame,
            length=self.length
        )
        self.path_pub = self.create_publisher(Path, self.path_topic, 5)
        self.dt = 0.2
        self.prepare_trajectory()

    def declare_and_get_parameters(self):
        """
        """

        self.declare_parameter('traj_type')
        self.declare_parameter('move_plan')
        self.declare_parameter('num_of_subs', 1.0)
        self.declare_parameter('length', 3.0)
        self.declare_parameter('step_size', 0.1)
        self.declare_parameter('path_topic', '/path')
        self.declare_parameter('path_frame', 'odom')

        self.traj_type = self.get_parameter(
            'traj_type').get_parameter_value().string_value
        self.move_plan = self.get_parameter(
            'move_plan').get_parameter_value().string_value
        self.path_frame = self.get_parameter(
            'path_frame').get_parameter_value().string_value
        self.path_topic = self.get_parameter(
            'path_topic').get_parameter_value().string_value
        self.step_size = self.get_parameter(
            'step_size').get_parameter_value().double_value
        self.length = self.get_parameter(
            'length').get_parameter_value().double_value
        self.num_of_subs = self.get_parameter(
            'num_of_subs').get_parameter_value().double_value

    def prepare_trajectory(self):
        """
        """
        if self.traj_type == "from_file":
            self.trajectory.from_move_plan(self.move_plan)
        else:
            self.trajectory.from_string(self.traj_type)

    def run(self):
        """
        Main function. Check the correctness of a type_traj, generate message
        and publish it in topic
        """

        while self.path_pub.get_subscription_count() < self.num_of_subs:
            time.sleep(self.dt)

        self.path_pub.publish(self.trajectory.get_path())
        self.destroy_node()
        rclpy.shutdown()


def main():
    traj_publ = TrajPublish("path_pub")
    traj_publ.run()
    return


if __name__ == '__main__':
    main()
