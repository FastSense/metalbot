import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from .robot_trajectory import Trajectory
from rosbot_controller.rosbot_2D import RobotState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from argparse import Namespace
from scipy.spatial.transform import Rotation
from nav_msgs.msg import Odometry
import numpy as np


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

        super().__init__(node_name)
        self.declare_and_get_parameters()
        self.parent_frame = "odom"
        self.robot_frame = "base_link"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.dt = 0.2
        self.path_pub = self.create_publisher(Path, self.path_topic, 5)
        rclpy.spin_once(self)
        self.initial_pose = None
        self.timer = self.create_timer(1, self.get_robot_pose)
        # self.run()

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

    def get_robot_pose(self):
        """
        """
        # print("!!!!!1")
        # try:
        trans = self.tf_buffer.lookup_transform(
            self.parent_frame,
            self.robot_frame,
            rclpy.time.Time())
        x = trans.transform.translation.x
        y = trans.transform.translation.y
        yaw = Rotation.from_quat([
            np.float(trans.transform.rotation.x),
            np.float(trans.transform.rotation.y),
            np.float(trans.transform.rotation.z),
            np.float(trans.transform.rotation.w)]
        ).as_euler('xyz')[2]
        print("POSE", x, y, yaw)
        print(self.path_pub.get_subscription_count(), self.num_of_subs)
        self.initial_pose = RobotState(x, y, yaw)
        if self.initial_pose is not None and self.path_pub.get_subscription_count() >= self.num_of_subs:
            print("GO IN")
            self.run()
            self.timer.destroy()
            self.timer.cancel()
        # except:
        #    print("JOPA")

    def prepare_trajectory(self):
        """
        """
        if self.traj_type == "from_file":
            print(4)
            self.trajectory.from_move_plan(self.move_plan)
        else:
            print(3)
            self.trajectory.from_string(self.traj_type)

    def run(self):
        """
        Main function. Check the correctness of a type_traj, generate message
        and publish it in topic
        """
        print("1")
        self.trajectory = Trajectory(
            start_point=self.initial_pose,
            step=self.step_size,
            frame=self.path_frame,
            length=self.length
        )
        print("2")
        self.prepare_trajectory()
        # print("TRAJECTORY", self.trajectory.get_path())
        self.path_pub.publish(self.trajectory.get_path())
        self.destroy_node()
        rclpy.shutdown()
        return


def main():

    rclpy.init()
    traj_publ = TrajPublish("path_pub")
    try:
        rclpy.spin(traj_publ)
    except:
        pass
    return


if __name__ == '__main__':
    main()
