import rclpy
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Path
from scipy.spatial.transform import Rotation
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rosbot_controller.rosbot_2D import RobotState
from rosbot_controller.robot_trajectory import Trajectory


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
        self.initial_pose = None
        self.timer = self.create_timer(1, self.get_robot_pose)

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
        self.declare_parameter('reverse', 'False')

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
        self.reverse = self.get_parameter(
            'reverse').get_parameter_value().bool_value

    def get_robot_pose(self):
        """
        """
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
        self.initial_pose = RobotState(x, y, yaw)
        if self.initial_pose is not None and self.path_pub.get_subscription_count() >= self.num_of_subs:
            self.run()
            self.timer.destroy()
            self.timer.cancel()

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
        self.trajectory = Trajectory(
            start_point=self.initial_pose,
            step=self.step_size,
            frame=self.path_frame,
            length=self.length,
            reverse=self.reverse
        )
        self.prepare_trajectory()
        self.path_pub.publish(self.trajectory.get_path())
        self.destroy_node()
        rclpy.try_shutdown()
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
