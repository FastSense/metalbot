import rclpy
import numpy as np
import pandas as pd
from nav_msgs.msg import Path
from logger.logger import Logger
from logger.utils import calculate_ATE
from scipy.spatial.transform import Rotation


class PathAnalyzer(Logger):
    """
    """

    def __init__(self, name='path_analyzer'):
        """
        """
        super().__init__('path_analyzer')
        self.init_and_get_extra_parameters()
        self.path = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw'
            ]
        )
        self.path_sub = self.create_subscription(
            Path,
            self.path_topic,
            self.path_callback,
            1
        )

    def init_and_get_extra_parameters(self):
        """
        """
        self.declare_parameter('path_topic', "/path")
        self.path_topic = self.get_parameter(
            'path_topic').get_parameter_value().string_value

    def path_callback(self, msg):
        """
        """
        self.convert_path_to_numpy(msg)

    def convert_path_to_numpy(self, path):
        """
        """
        for item in path.poses:
            item = item.pose
            xyz = [item.position.x,
                   item.position.y,
                   item.position.z]

            rpy = Rotation.from_quat([
                np.float(item.orientation.x),
                np.float(item.orientation.y),
                np.float(item.orientation.z),
                np.float(item.orientation.w)]
            ).as_euler('xyz')
            rpy = list(rpy)

            last_row = len(self.path)
            self.path.loc[last_row] = xyz + rpy

    def on_shutdown(self):
        """
        """

        print("END")
        print("ROBOT_STATE")
        print(self.robot_state)
        print("PATH")
        print(self.path)
        print("PATH X Y")
        print(self.path[0:2])

        ate = calculate_ATE(
            self.path.to_numpy(),
            self.robot_state.to_numpy()
        )


def main():
    """
    Declares the logger node.
    Node works
    """
    rclpy.init()
    path_analyzer = PathAnalyzer('logger')

    try:
        rclpy.spin(path_analyzer)
    except:
        pass

    path_analyzer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
