import cv2
import nnio

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import tf2_ros


class OdometryNode(Node):
    def __init__(self):
        super().__init__('oakd')

        self.bridge = CvBridge()

        self.declare_parameter('network_path', 'http://192.168.194.51:8345/odometry/2021.06.01_odometry/odometry_op11.onnx')

        network_path = self.get_parameter('network_path')

        # Odometry neural network
        self.network = nnio.ONNXModel(network_path)
        self.preprocess = nnio.Preprocessing(
            resize=(256, 192),
            dtype='float32',
            divide_by_255=True,
            channels_first=True,
            batch_dimension=True,
        )

    def check_queues(self):
        pass


def main(args=None):
    print('Hi from odometry.')

    rclpy.init(args=args)

    node = OdometryNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
