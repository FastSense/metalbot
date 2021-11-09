import cv2
import nnio
import numpy as np
from scipy.spatial.transform import Rotation
from argparse import Namespace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros

from perception_msgs.msg import OdoFlow


class DebugNode(Node):
    def __init__(self):
        super().__init__('debug_node')

        self.bridge = CvBridge()

        # Subscribe to sensor topics
        self.create_subscription(
            OdoFlow,
            'odom_flow',
            self.create_message_callback('odom_flow'),
            10,
        )
        self.create_subscription(
            Imu,
            'imu',
            self.create_message_callback('imu'),
            10,
        )
        self.create_subscription(
            CameraInfo,
            'rectified_camera_info',
            self.create_message_callback('rectified_camera_info'),
            10,
        )
        self.create_subscription(
            Image,
            'left_rect',
            self.create_message_callback('left_rect'),
            10,
        )
        self.create_subscription(
            Image,
            'depth',
            self.create_message_callback('depth'),
            10,
        )

        self.messages = []

        self.create_timer(1, self.check_and_print)

    def create_message_callback(self, msg_type):
        def callback(msg):
            sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.messages.append({
                'time': sec,
                'type': msg_type,
            })

        return callback

    def check_and_print(self):
        print()
        print(' '.join([m['type'] for m in self.messages]))
        self.messages = []


def main(args=None):
    print('Hi from debug.')

    rclpy.init(args=args)

    node = DebugNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
