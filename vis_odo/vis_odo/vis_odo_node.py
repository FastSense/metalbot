import cv2
import nnio
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros


class OdometryNode(Node):
    def __init__(self):
        super().__init__('vis_odo')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('network_path', 'http://192.168.194.51:8345/odometry/2021.06.01_odometry/odometry_op11.onnx')
        self.declare_parameter('period', 0.1)

        # Subscribe to camera topics
        self.create_subscription(
            Image,
            'left_rect',
            self.left_rect_callback,
            10,
        )
        self.create_subscription(
            Image,
            'right_rect',
            self.right_rect_callback,
            10,
        )

        # Publisher
        self.odom_publisher = self.create_publisher(
            Odometry,
            'vis_odo',
            10,
        )

        # Odometry neural network
        network_path = self.get_parameter('network_path').get_parameter_value().string_value
        self.network = nnio.ONNXModel(network_path)
        self.preprocess = nnio.Preprocessing(
            resize=(256, 192),
            dtype='float32',
            divide_by_255=True,
            channels_first=True,
            batch_dimension=True,
        )

        # Buffer for images
        self.left_msg = None
        self.right_msg = None
        self.last_pair = None
        self.last_pair_preprocessed = None

        # Create timer for odometry
        self.period = self.get_parameter('period').get_parameter_value().double_value
        self.create_timer(self.period, self.publish_odometry)

        # Covariance matrix
        std_linear = 0.1
        std_angular = 0.1
        covariance = np.diag([std_angular] * 3 + [std_linear] * 3)
        self.covariance = list(covariance.flatten())

    def left_rect_callback(self, msg):
        self.left_msg = msg
        self.check_pair()

    def right_rect_callback(self, msg):
        self.right_msg = msg
        self.check_pair()

    def check_pair(self):
        if self.left_msg is None or self.right_msg is None:
            return
        time_left = self.left_msg.header.stamp.sec + self.left_msg.header.stamp.nanosec * 1e-9
        time_right = self.right_msg.header.stamp.sec + self.right_msg.header.stamp.nanosec * 1e-9
        if abs(time_left - time_right) < 0.01:
            self.last_pair = self.left_msg, self.right_msg

    def publish_odometry(self):
        if self.last_pair is None:
            return
        # Prepare inputs
        img_left = self.bridge.imgmsg_to_cv2(self.last_pair[0])
        img_left = cv2.cvtColor(img_left, cv2.COLOR_GRAY2RGB)
        img_right = self.bridge.imgmsg_to_cv2(self.last_pair[1])
        img_right = cv2.cvtColor(img_right, cv2.COLOR_GRAY2RGB)
        pair_preprocessed = np.concatenate([
            self.preprocess(img_left),  # [1, 3, H, W]
            self.preprocess(img_right), # [1, 3, H, W]
        ], 1) # [1, 6, H, W]
        # Get previuosly prepared inputs
        if self.last_pair_preprocessed is None:
            self.last_pair_preprocessed = pair_preprocessed
            return
        nn_inp = np.concatenate([
            pair_preprocessed,           # [1, 6, H, W]
            self.last_pair_preprocessed, # [1, 6, H, W]
        ], 1) # [1, 12, H, W]
        self.last_pair_preprocessed = pair_preprocessed
        # Compute odometry
        odom = self.network(nn_inp) # [1, 6]
        spd = odom / self.period
        spd = [float(val) for val in spd[0]]
        # Make odometry message
        msg = Odometry()
        msg.header.stamp = self.last_pair[0].header.stamp
        msg.header.frame_id = self.last_pair[0].header.frame_id
        msg.child_frame_id = self.last_pair[0].header.frame_id
        msg.twist.twist.angular.x = spd[0]
        msg.twist.twist.angular.y = spd[1]
        msg.twist.twist.angular.z = spd[2]
        msg.twist.twist.linear.x = spd[3]
        msg.twist.twist.linear.y = spd[4]
        msg.twist.twist.linear.z = spd[5]
        msg.twist.covariance = self.covariance
        self.odom_publisher.publish(msg)
        # print(msg)


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
