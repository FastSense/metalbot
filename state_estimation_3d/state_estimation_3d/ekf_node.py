import cv2
import nnio
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_3d')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('period', 0.1)

        # Subscribe to sensor topics
        self.create_subscription(
            Odometry,
            'vis_odo',
            self.odometry_callback,
            10,
        )
        self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10,
        )

        # Publisher
        self.pose_publisher = self.create_publisher(
            Odometry,
            'pose',
            10,
        )

        # Create timer
        self.period = self.get_parameter('period').get_parameter_value().double_value
        self.create_timer(self.period, self.step)

    def odometry_callback(self, msg):
        pass

    def imu_callback(self, msg):
        pass

    def step(self):
        '''
        EKF predict and update step
        '''
        

    def publish_pose(self):
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


def main(args=None):
    print('Hi from ekf_3d.')

    rclpy.init(args=args)

    node = EKFNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
