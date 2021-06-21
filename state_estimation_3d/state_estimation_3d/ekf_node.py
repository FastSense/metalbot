import cv2
import nnio
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros

from . import spacekf

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_3d')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('period', 0.1)
        self.declare_parameter('acceleration_std', 0.1)
        self.declare_parameter('rot_vel_std', 0.1)

        # Kalman filter parameters
        acceleration_std = self.get_parameter('acceleration_std').get_parameter_value().double_value
        rot_vel_std = self.get_parameter('rot_vel_std').get_parameter_value().double_value

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

        # Create Kalman filter
        self.tracker = spacekf.SpaceKF12(dt=self.period, acceleration_std=acceleration_std, rot_vel_std=rot_vel_std)
        self.tracker.P = self.tracker.P * 0.01

        # Buffers for measurements

    def odometry_callback(self, msg):
        pass

    def imu_callback(self, msg):
        pass

    def step(self):
        '''
        EKF predict and update step
        '''
        self.tracker.predict()
        self.publish_pose()

    def publish_pose(self):
        # Make odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        msg.twist.twist.angular.x = self.tracker.rot_vel[0]
        msg.twist.twist.angular.y = self.tracker.rot_vel[1]
        msg.twist.twist.angular.z = self.tracker.rot_vel[2]
        msg.twist.twist.linear.x = self.tracker.vel[0]
        msg.twist.twist.linear.y = self.tracker.vel[1]
        msg.twist.twist.linear.z = self.tracker.vel[2]
        msg.twist.covariance = self.tracker.P
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
