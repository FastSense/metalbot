#!/usr/bin/env python
# license removed for brevity
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time import Duration
from rclpy.clock import Clock
import sys
import cv2
import os
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import message_filters
import numpy as np
import json
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Point, Quaternion, Pose, PoseWithCovariance, PoseWithCovarianceStamped

class StateEstimation2D(Node):
    def __init__(self):
        super().__init__('state_estimation_2d')

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            15)
        self.odom = Odometry()
        self.imu = Imu()
        
    def odometry_callback(self, msg):
        self.odom = msg

    def imu_callback(self, msg):
        print(msg)
        self.imu = msg



def main():
    rclpy.init()
    state_estimator = StateEstimation2D()
    try:
        rclpy.spin(state_estimator)
    except KeyboardInterrupt:
        rclpy.logerr("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()