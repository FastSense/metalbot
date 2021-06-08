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

import Model2D
import Measurement2D
import Filter2D
from . import geometry

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

        self.dt = 0.01
        self.filter = Filter2D()
        self.x_opt = np.zeros(8)
        self.P_opt = np.zeros((8, 8))
        self.odom_filtered = Odometry()

        self.create_timer(0.2, self.timer_callback)
        self.predict_timer = self.create_timer(
            self.dt,
            self.predict_callback
        )

        self.pose_pub = self.create_publisher(Odometry, '/pose_filtered', 10)
        
    def odometry_callback(self, msg):
        self.odom = msg
        filter.set_odometry(msg)
        self.x_opt, self.P_opt = filter.update_odom()
        state_to_odometry(msg)

    def state_to_odometry(self, msg):
        self.odom_filtered.header = msg.header
        self.odom_filtered.child_frame_id = msg.child_frame_id
        self.odom_filtered.pose.position.x = 
        self.odom_filtered.pose.position.y =
        self.odom_filtered.pose.position.z = msg.pose.position.z
        self.odom_filtered.pose.orientation.x = msg.pose.orientation.x
        self.odom_filtered.pose.orientation.y = msg.pose.orientation.y 
        self.odom_filtered.pose.orientation.z = 
        self.odom_filtered.pose.orientation.w = 

    def imu_callback(self, msg):
        self.imu = msg
        filter.set_imu(msg)
        self.x_opt, self.P_opt = filter.update_imu()

    def predict_callback(self):
        filter.predict()

    def model_measurements(self, vec):
        filter.set_z_model(vec)
        self.x_opt, self.P_opt = filter.update_model()



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