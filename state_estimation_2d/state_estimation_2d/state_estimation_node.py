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
        self.odom_filtered = Odometry()

        self.predict_timer = self.create_timer(
            self.dt,
            self.update
        )

        self.pose_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        
    def odometry_callback(self, msg):
        self.odom = msg
        z_odom = self.odometry_to_vector(msg)
        self.filter.set_odometry(z_odom)
        # self.odom = msg
        # filter.set_odometry(msg)
        # self.x_opt, self.P_opt = filter.update_odom()
        # self.state_to_odometry(msg)
        # covariance_to_vector(msg)

    def state_to_odometry(self, x, P):
        self.odom_filtered.header = msg.header
        self.odom_filtered.child_frame_id = msg.child_frame_id
        self.odom_filtered.pose.position.x = self.x_opt[0]
        self.odom_filtered.pose.position.y = self.x_opt[1]
        self.odom_filtered.pose.position.z = msg.pose.position.z
        roll, pitch, yaw = euler_from_quaternion(msg.pose.position.orientation)
        q = quaternion_from_euler(roll, pitch, self.x_opt[7])
        self.odom_filtered.pose.orientation.x = q[0]
        self.odom_filtered.pose.orientation.y = q[1]
        self.odom_filtered.pose.orientation.z = q[2]
        self.odom_filtered.pose.orientation.w = q[3]

    def covariance_to_vector(self, msg):
        cov_vector = np.zeros(36)

    def imu_callback(self, msg):
        self.imu = msg
        z_imu = self.imu_to_vector(msg)
        self.filter.set_imu(z_imu)
        # self.imu = msg
        # filter.set_imu(msg)
        # self.x_opt, self.P_opt = filter.update_imu()

    def update_callback(self):
        x_predict, P_predict = self.filter.predict()
        x_opt, P_opt = self.filter.update(x_predict, P_predict)
        self.state_to_odometry(x_opt, P_opt)
        self.pose_pub.publish(self.odom_filtered)


    # def model_measurements(self, vec):
    #     filter.set_z_model(vec)
    #     self.x_opt, self.P_opt = filter.update_model()



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