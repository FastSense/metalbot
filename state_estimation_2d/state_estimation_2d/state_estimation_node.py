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
        self.got_measurements = 0

        self.predict_timer = self.create_timer(
            self.dt,
            self.update
        )

        self.pose_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        
    def odometry_callback(self, msg):
        self.odom = msg
        z_odom = self.odometry_to_vector(msg)
        self.filter.set_odometry(z_odom)
        self.got_measurements = 1
        # self.odom = msg
        # filter.set_odometry(msg)
        # self.x_opt, self.P_opt = filter.update_odom()
        # self.state_to_odometry(msg)
        # covariance_to_vector(msg)

    def odometry_to_vector(self, odom):
        z_odom = np.zeros(3)
        z_odom[0] = odom.twist.twist.linear.x
        z_odom[1] = odom.twist.twist.linear.y
        z_odom[2] = odom.twist.twist.angular.z
        return z_odom

    def imu_callback(self, msg):
        self.imu = msg
        z_imu = self.imu_to_vector(msg)
        self.filter.set_imu(z_imu)
        self.got_measurements = 1
        # self.imu = msg
        # filter.set_imu(msg)
        # self.x_opt, self.P_opt = filter.update_imu()
    
    def imu_to_vector(self, imu):
        z_imu = np.zeros(1)
        z_imu[0] = imu.angular_velocity.z
        return z_imu

    def update_callback(self):
        if self.got_measurements:
            x_predict, P_predict = self.filter.predict()
            x_opt, P_opt = self.filter.update(x_predict, P_predict)
            self.state_to_odometry(x_opt, P_opt)
            self.pose_pub.publish(self.odom_filtered)

    def state_to_odometry(self, x, P):
        self.odom_filtered.header = self.odom.header
        self.odom_filtered.child_frame_id = self.odom.child_frame_id
        self.odom_filtered.pose.position.x = x[0]
        self.odom_filtered.pose.position.y = x[1]
        self.odom_filtered.pose.position.z = self.odom.pose.position.z
        roll, pitch, yaw = euler_from_quaternion(msg.pose.position.orientation)
        q = quaternion_from_euler(roll, pitch, x[6])
        self.odom_filtered.pose.orientation.x = q[0]
        self.odom_filtered.pose.orientation.y = q[1]
        self.odom_filtered.pose.orientation.z = q[2]
        self.odom_filtered.pose.orientation.w = q[3]
        self.odom_filtered.twist.twist.linear.x = x[2]
        self.odom_filtered.twist.twist.linear.y = x[3]
        self.odom_filtered.twist.twist.linear.z = self.odom.twist.twist.linear.z
        self.odom_filtered.twist.twist.angular.x = self.odom.twist.twist.angular.x
        self.odom_filtered.twist.twist.angular.y = self.odom.twist.twist.angular.y
        self.odom_filtered.twist.twist.angular.z = x[7]
        self.odom_filtered.pose.covariance = self.pose_covariance_to_vector(P)
        self.odom_filtered.twist.covariance = self.twist_covariance_to_vector(P)

    def pose_covariance_to_vector(self, P):
        cov_vector = self.odom.pose.covariance
        cov_vector[0] = P[0,0]
        cov_vector[1] = P[0,1]
        cov_vector[5] = P[0,6]
        cov_vector[6] = P[1,0]
        cov_vector[7] = P[1,1]
        cov_vector[11] = P[1,6]
        cov_vector[30] = P[6,0]
        cov_vector[31] = P[6,1]
        cov_vector[35] = P[6,6]
        return cov_vector
    
    def twist_covariance_to_vector(self, P):
        cov_vector = self.odom.twist.covariance
        cov_vector[0] = P[2,2]
        cov_vector[1] = P[2,3]
        cov_vector[5] = P[2,7]
        cov_vector[6] = P[3,2]
        cov_vector[7] = P[3,3]
        cov_vector[11] = P[3,7]
        cov_vector[30] = P[7,2]
        cov_vector[31] = P[7,3]
        cov_vector[35] = P[7,7]
        return cov_vector



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