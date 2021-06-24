#!/usr/bin/env python
# license removed for brevity
import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
import scipy
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation as R

from state_estimation_2d.filter import *
from state_estimation_2d.geometry import *
from state_estimation_2d.ate import *
import nnio

class StateEstimation2D(Node):
    def __init__(self):
        super().__init__('state_estimation_2d')
        self.odom_gt_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_gt_callback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom_noised',
            self.odometry_callback,
            10)
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            15)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.control_callback,
            15)
        self.control = Twist()
        self.odom = Odometry()
        self.odom_gt = Odometry()
        self.imu = Imu()
        self.model_path = "/home/user/ros2_ws/new_model_dynamic_batch.onnx"
        self.model = nnio.ONNXModel(self.model_path)
        self.ate = ErrorEstimator()
        self.dt = 0.1
        self.R_odom = np.array([[0.5, 0, 0],
                                [0, 0.5, 0],
                                [0, 0, 0.1]])
        self.R_imu = np.array([[0.1, 0],
                               [0, 0.1]])
        Q_rot = np.array([
            [0.333 * self.dt**3, 0.5 * self.dt**2],
            [ 0.5 * self.dt**2,           self.dt],
        ]) * 0.1
        self.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(dim=2, dt=self.dt, var=0.1, block_size=2),
            Q_rot,
        )
        self.filter = Filter2D(x_init = np.zeros(6), 
                               P_init = np.eye(6) * 0.01, 
                               R_odom = self.R_odom, 
                               R_imu = self.R_imu, 
                               Q = self.Q)
        self.odom_filtered = Odometry()
        self.got_measurements = 0
        self.predict_timer = self.create_timer(self.dt,
                                               self.update)
        self.pose_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        
    def control_callback(self, msg):
        self.control = msg
        self.filter.set_control(self.control)
    
    def odometry_callback(self, msg):
        self.odom = msg
        z_odom = self.odometry_to_vector(msg)
        self.filter.set_odometry(z_odom)
        self.got_measurements = 1

    def odometry_gt_callback(self, msg):
        self.odom_gt = msg

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
    
    def imu_to_vector(self, imu):
        z_imu = np.zeros(2)
        z_imu[0] = imu.linear_acceleration.y
        z_imu[1] = imu.angular_velocity.z
        return z_imu

    def update(self):
        if self.got_measurements:
            self.filter.update_state_by_nn_model(self.model)
            x_opt, P_opt = self.filter.update()
            self.state_to_odometry(x_opt, P_opt)
            self.ate.compute_curr_ate(self.odom_gt, self.odom_filtered)
            self.pose_pub.publish(self.odom_filtered)

    def state_to_odometry(self, x, P):
        self.odom_filtered.header = self.odom.header
        self.odom_filtered.child_frame_id = self.odom.child_frame_id
        self.odom_filtered.pose.pose.position.x = x[0]
        self.odom_filtered.pose.pose.position.y = x[1]
        self.odom_filtered.pose.pose.position.z = self.odom.pose.pose.position.z
        r = R.from_euler('z', x[4], degrees=True)
        q = r.as_quat()
        self.odom_filtered.pose.pose.orientation.x = q[0]
        self.odom_filtered.pose.pose.orientation.y = q[1]
        self.odom_filtered.pose.pose.orientation.z = q[2]
        self.odom_filtered.pose.pose.orientation.w = q[3]
        self.odom_filtered.twist.twist.linear.x = x[2]
        self.odom_filtered.twist.twist.linear.y = x[3]
        self.odom_filtered.twist.twist.linear.z = self.odom.twist.twist.linear.z
        self.odom_filtered.twist.twist.angular.x = self.odom.twist.twist.angular.x
        self.odom_filtered.twist.twist.angular.y = self.odom.twist.twist.angular.y
        self.odom_filtered.twist.twist.angular.z = x[5]
        self.odom_filtered.pose.covariance = self.pose_covariance_to_vector(P)
        self.odom_filtered.twist.covariance = self.twist_covariance_to_vector(P)

    def pose_covariance_to_vector(self, P):
        cov_vector = self.odom.pose.covariance
        cov_vector[14] = 0.1
        cov_vector[21] = 0.01
        cov_vector[28] = 0.01
        cov_vector[0] = P[0,0]
        cov_vector[1] = P[0,1]
        cov_vector[5] = P[0,4]
        cov_vector[6] = P[1,0]
        cov_vector[7] = P[1,1]
        cov_vector[11] = P[1,4]
        cov_vector[30] = P[4,0]
        cov_vector[31] = P[4,1]
        cov_vector[35] = P[4,4]
        return cov_vector
    
    def twist_covariance_to_vector(self, P):
        cov_vector = self.odom.twist.covariance
        cov_vector[0] = P[2,2]
        cov_vector[1] = P[2,3]
        cov_vector[5] = P[2,5]
        cov_vector[6] = P[3,2]
        cov_vector[7] = P[3,3]
        cov_vector[11] = P[3,5]
        cov_vector[30] = P[5,2]
        cov_vector[31] = P[5,3]
        cov_vector[35] = P[5,5]
        return cov_vector



def main():
    rclpy.init()
    state_estimator = StateEstimation2D()
    try:
        rclpy.spin(state_estimator)
    except KeyboardInterrupt:
        ate, std = state_estimator.ate.evaluate_ate()
        print("ATE:")
        print(ate)
        print("STD:")
        print(std)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()