#!/usr/bin/env python
# license removed for brevity
import rclpy
from rclpy.node import Node
import cv2
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from filterpy.common import Q_discrete_white_noise
from scipy.spatial.transform import Rotation as R

from state_estimation_2d.filter import *
#from state_estimation_2d.geometry import *
from state_estimation_2d.ate import *
import nnio

"""
State vector:
0. x
1. y
2. V_parallel       (velocity along the robot direction)
3. yaw
4. yaw_vel

Measurements:
Odometry:
0. V_parallel
1. yaw_vel

IMU:
0. a_normal         (acceleration orthogonal to the robot direction)
1. yaw_vel
"""

class StateEstimation2D(Node):
    """
    Class for ROS2 node for state estimation
    ------------------
    Attributes:

    odom_gt_sub: ros::Subscriber
        Subscriber to ground truth odometry
    odom_sub: ros::Subscriber
        Subscriber to noised odometry from odom_noiser node
    imu_sub: ros::Subscriber
        Subscriber to imu
    cmd_vel_sub: ros::Subscriber
        Subscriber to controls from cmd_vel
    pose_pub: ros::Publisher
        Publish filtered odometry to /odom_filtered
    odom_noised: Odometry
        Noised odometry from odom_noiser node
    odom_gt: Odometry
        Ground truth odometry
    odom_filtered: Odometry
        Kalman filtered odometry
    got_measurements: bool
        Flag that is true when got first measurement 
    model_path: str
        Path to file with NN control model
    model: ONNXModel
        Pretrained NN control model
    dt: float
        Time period for update step
    R_odom: np.array
        Odometry measurement covariance matrix
    R_imu: np.array
        Imu measurement covariance matrix
    Q_rot: np.array
        Rotation component of model noise covariance matrix
    Q: np.array
        Model noise covariance matrix
    filter: Filter2D
        EKF class object
    ate: ErrorEstimator
        Object for ate metrics evaluation
    predict_timer: ros::Timer
        Timer for update function
    """
    def __init__(self):
        super().__init__('state_estimation_2d')
        # ROS Subscribers
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
        self.pose_pub = self.create_publisher(Odometry, '/odom_filtered', 10)
        # ROS variables
        self.odom_noised = Odometry()
        self.odom_gt = Odometry()
        self.odom_filtered = Odometry()
        self.got_measurements = 0
        self.control = np.zeros(2)
        self.z_odom = np.zeros(2)
        self.z_imu = np.zeros(2)
        # Upload NN control model
        #self.model_path = "/home/user/ros2_ws/new_model_dynamic_batch.onnx"
        self.model_path = 'http://192.168.194.51:8345/ml-control/gz-rosbot/new_model_dynamic_batch.onnx'
        self.model = nnio.ONNXModel(self.model_path)
        # Filter parameters
        self.dt = 0.1
        self.R_odom = np.array([[0.5, 0],
                                [0, 0.1]])
        self.R_imu = np.array([[0.1, 0],
                               [0, 0.1]])
        
        self.filter = Filter2D(
            x_init=np.zeros(5), 
            P_init=np.eye(5) * 0.01,                          
            dt=self.dt,
            v_var=0.1,
            w_var=0.1,
        )
        self.ate = ErrorEstimator()
        # Timer for update function
        self.predict_timer = self.create_timer(
            self.dt,
            self.step_filter
        )
    
    def control_callback(self, msg):
        """
        Callback from /cmd_vel topic with control commands
        @ parameters
        msg: Twist
        """
        self.control = np.array([msg.linear.x, msg.angular.z])
    
    def odometry_callback(self, msg):
        """
        Callback from /odom_noised topic
        @ parameters
        msg: Odometry
        """
        self.odom_noised = msg
        self.z_odom = self.odometry_to_vector(msg)
        self.got_measurements = 1

    def odometry_to_vector(self, odom):
        """
        Transfer odometry message to measurement vector for filter
        @ parameters
        odom: Odometry
            Odometry msg that needs to be transfered
        """
        z_odom = np.zeros(2)
        z_odom[0] = odom.twist.twist.linear.x
        z_odom[1] = odom.twist.twist.angular.z
        return z_odom

    def odometry_gt_callback(self, msg):
        """
        Callback from /odom topic
        @ parameters
        msg: Odometry
        """
        self.odom_gt = msg
    
    def imu_callback(self, msg):
        """
        Callback from /imu topic
        @ parameters
        msg: Imu
        """
        self.z_imu = self.imu_to_vector(msg)
        self.got_measurements = 1
    
    def imu_to_vector(self, imu):
        """Transfer imu message to measurement vector for filter"""
        z_imu = np.zeros(2)
        z_imu[0] = imu.linear_acceleration.y
        z_imu[1] = imu.angular_velocity.z
        return z_imu
    
    def step_filter(self):
        """
        Kalman filter iteration
        Theory: https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf
        """
        if self.got_measurements:
            # Predict step
            self.filter.predict_by_nn_model(self.model, self.control)
            # Measurement update step
            self.filter.update_odom(self.z_odom, self.R_odom)
            self.filter.update_imu(self.z_imu, self.R_imu)
            # Transfer vectors to odometry messages
            self.state_to_odometry(self.filter.x_opt, self.filter.P_opt)
            # Compute error metrics
            self.ate.compute_curr_ate(self.odom_gt, self.odom_filtered)
            self.pose_pub.publish(self.odom_filtered)

    def state_to_odometry(self, x, P):
        """
        Transfer filtered state vectors to odometry message
        @ parameters
        x: np.array
            State vector
        P: np.array
            Covariance matrix
        """
        self.odom_filtered.header = self.odom_noised.header
        self.odom_filtered.child_frame_id = self.odom_noised.child_frame_id
        self.odom_filtered.pose.pose.position.x = x[0]
        self.odom_filtered.pose.pose.position.y = x[1]
        self.odom_filtered.pose.pose.position.z = self.odom_noised.pose.pose.position.z
        # Transfer yaw angle from euler to quaternion
        r = R.from_euler('z', x[3], degrees=False)
        q = r.as_quat()
        self.odom_filtered.pose.pose.orientation.x = q[0]
        self.odom_filtered.pose.pose.orientation.y = q[1]
        self.odom_filtered.pose.pose.orientation.z = q[2]
        self.odom_filtered.pose.pose.orientation.w = q[3]
        self.odom_filtered.twist.twist.linear.x = x[2]
        self.odom_filtered.twist.twist.linear.y = self.odom_noised.twist.twist.linear.y
        self.odom_filtered.twist.twist.linear.z = self.odom_noised.twist.twist.linear.z
        self.odom_filtered.twist.twist.angular.x = self.odom_noised.twist.twist.angular.x
        self.odom_filtered.twist.twist.angular.y = self.odom_noised.twist.twist.angular.y
        self.odom_filtered.twist.twist.angular.z = x[4]
        # Fill the odometry message covariance matrix with computed KF covariance
        self.odom_filtered.pose.covariance = self.pose_covariance_to_vector(P)
        self.odom_filtered.twist.covariance = self.twist_covariance_to_vector(P)

    def pose_covariance_to_vector(self, P):
        """
        Transfer filter computed pose covariance matrix to vector
        http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
        @ parameters
        P: np.array
            Covariance matrix
        """
        cov_vector = self.odom_noised.pose.covariance
        cov_vector[14] = 0.1
        cov_vector[21] = 0.01
        cov_vector[28] = 0.01
        cov_vector[0] = P[0,0]
        cov_vector[1] = P[0,1]
        cov_vector[5] = P[0,3]
        cov_vector[6] = P[1,0]
        cov_vector[7] = P[1,1]
        cov_vector[11] = P[1,3]
        cov_vector[30] = P[3,0]
        cov_vector[31] = P[3,1]
        cov_vector[35] = P[3,3]
        return cov_vector
    
    def twist_covariance_to_vector(self, P):
        """
        Transfer filter computed twist covariance matrix to vector
        http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/TwistWithCovariance.html
        @ parameters
        P: np.array
            Covariance matrix
        """
        cov_vector = self.odom_noised.twist.covariance
        cov_vector[0] = P[2,2]
        cov_vector[5] = P[2,4]
        cov_vector[30] = P[4,2]
        cov_vector[35] = P[4,4]
        return cov_vector

def main():
    rclpy.init()
    state_estimator = StateEstimation2D()
    try:
        rclpy.spin(state_estimator)
    except KeyboardInterrupt:
        ate, std = state_estimator.ate.evaluate_ate()
        print('ATE:', ate)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
