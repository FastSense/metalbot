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
import tf2_ros
from argparse import Namespace

from state_estimation_2d.filter import *
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
        self.init_parameters()
        self.update_parameters()

        self.odom_sub = self.create_subscription(
            Twist,
            self.odom_topic,
            self.odometry_callback,
            10)
        self.odom_sim_sub = self.create_subscription(
            Odometry,
            self.odom_sim_topic,
            self.odom_sim_callback,
            10)
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            self.cmd_topic,
            self.control_callback,
            15)
        self.imu_accel_sub = self.create_subscription(
            Imu,
            self.imu_accel_topic,
            self.imu_accel_callback,
            15)
        self.imu_gyro_sub = self.create_subscription(
            Imu,
            self.imu_gyro_topic,
            self.imu_gyro_callback,
            15)
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            15)

        self.pose_pub = self.create_publisher(Odometry, self.publish_topic, 10)
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ROS variables
        self.odom_noised = Odometry()
        self.odom_gt = Odometry()
        self.odom_filtered = Odometry()
        self.got_measurements = 0
        self.control = np.zeros(2)
        self.z_odom = np.zeros(2)
        self.z_imu = np.zeros(2)
        self.z_accel = None
        self.z_gyro = None
        self.imu_extrinsic = None
        self.rot_extrinsic = None

        # Upload NN control model
        if self.use_nn_model:
            self.model = nnio.ONNXModel(self.model_path)

        # Filter parameters
        self.filter = Filter2D(
            x_init=np.zeros(5), 
            P_init=np.eye(5) * 0.01,                          
            dt=self.dt,
            v_var=self.R_odom[0][0]**2,
            w_var=self.R_odom[1][1]**2,
        )

        self.distance = 0
        self.x_prev = 0
        self.y_prev = 0
        self.odom_gt_prev = Odometry()
        self.ate = ErrorEstimator()

        # Timer for update function
        self.predict_timer = self.create_timer(
            self.dt,
            self.step_filter
        )
    
    def init_parameters(self):
        self.declare_parameters(
            namespace="",
            parameters=[
                ("use_nn_model", False),
                ("odom_topic", "odom"),
                ("imu_topic", "imu"),
                ("imu_accel_topic", "/camera/accel/sample"),
                ("imu_gyro_topic", "/camera/gyro/sample"),
                ("cmd_topic", "cmd"),
                ("odom_sim_topic", "odom_noised"),
                ("publish_topic", "odom_filtered"),
                ("imu_frame", "camera_gyro_optical_frame"),
                ("robot_base_frame", "base_link"),
                ("path_to_nn_model", "http://192.168.194.51:8345/ml-control/gz-rosbot/new_model_dynamic_batch.onnx"),
                ("odom_noised_sim_topic", "odom_noised"),
                ("time_step", 0.1),
                ("odom_sim_covariance", [0.5, 0,
                                        0, 0.1]),
                ("imu_sim_covariance", [0.5, 0,
                                        0, 0.1]),
                ("gyro_robot_covariance", 200.0),
                ("accel_robot_covariance", 200.0),
            ]
        )

    def update_parameters(self):
        self.use_nn_model = self.get_parameter('use_nn_model').get_parameter_value().bool_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value
        self.imu_accel_topic = self.get_parameter('imu_accel_topic').get_parameter_value().string_value
        self.imu_gyro_topic = self.get_parameter('imu_gyro_topic').get_parameter_value().string_value
        self.cmd_topic = self.get_parameter('cmd_topic').get_parameter_value().string_value
        self.odom_sim_topic = self.get_parameter('odom_sim_topic').get_parameter_value().string_value
        self.publish_topic = self.get_parameter('publish_topic').get_parameter_value().string_value
        self.imu_frame = self.get_parameter('imu_frame').get_parameter_value().string_value
        self.robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value
        self.model_path = self.get_parameter('path_to_nn_model').get_parameter_value().string_value
        self.odom_noised_topic = self.get_parameter('odom_noised_sim_topic').get_parameter_value().string_value
        self.dt = self.get_parameter('time_step').get_parameter_value().double_value
        R_odom_vec = self.get_parameter('odom_sim_covariance').get_parameter_value().double_array_value
        R_imu_vec = self.get_parameter('imu_sim_covariance').get_parameter_value().double_array_value
        self.R_accel = np.array(self.get_parameter('accel_robot_covariance').get_parameter_value().double_value)
        self.R_gyro = np.array(self.get_parameter('gyro_robot_covariance').get_parameter_value().double_value)

        self.R_odom = np.array(R_odom_vec).reshape(len(R_odom_vec)//2, len(R_odom_vec)//2)
        self.R_imu = np.array(R_imu_vec).reshape(len(R_imu_vec)//2, len(R_imu_vec)//2)
    
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

    def odom_sim_callback(self, msg):
        self.odom_sim = msg
        self.z_odom = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.angular.z,
        ])
        self.got_measurements = 1

    def odometry_to_vector(self, odom):
        """
        Transfer odometry message to measurement vector for filter
        @ parameters
        odom: Odometry
            Odometry msg that needs to be transfered
        """
        z_odom = np.zeros(2)
        z_odom[0] = odom.linear.x
        z_odom[1] = odom.angular.z * 2
        return z_odom

    def imu_accel_callback(self, msg):
        self.imu_accel = msg
        if self.rot_extrinsic is not None:
            accel = self.rot_extrinsic.T @ np.array([self.imu_accel.linear_acceleration.x,
                                            self.imu_accel.linear_acceleration.y,
                                            self.imu_accel.linear_acceleration.z])
            self.z_accel = accel[1]
    
    def imu_gyro_callback(self, msg):
        if self.imu_extrinsic is None:
            print("Finding imu extrinsics")
            self.imu_extrinsic = self.get_extrinsic(self.imu_frame, self.robot_base_frame)
        if self.imu_extrinsic is not None and self.rot_extrinsic is None:
            self.rot_extrinsic = np.ascontiguousarray(self.imu_extrinsic[:3,:3])
        self.imu_gyro = msg
        if self.rot_extrinsic is not None:
            gyro = self.rot_extrinsic.T @ np.array([self.imu_gyro.angular_velocity.x,
                                            self.imu_gyro.angular_velocity.y,
                                            self.imu_gyro.angular_velocity.z])
            self.z_gyro = gyro[2]

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
            if self.use_nn_model:
                self.filter.predict_by_nn_model(self.model, self.control)
            else:
                self.filter.predict_by_naive_model(self.control)
            # Measurement update step
            self.filter.update_odom(self.z_odom, self.R_odom)
            if self.z_imu is not None:
                self.filter.update_imu(self.z_imu, self.R_imu)
            # if self.z_gyro is not None:
            if self.z_gyro is not None:
                self.filter.update_imu_gyro(self.z_gyro, self.R_gyro)
            if self.z_accel is not None:
                self.filter.update_imu_accel(self.z_accel, self.R_accel)
            # Transfer vectors to odometry messages
            self.state_to_odometry(self.filter.x_opt, self.filter.P_opt)
            self.pose_pub.publish(self.odom_filtered)

    def get_extrinsic(self, frame1, frame2):
        '''
        Parameters:
        frame1 (str): tf frame
        frame2 (str): tf frame

        Returns:
        np.array of shape [3, 4]: rotation-translation matrix between two tf frames.
        '''
        extrinsic = None
        try:
            # t = self.get_clock().now()
            # rclpy.spin_once(self)
            t = Namespace(seconds=0, nanoseconds=0)
            trans = self.tf_buffer.lookup_transform(frame1, frame2, t, rclpy.duration.Duration(seconds=0.5))
            # print(f"Got transform! {frame1} -> {frame2}")
            # break
            tr = np.array([
                [trans.transform.translation.x],
                [trans.transform.translation.y],
                [trans.transform.translation.z],
            ])
            rot_q = np.array([
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w,
            ])
            rot = R.from_quat(rot_q).as_matrix()
            extrinsic = np.concatenate([rot, tr], 1)
        except tf2_ros.LookupException:
            # rclpy.spin_once(self)
            print(f"Retrying to get transform {frame1} -> {frame2}", self.get_clock().now())
        
        return extrinsic


    def state_to_odometry(self, x, P):
        """
        Transfer filtered state vectors to odometry message
        @ parameters
        x: np.array
            State vector
        P: np.array
            Covariance matrix
        """
        self.odom_filtered.header.frame_id = 'odom'
        self.odom_filtered.header.stamp = self.get_clock().now().to_msg()
        # self.odom_filtered.header.seq = 0
        self.odom_filtered.child_frame_id = 'base_link'
        self.odom_filtered.pose.pose.position.x = x[0]
        self.odom_filtered.pose.pose.position.y = x[1]
        self.odom_filtered.pose.pose.position.z = 0.0
        # Transfer yaw angle from euler to quaternion
        r = R.from_euler('z', x[3], degrees=False)
        q = r.as_quat()
        self.odom_filtered.pose.pose.orientation.x = q[0]
        self.odom_filtered.pose.pose.orientation.y = q[1]
        self.odom_filtered.pose.pose.orientation.z = q[2]
        self.odom_filtered.pose.pose.orientation.w = q[3]
        self.odom_filtered.twist.twist.linear.x = x[2]
        if self.z_accel is not None:
            self.odom_filtered.twist.twist.linear.y = self.odom_noised.linear.y
            self.odom_filtered.twist.twist.linear.z = self.odom_noised.linear.z
            self.odom_filtered.twist.twist.angular.x = self.odom_noised.angular.x
            self.odom_filtered.twist.twist.angular.y = self.odom_noised.angular.y
        else:
            self.odom_filtered.twist.twist.linear.y = self.odom_noised.twist.twist.linear.y
            self.odom_filtered.twist.twist.linear.z = self.odom_noised.twist.twist.linear.z
            self.odom_filtered.twist.twist.angular.x = self.odom_noised.twist.twist.angular.x
            self.odom_filtered.twist.twist.angular.y = self.odom_noised.twist.twist.angular.y
        self.odom_filtered.twist.twist.angular.z = x[4]
        # Fill the odometry message covariance matrix with computed KF covariance

        t = tf2_ros.TransformStamped()
        t.header.frame_id = 'odom'
        t.header.stamp = self.odom_filtered.header.stamp
        t.child_frame_id = 'base_link'
        t.transform.translation.x = x[0]
        t.transform.translation.y = x[1]
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf2_broadcaster.sendTransform(t)

    def pose_covariance_to_vector(self, P):
        """
        Transfer filter computed pose covariance matrix to vector
        http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseWithCovariance.html
        @ parameters
        P: np.array of shape (dim(x), dim(x))
            Covariance matrix for corresponding variables of state vector
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
        P: np.array of shape (dim(x), dim(x))
            Covariance matrix for corresponding variables of state vector
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
        print("Distance:", state_estimator.distance)
        print("ATE / distance:", float(ate)/state_estimator.distance)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
