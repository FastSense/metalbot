import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
import nnio
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import tf2_ros
import time
from argparse import Namespace

from . import filter
from . import geometry

class StateEstimation(Node):
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
    pose_publisher: ros::Publisher
        Publish filtered odometry to /odom_filtered
    tf2_broadcaster: ros::TransformBroadcaster
    tf2_buffer: 
    tf_listener:
        Variables for obtaining transforms between frames
    model_path: str
        Path to file with NN control model
    model: ONNXModel
        Pretrained NN control model
    odom: Odometry
        Noised odometry from odom_noiser node
    imu: Imu
        imu message from accelerometer and gyroscope
    z_odom:
        Vector for odometry measurements
    R_odom:
        Covariance matrix of odometry measurements
    z_rot_vel_imu:
        Gyro measurement
    R_rot_vel_imu:
        Covariance matrix of gyro measurement
    z_acc_imu:
        Accelerometer measurement
    R_acc_imu:
        Covariance matrix of accelerometer measurement
    imu_acc_extrinsic:
        Accelerometer extrinsic matrix 
    control:
        Control velocity vector
    dt: float
        Time period for update step
    vel_std:
        Std for linear velocity noise
    rot_vel_std:
        Std for angular velocity noise
    filter: Filter2D
        EKF
    """
    def __init__(self):
        super().__init__('state_estimation_25d')

        self.odom_sub = self.create_subscription(
            Odometry,
            'velocity',
            self.odometry_callback,
            10,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10,
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.control_callback,
            15,
        )
        self.pose_publisher = self.create_publisher(
            Odometry,
            'odom_filtered',
            10,
        )
        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.model_path = 'http://192.168.194.51:8345/ml-control/gz-rosbot/new_model_dynamic_batch.onnx'
        self.model = nnio.ONNXModel(self.model_path)

        self.odom = None
        self.imu = None
        self.z_odom = None
        self.R_odom = None
        self.z_rot_vel_imu = None
        self.R_rot_vel_imu = None
        self.z_acc_imu = None
        self.R_acc_imu = None
        self.imu_acc_extrinsic = None
        self.imu_gyro_extrinsic = None
        self.control = np.zeros(2)
        self.dt = 0.1
        self.vel_std = 1.0
        self.rot_vel_std = 0.1
        self.filter = filter.Filter(
            self.dt,
            self.vel_std,
            self.rot_vel_std,
        )

        self.create_timer(self.dt, self.step)

    def odometry_callback(self, msg):
        """
        Callback from /odom_noised topic
        @ parameters
        msg: Odometry
        """
        self.odom = msg
        self.set_odometry_measurement()
    
    def imu_callback(self, msg):
        """
        Callback from /imu topic
        @ parameters
        msg: Imu
        """
        if msg.header.frame_id == 'oakd_imu':
            self.imu = msg
            self.set_imu_measurement()
    
    def control_callback(self, msg):
        """
        Callback from /cmd_vel topic with control commands
        @ parameters
        msg: Twist
        """
        self.control = np.array([msg.linear.x, msg.angular.z])

    def step(self):
        """
        Kalman filter iteration
        """
        time_start = time.time()
        # Predict step
        self.filter.predict_by_nn_model(self.model, self.control)
        if self.odom is not None:
            # Update odometry
            self.filter.update_odom(self.z_odom, self.R_odom)
        if self.imu is not None:
            # Update imu
            self.get_imu_extrinsic()
            self.filter.update_imu(self.z_acc_imu, self.R_acc_imu, self.imu_acc_extrinsic,
                                   self.z_rot_vel_imu, self.R_rot_vel_imu, self.imu_gyro_extrinsic)
        # Publish filtered pose
        time_end = time.time()
        print(time_end - time_start)
        self.publish_pose()

    def get_imu_extrinsic(self):
        self.imu_acc_extrinsic = self.get_extrinsic('oakd_imu', 'base_link')
        self.imu_gyro_extrinsic = self.get_extrinsic('oakd_imu', 'base_link')

    def set_odometry_measurement(self):
        """
        Make odometry measurement vector from Odometry ros message
        """
        self.z_odom = np.array([
            self.odom.linear.x,
            self.odom.angular.z,
        ])
        self.R_odom = np.array([
            [0.01, 0],
            [0, 0.001],
        ])

    def set_imu_measurement(self):
        """
        Make imu measurement vector from Imu ros message
        """
        # Make KF-compatible measurements
        self.z_rot_vel_imu = np.array([self.imu.angular_velocity.x,
                                       self.imu.angular_velocity.y,
                                       self.imu.angular_velocity.z])
        self.z_acc_imu = np.array([
            self.imu.linear_acceleration.x,
            self.imu.linear_acceleration.y,
            self.imu.linear_acceleration.z,
        ])
        self.R_rot_vel_imu = np.eye(3) * 0.01**2
        self.R_acc_imu = np.array([
            [0.1, 0, 0],
            [0, 0.1, 0],
            [0, 0, 0.1]
        ])
    
    def get_extrinsic(self, frame1, frame2):
        '''
        Parameters:
        frame1 (str): tf frame
        frame2 (str): tf frame

        Returns:
        np.array of shape [3, 4]: rotation-translation matrix between two tf frames.
        '''
        while True:
            try:
                t = Namespace(seconds=0, nanoseconds=0)
                trans = self.tf_buffer.lookup_transform(frame1, frame2, t, rclpy.duration.Duration(seconds=10))
                print(f"Got transform! {frame1} -> {frame2}")
                break
            except tf2_ros.LookupException:
                print(f"Retrying to get transform {frame1} -> {frame2}", self.get_clock().now())

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
        rot = Rotation.from_quat(rot_q).as_matrix()
        extrinsic = np.concatenate([rot, tr], 1)
        return extrinsic

    def publish_pose(self):
        # Make odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'state_estimation'
        # Position
        msg.pose.pose.position.x = self.filter.pos[0]
        msg.pose.pose.position.y = self.filter.pos[1]
        msg.pose.pose.position.z = self.filter.pos[2]
        # Angle
        msg.pose.pose.orientation.w = self.filter.q[0]
        msg.pose.pose.orientation.x = self.filter.q[1]
        msg.pose.pose.orientation.y = self.filter.q[2]
        msg.pose.pose.orientation.z = self.filter.q[3]
        # Pose & angle covariance
        msg.pose.covariance = self.filter.get_pose_covariance()
        # Velocity
        vel_local = np.array([self.filter.v, 0, 0])
        rot_mat = geometry.quat_as_matrix(self.filter.q)
        vel_global = rot_mat.T @ vel_local
        msg.twist.twist.linear.x = vel_global[0]
        msg.twist.twist.linear.y = vel_global[1]
        msg.twist.twist.linear.z = vel_global[2]
        # Angular velocity
        msg.twist.twist.angular.x = self.filter.rot_vel[0]
        msg.twist.twist.angular.y = self.filter.rot_vel[1]
        msg.twist.twist.angular.z = self.filter.rot_vel[2]
        # Vel & angvel covariance
        msg.twist.covariance = self.filter.get_twist_covariance()
        # Publish
        self.pose_publisher.publish(msg)

        # Broadcast tf2
        t = tf2_ros.TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'state_estimation'
        t.transform.translation.x = self.filter.pos[0]
        t.transform.translation.y = self.filter.pos[1]
        t.transform.translation.z = self.filter.pos[2]
        t.transform.rotation.w = self.filter.q[0]
        t.transform.rotation.x = self.filter.q[1]
        t.transform.rotation.y = self.filter.q[2]
        t.transform.rotation.z = self.filter.q[3]
        self.tf2_broadcaster.sendTransform(t)

def main(args=None):
    print('Hi from state_estimation_25d.')

    rclpy.init(args=args)
    node = StateEstimation()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
