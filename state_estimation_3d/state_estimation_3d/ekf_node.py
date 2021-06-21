import cv2
import nnio
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros

from .spacekf import SpaceKF12

class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_3d')

        self.bridge = CvBridge()

        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('period', 0.01)
        self.declare_parameter('vel_std', 0.1)
        self.declare_parameter('rot_vel_std', 1.0)

        # Kalman filter parameters
        vel_std = self.get_parameter('vel_std').get_parameter_value().double_value
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
        self.tracker = SpaceKF12(dt=self.period, velocity_std=vel_std, rot_vel_std=rot_vel_std)
        self.tracker.P = self.tracker.P * 0.01

        # Buffers for measurements
        self.imu_buffer = None
        self.odom_buffer = None

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odometry_callback(self, msg):
        self.odom_buffer = msg

    def imu_callback(self, msg):
        self.imu_buffer = msg

    def step(self):
        '''
        EKF predict and update step
        '''
        # Predict
        self.tracker.predict()
        # Update
        if self.imu_buffer is not None:
            self.update_imu(self.imu_buffer)
            self.imu_buffer = None
        if self.odom_buffer is not None:
            self.update_odom(self.odom_buffer)
            self.odom_buffer = None
        # Publish
        self.publish_pose()

    def update_imu(self, msg):
        # extrinsic = np.empty(4)
        # extrinsic[0] = msg.header.orientation.w

        # Make KF-compatible measurements
        rot_vel = np.empty(3)
        rot_vel[0] = msg.angular_velocity.x
        rot_vel[1] = msg.angular_velocity.y
        rot_vel[2] = msg.angular_velocity.z
        rot_vel_R = np.array(msg.angular_velocity_covariance).reshape([3, 3])
        acc = np.empty(3)
        acc[0] = msg.linear_acceleration.x
        acc[1] = msg.linear_acceleration.y
        acc[2] = msg.linear_acceleration.z
        acc_R = np.array(msg.linear_acceleration_covariance).reshape([3, 3])

        # Get extrinsics from tf
        extrinsic_acc = self.get_extrinsic('oakd_accel', 'base_link')
        extrinsic_gyro = self.get_extrinsic('oakd_gyro', 'base_link')

        # Update
        self.tracker.update_acc(acc, acc_R, extrinsic=extrinsic_acc)
        self.tracker.update_rot_vel(rot_vel, rot_vel_R, extrinsic=extrinsic_gyro)

    def get_extrinsic(self, frame1, frame2):
        trans = self.tf_buffer.lookup_transform(frame1, frame2, self.get_clock().now())
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

    def update_odom(self, msg):
        pass

    def publish_pose(self):
        # Make odometry message
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = 'base_link'
        # Position
        msg.pose.pose.position.x = self.tracker.pos[0]
        msg.pose.pose.position.y = self.tracker.pos[1]
        msg.pose.pose.position.z = self.tracker.pos[2]
        # Angle
        msg.pose.pose.orientation.w = self.tracker.q[0]
        msg.pose.pose.orientation.x = self.tracker.q[1]
        msg.pose.pose.orientation.y = self.tracker.q[2]
        msg.pose.pose.orientation.z = self.tracker.q[3]
        # Pose & angle covariance
        msg.pose.covariance = self.tracker.get_pose_covariance()
        # Velocity
        msg.twist.twist.linear.x = self.tracker.vel[0]
        msg.twist.twist.linear.y = self.tracker.vel[1]
        msg.twist.twist.linear.z = self.tracker.vel[2]
        # Angular velocity
        msg.twist.twist.angular.x = self.tracker.rot_vel[0]
        msg.twist.twist.angular.y = self.tracker.rot_vel[1]
        msg.twist.twist.angular.z = self.tracker.rot_vel[2]
        # Vel & angvel covariance
        msg.twist.covariance = self.tracker.get_twist_covariance()
        # Publish
        self.pose_publisher.publish(msg)

        # Broadcast tf2
        t = tf2_ros.TransformStamped()
        t.header = msg.header
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.tracker.pos[0]
        t.transform.translation.y = self.tracker.pos[1]
        t.transform.translation.z = self.tracker.pos[2]
        t.transform.rotation.w = self.tracker.q[0]
        t.transform.rotation.x = self.tracker.q[1]
        t.transform.rotation.y = self.tracker.q[2]
        t.transform.rotation.z = self.tracker.q[3]
        self.tf2_broadcaster.sendTransform(t)


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
