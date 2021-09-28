import cv2
import nnio
import numpy as np
from scipy.spatial.transform import Rotation
from argparse import Namespace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros

from .spacekf import SpaceKF12
from perception_msgs.msg import OdoFlow
from optical_flow.stereo_camera import StereoCamera


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_3d')

        self.bridge = CvBridge()

        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('period', 0.02)
        self.declare_parameter('vel_std', 1.0)
        self.declare_parameter('rot_vel_std', 1.0)

        # Kalman filter parameters
        vel_std = self.get_parameter('vel_std').get_parameter_value().double_value
        rot_vel_std = self.get_parameter('rot_vel_std').get_parameter_value().double_value

        # Get camera parameters
        self.stereo = None

        # Subscribe to sensor topics
        self.create_subscription(
            OdoFlow,
            'odom_flow',
            self.odometry_callback,
            10,
        )
        self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10,
        )
        self.create_subscription(
            CameraInfo,
            'rectified_camera_info',
            self.calibration_callback,
            10,
        )

        # Publisher
        self.pose_publisher = self.create_publisher(
            Odometry,
            'pose_ekf',
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
        self.imu_count = 0
        self.odom_buffer = None

        # TF listener
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odometry_callback(self, msg):
        self.odom_buffer = msg

    def imu_callback(self, msg):
        if not 'oakd' in msg.header.frame_id:
            # print('Wrong imu frame:', msg.header.frame_id)
            return

        if self.imu_buffer is None:
            self.imu_buffer = msg
            self.imu_count = 1
        else:
            self.imu_buffer.angular_velocity.x += msg.angular_velocity.x
            self.imu_buffer.angular_velocity.y += msg.angular_velocity.y
            self.imu_buffer.angular_velocity.z += msg.angular_velocity.z
            self.imu_buffer.linear_acceleration.x += msg.linear_acceleration.x
            self.imu_buffer.linear_acceleration.y += msg.linear_acceleration.y
            self.imu_buffer.linear_acceleration.z += msg.linear_acceleration.z
            self.imu_count += 1

    def step(self):
        '''
        EKF predict and update step
        '''
        # Predict
        self.tracker.predict()
        # Update
        # rclpy.spin_once(self)
        if self.imu_buffer is not None:
            self.update_imu(self.imu_buffer)
            self.imu_buffer = None
        if self.odom_buffer is not None:
            self.update_odom_flow(self.odom_buffer)
            self.odom_buffer = None
        # Publish
        self.publish_pose()

    def update_imu(self, msg):
        '''
        Update filter state using IMU message
        '''

        # Make KF-compatible measurements
        rot_vel = np.empty(3)
        rot_vel[0] = msg.angular_velocity.x / self.imu_count
        rot_vel[1] = msg.angular_velocity.y / self.imu_count
        rot_vel[2] = msg.angular_velocity.z / self.imu_count
        rot_vel_R = np.array(msg.angular_velocity_covariance).reshape([3, 3])
        acc = np.empty(3)
        acc[0] = msg.linear_acceleration.x / self.imu_count
        acc[1] = msg.linear_acceleration.y / self.imu_count
        acc[2] = msg.linear_acceleration.z / self.imu_count
        acc_R = np.array(msg.linear_acceleration_covariance).reshape([3, 3])

        # Get extrinsics from tf
        extrinsic = self.get_extrinsic(msg.header.frame_id, 'base_link')

        # Update
        self.tracker.update_acc(acc, acc_R, extrinsic=extrinsic)
        self.tracker.update_rot_vel(rot_vel, rot_vel_R, extrinsic=extrinsic)

    def update_odom_flow(self, msg):
        '''
        Update filter state using flow odometry message
        '''
        if self.stereo is None:
            print('waiting for camera parameters...')
            return

        z = np.vstack([msg.flow_x, msg.flow_y, msg.delta_depth]).transpose() # [N, 3]
        pixels = np.vstack([msg.x, msg.y]).transpose() # [N, 2]
        R = np.diag(msg.covariance_diag)

        # Get extrinsics from tf
        extrinsic = self.get_extrinsic(msg.header.frame_id, 'base_link')

        # Compute time delay
        msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_stamp = self.get_clock().now().seconds_nanoseconds()
        ros_time = ros_stamp[0] + ros_stamp[1] * 1e-9
        delay = ros_time - msg_time
        print('odom delay:', delay)

        self.tracker.update_flow(
            z,
            msg.delta_t,
            msg.depth,
            pixels,
            R,
            self.stereo.M1,
            self.stereo.M1_inv,
            extrinsic=extrinsic,
            delay=delay*0,
        )

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
                # t = self.get_clock().now()
                # rclpy.spin_once(self)
                t = Namespace(seconds=0, nanoseconds=0)
                trans = self.tf_buffer.lookup_transform(frame1, frame2, t, rclpy.duration.Duration(seconds=10))
                # print(f"Got transform! {frame1} -> {frame2}")
                break
            except tf2_ros.LookupException:
                # rclpy.spin_once(self)
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
        msg.child_frame_id = 'base_link'
        # Position
        msg.pose.pose.position.x = self.tracker.pos[0]
        msg.pose.pose.position.y = self.tracker.pos[1]
        msg.pose.pose.position.z = self.tracker.pos[2]
        # Angle
        msg.pose.pose.orientation.x = self.tracker.q[0]
        msg.pose.pose.orientation.y = self.tracker.q[1]
        msg.pose.pose.orientation.z = self.tracker.q[2]
        msg.pose.pose.orientation.w = self.tracker.q[3]
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
        t.transform.rotation.x = self.tracker.q[0]
        t.transform.rotation.y = self.tracker.q[1]
        t.transform.rotation.z = self.tracker.q[2]
        t.transform.rotation.w = self.tracker.q[3]
        self.tf2_broadcaster.sendTransform(t)

    def calibration_callback(self, msg):
        if self.stereo is None:
            M1 = np.array(msg.k).reshape([3, 3])
            M2 = M1
            T = [msg.p[3] / msg.k[0], msg.p[7] / msg.k[0], msg.p[11] / msg.k[0]]
            R = np.array(msg.r).reshape([3, 3])
            print('T', T)
            self.stereo = StereoCamera(
                M1=M1, M2=M2, R=R, T=T, image_h=msg.height, image_w=msg.width
            )
            self.stereo.change_dimensions_(128, 128)


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
