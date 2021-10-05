import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import depthai as dai
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import tf2_ros


class DummyNode(Node):
    def __init__(self):
        super().__init__('oakd')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('device_id', '14442C1051BDC2D200') # rosbot camera by default
        self.declare_parameter('fps', 30)
        self.declare_parameter('imu_freq', 100)
        self.declare_parameter('publish_left', False)
        self.declare_parameter('publish_right', False)
        self.declare_parameter('publish_rgb', True)
        self.declare_parameter('publish_rect', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_depth', True)

        # Get parameters
        device_id = self.get_parameter('device_id').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.imu_freq = self.get_parameter('imu_freq').get_parameter_value().integer_value
        self.publish_left = self.get_parameter('publish_left').get_parameter_value().bool_value
        self.publish_right = self.get_parameter('publish_right').get_parameter_value().bool_value
        self.publish_rgb = self.get_parameter('publish_rgb').get_parameter_value().bool_value
        self.publish_rect = self.get_parameter('publish_rect').get_parameter_value().bool_value
        self.publish_imu = self.get_parameter('publish_imu').get_parameter_value().bool_value
        self.publish_depth = self.get_parameter('publish_depth').get_parameter_value().bool_value

        # Create publishers
        self.params_publisher = self.create_publisher(CameraInfo, 'rectified_camera_info', 10)
        if self.publish_left:
            self.left_publisher = self.create_publisher(Image, 'left', 10)
        if self.publish_right:
            self.right_publisher = self.create_publisher(Image, 'right', 10)
        if self.publish_rgb:
            self.rgb_publisher = self.create_publisher(Image, 'rgb', 10)
        if self.publish_rect:
            self.left_rect_publisher = self.create_publisher(Image, 'left_rect', 10)
            self.right_rect_publisher = self.create_publisher(Image, 'right_rect', 10)
        if self.publish_imu:
            self.imu_publisher = self.create_publisher(Imu, 'imu', 10)
        if self.publish_depth:
            self.depth_publisher = self.create_publisher(Image, 'depth', 10)

        # Create timer
        if self.publish_left:
            self.create_timer(1 / self.fps, self.timer_callback_left)
        if self.publish_right:
            self.create_timer(1 / self.fps, self.timer_callback_right)
        if self.publish_rgb:
            self.create_timer(1 / self.fps, self.timer_callback_rgb)
        if self.publish_rect:
            self.create_timer(1 / self.fps, self.timer_callback_rect_left)
            self.create_timer(1 / self.fps, self.timer_callback_rect_right)
        if self.publish_imu:
            self.create_timer(1 / self.imu_freq, self.timer_callback_imu)
        if self.publish_depth:
            self.create_timer(1 / self.fps, self.timer_callback_depth)
        self.calib_rect_msg = None
        self.create_timer(1, self.publish_camera_parameters)
        # self.create_timer(1, self.publish_transforms)
        self.publish_transforms()
        self.min_delta = None

        # Covariance matrix
        std_accel = 0.01
        self.covariance_accel = list((np.eye(3) * std_accel**2).flatten())
        std_rotvel = 0.01
        self.covariance_rotvel = list((np.eye(3) * std_rotvel**2).flatten())

    def timer_callback_left(self):
        frame = np.random.randint(0, 255, size=(400, 640), dtype='uint8')
        msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
        msg.header.frame_id = 'oakd_left'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.left_publisher.publish(msg)

    def timer_callback_right(self):
        frame = np.random.randint(0, 255, size=(400, 640), dtype='uint8')
        msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
        msg.header.frame_id = 'oakd_right'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.right_publisher.publish(msg)

    def timer_callback_rgb(self):
        frame = np.random.randint(0, 255, size=(400, 640, 3), dtype='uint8')
        msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
        msg.header.frame_id = 'oakd_rgb'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.rgb_publisher.publish(msg)

    def timer_callback_rect_left(self):
        frame = np.random.randint(0, 255, size=(400, 640), dtype='uint8')
        msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
        msg.header.frame_id = 'oakd_left'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.left_rect_publisher.publish(msg)

    def timer_callback_rect_right(self):
        frame = np.random.randint(0, 255, size=(400, 640), dtype='uint8')
        msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
        msg.header.frame_id = 'oakd_right'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.right_rect_publisher.publish(msg)

    def timer_callback_imu(self):
        # Publish an IMU message
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'oakd_imu'
        msg.angular_velocity.x = np.random.normal()
        msg.angular_velocity.y = np.random.normal()
        msg.angular_velocity.z = np.random.normal()
        msg.angular_velocity_covariance = self.covariance_rotvel
        msg.linear_acceleration.x = np.random.normal()
        msg.linear_acceleration.y = np.random.normal()
        msg.linear_acceleration.z = np.random.normal()
        msg.linear_acceleration_covariance = self.covariance_accel
        self.imu_publisher.publish(msg)

    def timer_callback_depth(self):
        frame = np.random.randint(0, 255**2, size=(400, 640), dtype='uint16')
        msg = self.bridge.cv2_to_imgmsg(frame, 'mono16')
        msg.header.frame_id = 'oakd_left'
        msg.header.stamp = self.get_clock().now().to_msg()
        self.depth_publisher.publish(msg)

    def publish_camera_parameters(self):
        if self.calib_rect_msg is None:
            # Get calibration parameters
            print('Left intrinsics:')
            M1 = np.random.normal(size=(3, 3))
            print(M1)
            print('Left distortion:')
            d1 = np.random.normal(size=4)
            print(d1)
            print('Right intrinsics:')
            M2 = np.random.normal(size=(3, 3))
            print(M2)
            print('Right distortion:')
            d2 = np.random.normal(size=4)
            print(d2)
            print('RGB intrinsics:')
            M3 = np.random.normal(size=(3, 3))
            print(M3)
            print('RGB distortion:')
            d3 = np.random.normal(size=4)
            print(d3)

            self.calib_rect_msg = CameraInfo()
            self.calib_rect_msg.header.frame_id = 'oakd_left'
            self.calib_rect_msg.header.stamp = self.get_clock().now().to_msg()
            self.calib_rect_msg.height = 800
            self.calib_rect_msg.width = 1280
            self.calib_rect_msg.k = [float(num) for num in M2.flatten()]
            self.calib_rect_msg.r = [float(num) for num in np.eye(3).flatten()]
            P = np.zeros([3, 4])
            P[:3, :3] = M2
            P[0, 3] = -M2[0, 0] * 0.075
            self.calib_rect_msg.p = [float(num) for num in P.flatten()]
        
        self.params_publisher.publish(self.calib_rect_msg)
    
    def publish_transforms(self):
        stamp = self.get_clock().now().to_msg()

        # TF transforms
        # Robot to camera body
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'base_link'
        tf.header.stamp = stamp
        tf.child_frame_id = 'oakd'
        camera_elevation = 7.5 * np.pi / 180 # 5 degrees
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = float(np.sin(-camera_elevation / 2))
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = float(np.cos(-camera_elevation / 2))
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # Camera body to imu
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd'
        tf.header.stamp = stamp
        tf.child_frame_id = 'oakd_imu'
        rot_mat = np.array([
            [0., 0.,-1.],
            [0., 1., 0.],
            [1., 0., 0.],
        ]).T
        rot_q = Rotation.from_matrix(rot_mat).as_quat()
        tf.transform.rotation.x = rot_q[0]
        tf.transform.rotation.y = rot_q[1]
        tf.transform.rotation.z = rot_q[2]
        tf.transform.rotation.w = rot_q[3]
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # Camera body to RGB eye
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd'
        tf.header.stamp = stamp
        tf.child_frame_id = 'oakd_rgb'
        rot_mat = np.array([
            [0.,-1., 0],
            [0., 0.,-1.],
            [1., 0., 0.],
        ]).T
        rot_q = Rotation.from_matrix(rot_mat).as_quat()
        tf.transform.rotation.x = rot_q[0]
        tf.transform.rotation.y = rot_q[1]
        tf.transform.rotation.z = rot_q[2]
        tf.transform.rotation.w = rot_q[3]
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # RGB to left
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd_rgb'
        tf.header.stamp = stamp
        tf.child_frame_id = 'oakd_left'
        tf.transform.translation.x = -0.075 * 0.5
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # RGB to Right
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd_rgb'
        tf.header.stamp = stamp
        tf.child_frame_id = 'oakd_right'
        tf.transform.translation.x = 0.075 * 0.5
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)


def main(args=None):
    print('Hi from oakd.')

    rclpy.init(args=args)

    node = DummyNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
