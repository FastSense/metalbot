import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import depthai as dai

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import tf2_ros


class OAKDNode(Node):
    def __init__(self):
        super().__init__('oakd')

        self.bridge = CvBridge()

        # TF transforms
        # Robot to camera body
        tf_static_pub = tf2_ros.StaticTransformBroadcaster(self)
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'oakd'
        tf_static_pub.sendTransform(tf)
        # Camera body to accel
        tf_static_pub = tf2_ros.StaticTransformBroadcaster(self)
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd'
        tf.child_frame_id = 'oakd_accel'
        rot_mat = np.array([
            [0., 1., 0.],
            [0., 0., 1.],
            [1., 0., 0.],
        ]).T
        rot_q = Rotation.from_matrix(rot_mat).as_quat()
        tf.transform.rotation.x = rot_q[0]
        tf.transform.rotation.y = rot_q[1]
        tf.transform.rotation.z = rot_q[2]
        tf.transform.rotation.w = rot_q[3]
        tf_static_pub.sendTransform(tf)
        # Camera body to imu
        tf_static_pub = tf2_ros.StaticTransformBroadcaster(self)
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd'
        tf.child_frame_id = 'oakd_gyro'
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
        tf_static_pub.sendTransform(tf)
        # Camera body to left eye
        tf_static_pub = tf2_ros.StaticTransformBroadcaster(self)
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd'
        tf.child_frame_id = 'oakd_left'
        tf.transform.translation.y = 0.075 * 0.5
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
        tf_static_pub.sendTransform(tf)
        # Left to Right
        tf_static_pub = tf2_ros.StaticTransformBroadcaster(self)
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd_left'
        tf.child_frame_id = 'oakd_right'
        tf.transform.translation.x = 0.075
        tf_static_pub.sendTransform(tf)

        # Create publishers
        self.left_publisher = self.create_publisher(Image, 'left', 10)
        self.right_publisher = self.create_publisher(Image, 'right', 10)
        self.left_rect_publisher = self.create_publisher(Image, 'left_rect', 10)
        self.right_rect_publisher = self.create_publisher(Image, 'right_rect', 10)
        # self.rgb_publisher = self.create_publisher(Image, 'rgb', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu', 10)

        # Create timer
        self.create_timer(1e-2, self.timer_callback)
        self.min_delta = None

        self.initialize_device()

        # Covariance matrix
        std_accel = 0.1
        self.covariance_accel = list((np.eye(3) * std_accel**2).flatten())
        std_rotvel = 0.1
        self.covariance_rotvel = list((np.eye(3) * std_rotvel**2).flatten())

    def timer_callback(self):
        try:
            self.check_queues()
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()

    def initialize_device(self):
        # Start defining a pipeline
        pipeline = dai.Pipeline()

        # Left camera
        camLeft = pipeline.createMonoCamera()
        camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        # Outputs
        xoutLeft = pipeline.createXLinkOut()
        xoutLeft.setStreamName('left')
        camLeft.out.link(xoutLeft.input)

        # Right camera
        camRight = pipeline.createMonoCamera()
        camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        # Outputs
        xoutRight = pipeline.createXLinkOut()
        xoutRight.setStreamName('right')
        camRight.out.link(xoutRight.input)

        # RGB Camera
        # camRgb = pipeline.createColorCamera()
        # camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
        # # camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)
        # RGB output
        # xoutRgb = pipeline.createXLinkOut()
        # xoutRgb.setStreamName('rgb')
        # camRgb.video.link(xoutRgb.input)

        # Depth and rectificetion
        depth = pipeline.createStereoDepth()
        camLeft.out.link(depth.left)
        camRight.out.link(depth.right)
        # Left rectified outputs
        xoutLeft = pipeline.createXLinkOut()
        xoutLeft.setStreamName('left_rect')
        depth.rectifiedLeft.link(xoutLeft.input)
        # Right rectified outputs
        xoutRight = pipeline.createXLinkOut()
        xoutRight.setStreamName('right_rect')
        depth.rectifiedRight.link(xoutRight.input)

        # IMU
        imu = pipeline.createIMU()
        # enable ACCELEROMETER_RAW and GYROSCOPE_RAW at given rate
        imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
        # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
        imu.setBatchReportThreshold(1)
        # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
        # if lower or equal to batchReportThreshold then the sending is always blocking on device
        # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
        imu.setMaxBatchReports(10)
        # Output
        xoutImu = pipeline.createXLinkOut()
        xoutImu.setStreamName("imu")
        imu.out.link(xoutImu.input)

        # Pipeline is defined, now we can connect to the device
        self.device = dai.Device(pipeline)
        # Start pipeline
        self.device.startPipeline()

        # Output queues will be used to get the grayscale frames from the outputs defined above
        self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
        self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
        self.q_left_rect = self.device.getOutputQueue(name="left_rect", maxSize=4, blocking=False)
        self.q_right_rect = self.device.getOutputQueue(name="right_rect", maxSize=4, blocking=False)
        # self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.q_imu = self.device.getOutputQueue(name="imu", maxSize=4, blocking=False)

    def check_queues(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()

        # Instead of get (blocking), we use tryGet (nonblocking) which will return the available data or None otherwise

        # Left image
        in_left = self.q_left.tryGet()
        if in_left is not None:
            frame = in_left.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            msg.header.frame_id = 'oakd_left'
            ts = in_left.getTimestamp()
            msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
            self.left_publisher.publish(msg)

        # Right image
        in_right = self.q_right.tryGet()
        if in_right is not None:
            frame = in_right.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            msg.header.frame_id = 'oakd_right'
            ts = in_right.getTimestamp()
            msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
            self.right_publisher.publish(msg)

        # Left rectified image
        in_left_rect = self.q_left_rect.tryGet()
        if in_left_rect is not None:
            frame = in_left_rect.getCvFrame()[:, ::-1]
            msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            msg.header.frame_id = 'oakd_left'
            ts = in_left_rect.getTimestamp()
            msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
            self.left_rect_publisher.publish(msg)

        # Right rectified image
        in_right_rect = self.q_right_rect.tryGet()
        if in_right_rect is not None:
            frame = in_right_rect.getCvFrame()[:, ::-1]
            msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            msg.header.frame_id = 'oakd_right'
            ts = in_right_rect.getTimestamp()
            msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
            self.right_rect_publisher.publish(msg)

        # RGB image
        # in_rgb = self.q_rgb.tryGet()
        # if in_rgb is not None:
        #     frame = in_rgb.getCvFrame()[:,:,::-1]
        #     msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
        #     msg.header.frame_id = 'oakd_rgb'
        #     ts = in_rgb.getTimestamp()
        #     msg.header.stamp = self.get_time(ts)
        #     self.rgb_publisher.publish(msg)

        # IMU
        in_imu = self.q_imu.tryGet()
        if in_imu is not None:
            imuPackets = in_imu.packets
            for imuPacket in imuPackets:
                # Get data
                accelero_values = imuPacket.acceleroMeter
                gyro_values = imuPacket.gyroscope
                # accelero_ts = acceleroValues.timestamp.get()
                gyro_ts = gyro_values.timestamp.get()
                # Publish an IMU message
                msg = Imu()
                msg.header.stamp = self.get_corrected_time(gyro_ts, ros_stamp)
                msg.header.frame_id = 'oakd_imu'
                msg.angular_velocity.x = gyro_values.x
                msg.angular_velocity.y = gyro_values.y
                msg.angular_velocity.z = gyro_values.z
                msg.angular_velocity_covariance = self.covariance_rotvel
                msg.linear_acceleration.x = accelero_values.x
                msg.linear_acceleration.y = accelero_values.y
                msg.linear_acceleration.z = accelero_values.z
                msg.linear_acceleration_covariance = self.covariance_accel
                self.imu_publisher.publish(msg)

    def get_corrected_time(self, oakd_timestamp, ros_stamp):
        # Compute time delta
        stamp_seconds = ros_stamp.nanoseconds * 1e-9
        hw_seconds = oakd_timestamp.total_seconds()
        # Current delta
        delta = stamp_seconds - hw_seconds
        # Minimun delta
        self.min_delta = self.min_delta or delta
        self.min_delta = min(self.min_delta, delta)
        # assert abs(delta - self.min_delta) < 10
        seconds = hw_seconds + self.min_delta
        # Convert to message
        msg = ros_stamp.to_msg()
        msg.sec = int(seconds)
        msg.nanosec = int((seconds % 1) * 1e9)
        return msg


def main(args=None):
    print('Hi from oakd.')

    rclpy.init(args=args)

    node = OAKDNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
