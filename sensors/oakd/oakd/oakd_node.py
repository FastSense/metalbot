import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import depthai as dai

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import tf2_ros


class OAKDNode(Node):
    def __init__(self):
        super().__init__('oakd')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('device_id', '14442C1051BDC2D200') # rosbot camera by default
        self.declare_parameter('fps', 10)
        self.declare_parameter('publish_left', False)
        self.declare_parameter('publish_right', False)
        self.declare_parameter('publish_rgb', False)
        self.declare_parameter('publish_rect', True)
        self.declare_parameter('publish_imu', True)
        self.declare_parameter('publish_depth', True)

        # Get parameters
        device_id = self.get_parameter('device_id').get_parameter_value().string_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
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
            self.create_timer(1e-2, self.timer_callback_left)
        if self.publish_right:
            self.create_timer(1e-2, self.timer_callback_right)
        if self.publish_rgb:
            self.create_timer(1e-2, self.timer_callback_rgb)
        if self.publish_rect:
            self.create_timer(1e-2, self.timer_callback_rect_left)
            self.create_timer(1e-2, self.timer_callback_rect_right)
        if self.publish_imu:
            self.create_timer(1e-2, self.timer_callback_imu)
        if self.publish_depth:
            self.create_timer(1e-2, self.timer_callback_depth)
        self.calib_rect_msg = None
        self.create_timer(1, self.publish_camera_parameters)
        self.create_timer(1, self.publish_transforms)
        self.min_delta = None

        self.initialize_device(device_id)

        # Covariance matrix
        std_accel = 0.01
        self.covariance_accel = list((np.eye(3) * std_accel**2).flatten())
        std_rotvel = 0.01
        self.covariance_rotvel = list((np.eye(3) * std_rotvel**2).flatten())

    def timer_callback_left(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
            # Left image
            in_left = self.q_left.tryGet()
            if in_left is not None:
                frame = in_left.getCvFrame()
                msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
                msg.header.frame_id = 'oakd_left'
                ts = in_left.getTimestamp()
                msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
                self.left_publisher.publish(msg)
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()

    def timer_callback_right(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
            # Right image
            in_right = self.q_right.tryGet()
            if in_right is not None:
                frame = in_right.getCvFrame()
                msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
                msg.header.frame_id = 'oakd_right'
                ts = in_right.getTimestamp()
                msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
                self.right_publisher.publish(msg)
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()
    
    def timer_callback_rgb(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
            # RGB image
            in_rgb = self.q_rgb.tryGet()
            if in_rgb is not None:
                frame = in_rgb.getCvFrame()[:,:,::-1]
                msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
                msg.header.frame_id = 'oakd'
                ts = in_rgb.getTimestamp()
                msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
                self.rgb_publisher.publish(msg)
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()
    
    def timer_callback_rect_left(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
            # Left rectified image
            in_left_rect = self.q_left_rect.tryGet()
            if in_left_rect is not None:
                frame = in_left_rect.getCvFrame()[:, ::-1]
                msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
                msg.header.frame_id = 'oakd_left'
                ts = in_left_rect.getTimestamp()
                msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
                self.left_rect_publisher.publish(msg)
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()
    
    def timer_callback_rect_right(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
            # Right rectified image
            in_right_rect = self.q_right_rect.tryGet()
            if in_right_rect is not None:
                frame = in_right_rect.getCvFrame()[:, ::-1]
                msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
                msg.header.frame_id = 'oakd_right'
                ts = in_right_rect.getTimestamp()
                msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
                self.right_rect_publisher.publish(msg)
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()
    
    def timer_callback_imu(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
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
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()
    
    def timer_callback_depth(self):
        # ROS time stamp
        ros_stamp = self.get_clock().now()
        try:
            # Depth image
            in_depth = self.q_depth.tryGet()
            if in_depth is not None:
                frame = in_depth.getCvFrame()[:,::-1]
                msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
                msg.header.frame_id = 'oakd_left'
                ts = in_depth.getTimestamp()
                msg.header.stamp = self.get_corrected_time(ts, ros_stamp)
                self.depth_publisher.publish(msg)
        except RuntimeError as e:
            print(e)
            print('Restarting device')
            self.initialize_device()

    def publish_camera_parameters(self):
        if self.calib_rect_msg is None:
            # Get calibration parameters
            calib_data = self.device.readCalibration()
            print('Left intrinsics:')
            M1 = np.array(calib_data.getCameraIntrinsics(dai.CameraBoardSocket.LEFT))
            print(M1)
            print('Left distortion:')
            d1 = np.array(calib_data.getDistortionCoefficients(dai.CameraBoardSocket.LEFT))
            print(d1)
            print('Right intrinsics:')
            M2 = np.array(calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT))
            print(M2)
            print('Right distortion:')
            d2 = np.array(calib_data.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT))
            print(d2)
            print('RGB intrinsics:')
            M3 = np.array(calib_data.getCameraIntrinsics(dai.CameraBoardSocket.RGB))
            print(M3)
            print('RGB distortion:')
            d3 = np.array(calib_data.getDistortionCoefficients(dai.CameraBoardSocket.RGB))
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

    def initialize_device(self, device_id):
        # Start defining a pipeline
        pipeline = dai.Pipeline()

        if self.publish_left or self.publish_rect or self.publish_depth:
            # Left camera
            camLeft = pipeline.createMonoCamera()
            camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
            camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            camLeft.setFps(self.fps)
            if self.publish_left:
                # Outputs
                xoutLeft = pipeline.createXLinkOut()
                xoutLeft.setStreamName('left')
                camLeft.out.link(xoutLeft.input)

        if self.publish_right or self.publish_rect or self.publish_depth:
            # Right camera
            camRight = pipeline.createMonoCamera()
            camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
            camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
            camRight.setFps(self.fps)
            if self.publish_right:
                # Outputs
                xoutRight = pipeline.createXLinkOut()
                xoutRight.setStreamName('right')
                camRight.out.link(xoutRight.input)

        # RGB Camera
        if self.publish_rgb:
            camRgb = pipeline.createColorCamera()
            camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setFps(self.fps)
            # RGB output
            xoutRgb = pipeline.createXLinkOut()
            xoutRgb.setStreamName('rgb')
            camRgb.video.link(xoutRgb.input)

        # Depth and rectification
        if self.publish_rect or self.publish_depth:
            depth = pipeline.createStereoDepth()
            depth.initialConfig.setConfidenceThreshold(200)
            # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
            depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
            depth.setLeftRightCheck(False)
            depth.setExtendedDisparity(False)
            depth.setSubpixel(False)
            camLeft.out.link(depth.left)
            camRight.out.link(depth.right)
        if self.publish_rect:
            # Left rectified outputs
            xoutLeft = pipeline.createXLinkOut()
            xoutLeft.setStreamName('left_rect')
            depth.rectifiedLeft.link(xoutLeft.input)
            # Right rectified outputs
            xoutRight = pipeline.createXLinkOut()
            xoutRight.setStreamName('right_rect')
            depth.rectifiedRight.link(xoutRight.input)
        if self.publish_depth:
            # Depth image
            xoutDepth = pipeline.createXLinkOut()
            xoutDepth.setStreamName('depth')
            depth.disparity.link(xoutDepth.input)

        # IMU
        if self.publish_imu:
            imu = pipeline.createIMU()
            # Enable ACCELEROMETER_RAW and GYROSCOPE_RAW at given rate
            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW, dai.IMUSensor.GYROSCOPE_RAW], 200)
            # Above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
            imu.setBatchReportThreshold(1)
            # Maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
            # If lower or equal to batchReportThreshold then the sending is always blocking on device
            # Useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
            imu.setMaxBatchReports(10)
            # Output
            xoutImu = pipeline.createXLinkOut()
            xoutImu.setStreamName("imu")
            imu.out.link(xoutImu.input)

        # Take camera device
        device_info = None
        if device_id != '':
            found, device_info = dai.Device.getDeviceByMxId(device_id)
        if not found:
            print(f'Could not find device {device_id}. Trying to find any device')
            device_info = None

        # Pipeline is defined, now we can connect to the device
        self.device = dai.Device(pipeline, device_info)
        # Start pipeline
        self.device.startPipeline()

        # Output queues will be used to get the grayscale frames from the outputs defined above
        if self.publish_left:
            self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
        if self.publish_right:
            self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
        if self.publish_rgb:
            self.q_rgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        if self.publish_rect:
            self.q_left_rect = self.device.getOutputQueue(name="left_rect", maxSize=4, blocking=False)
            self.q_right_rect = self.device.getOutputQueue(name="right_rect", maxSize=4, blocking=False)
        if self.publish_imu:
            self.q_imu = self.device.getOutputQueue(name="imu", maxSize=4, blocking=False)
        if self.publish_depth:
            self.q_depth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        

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
    
    def publish_transforms(self):
        # TF transforms
        # Robot to camera body
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'base_link'
        tf.child_frame_id = 'oakd'
        camera_elevation = 7.5 * np.pi / 180 # 5 degrees
        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = float(np.sin(-camera_elevation / 2))
        tf.transform.rotation.z = 0.0
        tf.transform.rotation.w = float(np.cos(-camera_elevation / 2))
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # Camera body to accel
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
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # Camera body to imu
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
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # Camera body to RGB eye
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd'
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
        tf.child_frame_id = 'oakd_left'
        tf.transform.translation.x = -0.075 * 0.5
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)
        # RGB to Right
        tf = tf2_ros.TransformStamped()
        tf.header.frame_id = 'oakd_rgb'
        tf.child_frame_id = 'oakd_right'
        tf.transform.translation.x = 0.075 * 0.5
        tf2_ros.StaticTransformBroadcaster(self).sendTransform(tf)


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
