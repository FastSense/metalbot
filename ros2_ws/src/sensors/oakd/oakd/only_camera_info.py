import numpy as np
import cv2
from scipy.spatial.transform import Rotation
import depthai as dai

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from cv_bridge import CvBridge
import tf2_ros


class OnlyInfoNode(Node):
    def __init__(self):
        super().__init__('oakd')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('device_id', '14442C1051BDC2D200') # rosbot camera by default

        # Get parameters
        device_id = self.get_parameter('device_id').get_parameter_value().string_value

        # Create publishers
        self.params_publisher = self.create_publisher(CameraInfo, 'rectified_camera_info', 10)
        self.calib_rect_msg = None

        # Create timer
        self.create_timer(1, self.publish_camera_parameters)
        self.min_delta = None

        self.initialize_device(device_id)


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

    node = OnlyInfoNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
