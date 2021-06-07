import cv2
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

        # Create publishers
        self.left_publisher = self.create_publisher(Image, 'left', 10)
        self.right_publisher = self.create_publisher(Image, 'right', 10)
        # self.rgb_publisher = self.create_publisher(Image, 'rgb', 10)
        # self.imu_publisher = self.create_publisher(Imu, 'imu', 10)

        # Create timer
        self.create_timer(1e-3, self.check_queues)

        # Start defining a pipeline
        pipeline = dai.Pipeline()

        # Define a source - two mono (grayscale) cameras
        camLeft = pipeline.createMonoCamera()
        camLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
        camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        camRight = pipeline.createMonoCamera()
        camRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)
        camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)

        # Depth calculation
        depth = pipeline.createStereoDepth()

        camLeft.out.link(depth.left)
        camRight.out.link(depth.right)

        # Left/right outputs
        xoutLeft = pipeline.createXLinkOut()
        xoutLeft.setStreamName('left')
        camLeft.out.link(xoutLeft.input)

        xoutRight = pipeline.createXLinkOut()
        xoutRight.setStreamName('right')
        camRight.out.link(xoutRight.input)

        # Rectified outputs
        xoutLeft = pipeline.createXLinkOut()
        xoutLeft.setStreamName('left_rect')
        depth.rectifiedLeft.link(xoutLeft.input)

        xoutRight = pipeline.createXLinkOut()
        xoutRight.setStreamName('right_rect')
        depth.rectifiedRight.link(xoutRight.input)

        # Pipeline is defined, now we can connect to the device
        self.device = dai.Device(pipeline)
        # Start pipeline
        self.device.startPipeline()

        # Output queues will be used to get the grayscale frames from the outputs defined above
        self.q_left = self.device.getOutputQueue(name="left", maxSize=4, blocking=False)
        self.q_right = self.device.getOutputQueue(name="right", maxSize=4, blocking=False)
        self.q_left_rect = self.device.getOutputQueue(name="left_rect", maxSize=4, blocking=False)
        self.q_right_rect = self.device.getOutputQueue(name="right_rect", maxSize=4, blocking=False)

    def check_queues(self):
        # Instead of get (blocking), we use tryGet (nonblocking) which will return the available data or None otherwise
        in_left = self.q_left.tryGet()
        in_right = self.q_right.tryGet()
        in_left_rect = self.q_left_rect.tryGet()
        in_right_rect = self.q_right_rect.tryGet()

        # Left image
        if in_left is not None:
            frame = in_left.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            msg.header.frame_id = 'oakd_left'
            # ts = in_left.getTimestamp().total_seconds()
            # msg.header.stamp.sec = int(ts)
            # msg.header.stamp.nanosec = int((ts % 1) * 1e9)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.left_publisher.publish(msg)
        
        # Right image
        if in_right is not None:
            frame = in_right.getCvFrame()
            msg = self.bridge.cv2_to_imgmsg(frame, 'mono8')
            msg.header.frame_id = 'oakd_right'
            # ts = in_right.getTimestamp().total_seconds()
            # msg.header.stamp.sec = int(ts)
            # msg.header.stamp.nanosec = int((ts % 1) * 1e9)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.right_publisher.publish(msg)


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
