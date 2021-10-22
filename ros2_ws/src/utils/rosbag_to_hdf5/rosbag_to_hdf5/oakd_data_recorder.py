import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
import h5py
import numpy as np
import signal
import sys

path_to_save_hdf5 = None

depths = []
lefts = []
rights = []
left_stamps = []
right_stamps = []
positions = []
rotations = []
depth_stamps = []
pose_stamps = []

class TopicDataRecorder(Node):

    def __init__(self):
        global path_to_save_hdf5
        super().__init__('data_recorder')

        # Get parameters for topics and HDF5 file destination
        self.bridge = CvBridge()
        self.declare_parameter('path_to_save_hdf5', 'default.hdf5')
        self.declare_parameter('depth_topic', 'depth')
        self.declare_parameter('left_image_topic', 'left_rect')
        self.declare_parameter('right_image_topic', 'right_rect')
        self.declare_parameter('pose_topic', 'pose')
        self.declare_parameter('verbose', False)
        path_to_save_hdf5 = self.get_parameter('path_to_save_hdf5').value
        depth_topic = self.get_parameter('depth_topic').value
        left_image_topic = self.get_parameter('depth_topic').value
        right_image_topic = self.get_parameter('right_image_topic').value
        pose_topic = self.get_parameter('pose_topic').value
        self.verbose = self.get_parameter('verbose').value
        print('Path to save hdf5:', path_to_save_hdf5)
        print('Depth topic:', depth_topic)
        print('Pose topic:', pose_topic)
        print('Left image topic:', left_image_topic)
        print('Right image topic:', right_image_topic)
        print('Verbose:', self.verbose)

        # Initialize subscriptions for depth, left and right images, and pose
        self.depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        self.left_subscription = self.create_subscription(
            Image,
            left_image_topic,
            self.left_callback,
            10
        )
        self.right_subscription = self.create_subscription(
            Image,
            right_image_topic,
            self.right_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self.pose_callback,
            10
        )

        # Touch subscriptions to avoid unused variable warnings
        _ = self.pose_subscription
        _ = self.depth_subscription
        _ = self.left_subscription
        _ = self.right_subscription


    def depth_callback(self, msg):
        depths.append(self.bridge.imgmsg_to_cv2(msg))
        depth_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received depth at time {}'.format(depth_stamps[-1]))


    def pose_callback(self, msg):
        position = msg.pose.position
        rotation = msg.pose.orientation
        positions.append([position.x, position.y, position.z])
        rotations.append([rotation.x, rotation.y, rotation.z, rotation.w])
        pose_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received pose at time {}'.format(pose_stamps[-1]))


    def left_callback(self, msg):
        lefts.append(self.bridge.imgmsg_to_cv2(msg))
        left_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received left image at time {}'.format(left_stamps[-1]))


    def right_callback(self, msg):
        rights.append(self.bridge.imgmsg_to_cv2(msg))
        right_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received right image at time {}'.format(right_stamps[-1]))


def signal_handler(signal, frame):
    rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    data_recorder = TopicDataRecorder()

    signal.signal(signal.SIGINT, signal_handler)

    while rclpy.ok():
        rclpy.spin_once(data_recorder)

    print('On shutdown')
    data_recorder.destroy_node()
    with h5py.File(path_to_save_hdf5, 'w') as f:
        f.create_dataset('depth', data=np.array(depths))
        f.create_dataset('left', data=np.array(lefts))
        f.create_dataset('right', data=np.array(rights))
        f.create_dataset('depth_stamp', data=np.array(depth_stamps))
        f.create_dataset('left_stamp', data=np.array(left_stamps))
        f.create_dataset('right_stamp', data=np.array(right_stamps))
        f.create_dataset('position', data=np.array(positions))
        f.create_dataset('rotation', data=np.array(rotations))
        f.create_dataset('pose_stamp', data=np.array(pose_stamps))
    print('Dataset saved to file {}'.format(path_to_save_hdf5))


if __name__ == '__main__':
    main()