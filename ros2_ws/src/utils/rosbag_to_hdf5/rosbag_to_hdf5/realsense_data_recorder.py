import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
import h5py
import numpy as np
import signal
import sys

path_to_save_hdf5 = None

depths = []
rgbs = []
pcds = []
rgb_stamps = []
depth_stamps = []
pcd_stamps = []

class TopicDataRecorder(Node):

    def __init__(self):
        global path_to_save_hdf5
        super().__init__('data_recorder')

        # Get parameters for topics and HDF5 file destination
        self.bridge = CvBridge()
        self.declare_parameter('path_to_save_hdf5', 'default.hdf5')
        self.declare_parameter('depth_topic', 'camera/depth/image_rect_raw')
        self.declare_parameter('image_topic', 'camera/color/image_raw')
        self.declare_parameter('pcd_topic', 'camera/depth/color/points')
        self.declare_parameter('verbose', False)
        path_to_save_hdf5 = self.get_parameter('path_to_save_hdf5').value
        depth_topic = self.get_parameter('depth_topic').value
        image_topic = self.get_parameter('image_topic').value
        pcd_topic = self.get_parameter('pcd_topic').value
        self.verbose = self.get_parameter('verbose').value

        # Initialize topic subscribers
        self.depth_subscription = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        self.rgb_subscription = self.create_subscription(
            Image,
            image_topic,
            self.rgb_callback,
            10
        )
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            pcd_topic,
            self.pcd_callback,
            10
        )
        _ = self.rgb_subscription
        _ = self.depth_subscription
        _ = self.pcd_subscription

        print('Path to save hdf5:', path_to_save_hdf5)
        print('Depth topic:', depth_topic)
        print('Image topic:', image_topic)
        print('Pointcloud topic:', pcd_topic)
        print('Verbose:', self.verbose)


    def depth_callback(self, msg):
        depths.append(self.bridge.imgmsg_to_cv2(msg))
        depth_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received depth at time {}'.format(depth_stamps[-1]))


    def pcd_callback(self, msg):
        pcds.append(np.array(msg.data))
        pcd_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received pointcloud at time {}'.format(pcd_stamps[-1]))


    def rgb_callback(self, msg):
        rgbs.append(self.bridge.imgmsg_to_cv2(msg))
        rgb_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received image at time {}'.format(rgb_stamps[-1]))


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

    # Synchronize rgb to depth
    rgbs_sync = []
    pcds_sync = []
    depths_sync = []
    stamps = []
    j = 0
    k = 0
    eps = 1e-3
    for i in range(len(depth_stamps)):
        while j < len(rgb_stamps) and rgb_stamps[j] < depth_stamps[i] - eps:
            j += 1
        while k < len(pcd_stamps) and pcd_stamps[k] < depth_stamps[i] - eps:
            k += 1
        if j == len(rgb_stamps) or abs(rgb_stamps[j] - depth_stamps[i]) > eps:
            continue
        if k == len(pcd_stamps) or abs(pcd_stamps[k] - depth_stamps[i]) > eps:
            continue
        rgbs_sync.append(rgbs[j])
        pcds_sync.append(pcds[k])
        depths_sync.append(depths[i])
        stamps.append(depth_stamps[i])

    # Write dataset
    pcd_lengths = [len(x) for x in pcds_sync]
    max_len = max(pcd_lengths)
    for i in range(len(pcds_sync)):
        pcds_sync[i] = np.concatenate([pcds_sync[i], np.zeros((max_len - len(pcds_sync[i])), dtype=np.uint8)], axis=0)
    with h5py.File(path_to_save_hdf5, 'w') as f:
        f.create_dataset('depth', data=np.array(depths_sync))
        f.create_dataset('rgb', data=np.array(rgbs_sync))
        f.create_dataset('pcd_lengths', data=np.array(pcd_lengths))        
        f.create_dataset('pcd', data=np.array(pcds_sync))
        f.create_dataset('stamp', data=np.array(stamps))
    print('Dataset saved to file {}'.format(path_to_save_hdf5))


if __name__ == '__main__':
    main()