import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, PointCloud2
from tf2_msgs.msg import TFMessage
#from tf2_ros import TransformListener, Buffer
import h5py
import numpy as np
import signal
import sys

path_to_save_hdf5 = None
record_tf = False

depths = []
rgbs = []
pcds = []
rgb_stamps = []
depth_stamps = []
pcd_stamps = []
positions = []
rotations = []
pose_stamps = []

class TopicDataRecorder(Node):

    def __init__(self):
        global path_to_save_hdf5, record_tf, record_depth
        super().__init__('data_recorder')

        # Get parameters for topics and HDF5 file destination
        self.bridge = CvBridge()
        self.declare_parameter('path_to_save_hdf5', 'default.hdf5')
        self.declare_parameter('depth_topic', 'camera/depth/image_rect_raw')
        self.declare_parameter('image_topic', 'camera/color/image_raw')
        self.declare_parameter('pcd_topic', 'camera/depth/color/points')
        self.declare_parameter('record_tf', False)
        self.declare_parameter('verbose', False)
        self.declare_parameter('odom_frame_id', 'odom_frame')
        self.declare_parameter('camera_frame_id', 'camera_pose_frame')
        path_to_save_hdf5 = self.get_parameter('path_to_save_hdf5').value
        depth_topic = self.get_parameter('depth_topic').value
        image_topic = self.get_parameter('image_topic').value
        pcd_topic = self.get_parameter('pcd_topic').value
        self.verbose = self.get_parameter('verbose').value
        record_tf = self.get_parameter('record_tf').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value

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
        if record_tf:
            self.tf_subscription = self.create_subscription(
                TFMessage,
                'tf',
                self.tf_callback,
                10
            )
            _ = self.tf_subscription
        _ = self.rgb_subscription
        _ = self.pcd_subscription
        _ = self.depth_subscription

        print('Path to save hdf5:', path_to_save_hdf5)
        print('Depth topic:', depth_topic)
        print('Image topic:', image_topic)
        print('Pointcloud topic:', pcd_topic)
        print('Verbose:', self.verbose)
        if record_tf:
            print('Odom frame:', self.odom_frame_id)
            print('Camera frame:', self.camera_frame_id)


    def depth_callback(self, msg):
        depths.append(self.bridge.imgmsg_to_cv2(msg))
        depth_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received depth at time {}'.format(depth_stamps[-1]))


    def pcd_callback(self, msg):
        pcds.append(np.array(msg.data))
        pcd_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        print(len(pcds[-1]))
        if self.verbose:
            print('Received pointcloud at time {}'.format(pcd_stamps[-1]))


    def rgb_callback(self, msg):
        rgbs.append(self.bridge.imgmsg_to_cv2(msg))
        rgb_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received image at time {}'.format(rgb_stamps[-1]))


    def tf_callback(self, msg):
        tf_id = -1
        for i in range(len(msg.transforms)):
            if msg.transforms[i].header.frame_id == self.odom_frame_id and msg.transforms[i].child_frame_id == self.camera_frame_id:
                tf_id = i
                break
        if tf_id == -1:
            print('ERROR: Transform not found!')
            return
        pose_stamps.append(msg.transforms[0].header.stamp.sec + 1e-9 * msg.transforms[0].header.stamp.nanosec)
        position = msg.transforms[0].transform.translation
        rotation = msg.transforms[0].transform.rotation
        positions.append([position.x, position.y, position.z])
        rotations.append([rotation.x, rotation.y, rotation.z, rotation.w])
        if self.verbose:
            print('Received transform at time {}'.format(pose_stamps[-1]))


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
    print('Synchronizing rgbs, depths, and pointclouds')
    rgbs_sync = []
    pcds_sync = []
    depths_sync = []
    stamps = []
    j = 0
    k = 0
    eps = 1e-3
    for i in range(len(rgb_stamps)):
        while j < len(rgb_stamps) and rgb_stamps[j] < depth_stamps[i] - eps:
            j += 1
        while k < len(pcd_stamps) and pcd_stamps[k] < rgb_stamps[i] - eps:
            k += 1
        if j == len(rgb_stamps) or abs(rgb_stamps[j] - depth_stamps[i]) > eps:
            continue
        if k == len(pcd_stamps) or abs(pcd_stamps[k] - rgb_stamps[i]) > eps:
            continue
        rgbs_sync.append(rgbs[i])
        pcds_sync.append(pcds[k])
        depths_sync.append(depths[i])
        stamps.append(rgb_stamps[i])
    print('{} synchronized images to write'.format(len(stamps)))
    if record_tf:
        print('{} poses to write'.format(len(pose_stamps)))

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
        if record_tf:
            f.create_dataset('position', data=np.array(positions))
            f.create_dataset('rotation', data=np.array(rotations))
            f.create_dataset('pose_stamp', data=np.array(pose_stamps))
    print('Dataset saved to file {}'.format(path_to_save_hdf5))


if __name__ == '__main__':
    main()