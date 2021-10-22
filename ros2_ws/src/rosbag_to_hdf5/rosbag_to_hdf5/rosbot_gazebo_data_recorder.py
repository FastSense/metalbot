import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
import h5py
import numpy as np
import signal
import sys

pcd_topic = 'camera/points'
odom_topic = 'odom'
path_to_save_hdf5 = None

odom_positions = []
odom_rotations = []
odom_stamps = []
pcd_data = []
pcd_stamps = []

class TopicDataRecorder(Node):

    def __init__(self):
        global path_to_save_hdf5
        super().__init__('data_recorder')

        self.bridge = CvBridge()
        self.declare_parameter('path_to_save_hdf5', 'default.hdf5')
        self.declare_parameter('pcd_topic', 'camera/points')
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('verbose', False)
        path_to_save_hdf5 = self.get_parameter('path_to_save_hdf5').value
        pcd_topic = self.get_parameter('pcd_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.verbose = self.get_parameter('verbose').value

        self.odom_subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10
        )
        self.pcd_subscription = self.create_subscription(
            PointCloud2,
            pcd_topic,
            self.pcd_callback,
            10
        )
        _ = self.odom_subscription
        _ = self.pcd_subscription
        print('Path to save hdf5:', path_to_save_hdf5)
        print('Pointcloud topic:', pcd_topic)
        print('Odometry topic:', odom_topic)
        print('Verbose:', self.verbose)


    def odom_callback(self, msg):
        position = msg.pose.pose.position
        rotation = msg.pose.pose.orientation
        odom_positions.append([position.x, position.y, position.z])
        odom_rotations.append([rotation.x, rotation.y, rotation.z, rotation.w])
        odom_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received odometry at time {}'.format(odom_stamps[-1]))


    def pcd_callback(self, msg):
        pcd_data.append(msg.data)
        pcd_stamps.append(msg.header.stamp.sec + 1e-9 * msg.header.stamp.nanosec)
        if self.verbose:
            print('Received pointcloud at time {}'.format(msg.header.stamp))


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
        f.create_dataset('odom_position', data=odom_positions)
        f.create_dataset('odom_rotation', data=odom_rotations)
        f.create_dataset('odom_stamp', data=odom_stamps)
        f.create_dataset('pcd_data', data=pcd_data)
        f.create_dataset('pcd_stamp', data=pcd_stamps)
    print('Dataset saved to file {}'.format(path_to_save_hdf5))


if __name__ == '__main__':
    main()