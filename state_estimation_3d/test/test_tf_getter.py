import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import tf2_ros

from argparse import Namespace


class TestNode2(Node):
    def __init__(self):
        super().__init__('test_node_2')

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def get_trans(self):
        # t = self.get_clock().now()
        t = Namespace(seconds=0.0, nanoseconds=0.0)

        trans = self.tf_buffer.lookup_transform(u'oakd', u'oakd_left', t, rclpy.duration.Duration(seconds=1))

        return trans


rclpy.init()

node = TestNode2()

trans = node.get_trans()

print('trans:', trans)