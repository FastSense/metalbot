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
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        rclpy.spin_once(self)


    def start(self):
        try:
            print("Try Get Tf")
            # t = self.get_clock().now().to_msg()
            t = Namespace(seconds=0, nanoseconds=0)
            trans = self.tf_buffer.lookup_transform('oakd', 'oakd_left', t, rclpy.duration.Duration(seconds=1))
            print("Got tf")

            print('Tf:', trans)
        except:
            print("Error")


rclpy.init()

node = TestNode2()

trans = node.start()

print('trans:', trans)