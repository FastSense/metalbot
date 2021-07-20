import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import tf2_ros


rclpy.init()

node = Node('test_node')


tf = tf2_ros.TransformStamped()
tf.header.frame_id = 'test_head'
tf.child_frame_id = 'test_body'
tf2_ros.StaticTransformBroadcaster(node).sendTransform(tf)
