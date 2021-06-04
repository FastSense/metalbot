#!/usr/bin/env python
# license removed for brevity
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.time import Duration
from rclpy.clock import Clock
import tf2_py as tf2
import tf2_ros
import time
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_msgs.msg import TFMessage

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class path_viz(Node):

    def __init__(self, frame_name, color):
        super().__init__('path_viz')
        self.origin_frame = 'odom'
        self.target_frame = frame_name
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path_pub = self.create_publisher(Marker, '/' + str(frame_name) + '_path', 1)
        self.path_len_ = 0

        self.current_pose = Pose()
        self.last_pose = Pose()
        self.timer = self.create_timer(0.1, self.timer_callback)

		# self.goal_topic_sub = rospy.Subscriber("/mover/path", Path, self.goal_callback)

        self.pMarker = Marker()
        self.pMarker.header.frame_id = self.origin_frame
        self.pMarker.pose.orientation.w = 1.0
        self.pMarker.scale.x = 0.03
        self.pMarker.scale.y = 0.03
        self.pMarker.scale.z = 0.03

        self.pMarker.type = Marker.SPHERE  # LINE_STRIP
        self.pMarker.action = Marker.ADD # ADD

        if color == 'red':
            self.pMarker.color.r = 1.0
            self.pMarker.color.g = 0.0
            self.pMarker.color.b = 0.0
            self.pMarker.color.a = 1.0
        elif color == 'blue':
            self.pMarker.color.r = 0.0
            self.pMarker.color.g = 0.0
            self.pMarker.color.b = 1.0
            self.pMarker.color.a = 1.0
        elif color == 'yellow':
            self.pMarker.color.r = 1.0
            self.pMarker.color.g = 1.0
            self.pMarker.color.b = 0.0
            self.pMarker.color.a = 1.0

    def publish_pose_marker(self):

        self.pMarker.header.stamp = self.get_clock().now().to_msg()
        self.pMarker.id = self.path_len_
        self.pMarker.pose.position = self.current_pose.position


        self.pMarker.points.append(self.last_pose.position)
        self.pMarker.points.append(self.current_pose.position)

        self.pMarker.lifetime = Duration(seconds=0).to_msg()

        self.path_pub.publish(self.pMarker)

        self.path_len_ = self.path_len_ + 1
        self.last_pose = self.current_pose 

    def get_pose(self):
        pose = Pose()
        try:
            tf = self.tf_buffer.lookup_transform(self.origin_frame, self.target_frame , Time(seconds=0))
            translation = tf.transform.translation
            rotation = tf.transform.rotation
            pose.position.x = float(translation.x)
            pose.position.y = float(translation.y)
            pose.position.z = float(translation.z)
            #pose.orientation.x = float(orient[0])
            #pose.orientation.y = float(orient[1])
            #pose.orientation.z = float(orient[2])
            #pose.orientation.w = float(orient[3])
            pose.orientation.x = rotation.x
            pose.orientation.y = rotation.y
            pose.orientation.z = rotation.z
            pose.orientation.w = rotation.w
            self.current_pose = pose

            return True
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return False

    def log_pose(self):
        ret_val = self.get_pose()
        if ret_val:
            self.publish_pose_marker()


    def timer_callback(self):
        self.log_pose()

def main():

	# path_viz("base_link", 'blue', tf_listener)
	#path_viz("model_link", 'red', tf_listener) 
	#path_viz("nn_model_link", 'yellow', tf_listener)
    rclpy.init()
    path_viz_ = path_viz("base_link", "red")
    rclpy.spin(path_viz_)
    rclpy.shutdown()


if __name__ == '__main__':
	main()