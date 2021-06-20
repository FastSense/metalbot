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
from nav_msgs.msg import Odometry

from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')

        self.path_len_ = 0
        self.origin_frame = 'odom'
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback, 
            15)

        self.odom_noised_sub = self.create_subscription(
            Odometry,
            'odom_noised',
            self.odom_noised_callback, 
            15)
        
        self.odom_filtered_sub = self.create_subscription(
            Odometry,
            'odom_filtered',
            self.odom_filtered_callback, 
            15)

        self.path_ground_truth_pub = self.create_publisher(Marker, '/' + 'path_ground_truth', 1)
        self.path_noised_pub = self.create_publisher(Marker, '/' + 'path_noised', 1)
        self.path_filtered_pub = self.create_publisher(Marker, '/' + 'path_filtered', 1)
        self.odom = Odometry()
        self.odom_noised = Odometry()
        self.odom_filtered = Odometry()
        self.last_odom = Odometry()
        self.last_odom_filtered = Odometry()
        self.last_odom_noised = Odometry()

        self.pMarker = Marker()
        self.pMarker.header.frame_id = self.origin_frame
        self.pMarker.pose.orientation.w = 1.0
        self.pMarker.scale.x = 0.03
        self.pMarker.scale.y = 0.03
        self.pMarker.scale.z = 0.03

        self.pMarker.type = Marker.SPHERE  # LINE_STRIP
        self.pMarker.action = Marker.ADD # ADD

        self.pMarker.color.r = 1.0
        self.pMarker.color.g = 0.0
        self.pMarker.color.b = 0.0
        self.pMarker.color.a = 1.0

        self.pMarker_filtered = Marker()
        self.pMarker_filtered.header.frame_id = self.origin_frame
        self.pMarker_filtered.pose.orientation.w = 1.0
        self.pMarker_filtered.scale.x = 0.03
        self.pMarker_filtered.scale.y = 0.03
        self.pMarker_filtered.scale.z = 0.03

        self.pMarker_filtered.type = Marker.SPHERE  # LINE_STRIP
        self.pMarker_filtered.action = Marker.ADD # ADD

        self.pMarker_filtered.color.r = 0.0
        self.pMarker_filtered.color.g = 1.0
        self.pMarker_filtered.color.b = 0.0
        self.pMarker_filtered.color.a = 1.0

        self.pMarker_noised = Marker()
        self.pMarker_noised.header.frame_id = self.origin_frame
        self.pMarker_noised.pose.orientation.w = 1.0
        self.pMarker_noised.scale.x = 0.03
        self.pMarker_noised.scale.y = 0.03
        self.pMarker_noised.scale.z = 0.03

        self.pMarker_noised.type = Marker.SPHERE  # LINE_STRIP
        self.pMarker_noised.action = Marker.ADD # ADD

        self.pMarker_noised.color.r = 0.0
        self.pMarker_noised.color.g = 0.0
        self.pMarker_noised.color.b = 1.0
        self.pMarker_noised.color.a = 1.0

    def odom_callback(self, msg):
        self.odom = msg

    def odom_filtered_callback(self, msg):
        self.odom_filtered = msg
    
    def odom_noised_callback(self, msg):
        self.odom_noised = msg

    
    def publish_pose_marker(self):

        self.pMarker.header.stamp = self.get_clock().now().to_msg()
        self.pMarker.id = self.path_len_
        self.pMarker.pose.position = self.odom.pose.pose.position

        self.pMarker_filtered.header.stamp = self.get_clock().now().to_msg()
        self.pMarker_filtered.id = self.path_len_
        self.pMarker_filtered.pose.position = self.odom_filtered.pose.pose.position
        
        self.pMarker_noised.header.stamp = self.get_clock().now().to_msg()
        self.pMarker_noised.id = self.path_len_
        self.pMarker_noised.pose.position = self.odom_noised.pose.pose.position

        self.pMarker.points.append(self.last_odom.pose.pose.position)
        self.pMarker.points.append(self.odom.pose.pose.position)
        self.pMarker_filtered.points.append(self.last_odom_filtered.pose.pose.position)
        self.pMarker_filtered.points.append(self.odom_filtered.pose.pose.position)
        self.pMarker_noised.points.append(self.last_odom_noised.pose.pose.position)
        self.pMarker_noised.points.append(self.odom_noised.pose.pose.position)

        self.pMarker.lifetime = Duration(seconds=0).to_msg()
        self.pMarker_filtered.lifetime = Duration(seconds=0).to_msg()
        self.pMarker_noised.lifetime = Duration(seconds=0).to_msg()

        self.path_ground_truth_pub.publish(self.pMarker)
        self.path_filtered_pub.publish(self.pMarker_filtered)
        self.path_noised_pub.publish(self.pMarker_noised)

        self.path_len_ = self.path_len_ + 1
        self.last_odom = self.odom 
        self.last_odom_filtered = self.odom_filtered 
        self.last_odom_noised = self.odom_noised 
        
    def timer_callback(self):
        self.publish_pose_marker()

def main():
    rclpy.init()
    path_viz_ = Visualizer()
    rclpy.spin(path_viz_)
    rclpy.shutdown()


if __name__ == '__main__':
	main()