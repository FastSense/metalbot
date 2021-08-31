#!/usr/bin/env python
# license removed for brevity
import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

class Visualizer(Node):
    """
    ROS Node for visualizing odometry in rviz2
    Creates markers for different types of odometry
    ---------------------------------
    Attributes:
    odom_sub : ros::Subscriber
        Subscriber to /odom topic with gt odometry
    odom_noised_sub : ros::Subscriber
        Subscriber to /odom_noised topic
    odom_filtered_sub : ros::Subscriber
        Subscriber to /odom_filtered topic
    path_ground_truth_pub : ros::Publisher
        Odometry gt marker publisher
    path_noised_pub : ros::Publisher
        Odometry noised marker publisher
    path_filtered_pub : ros::Publisher
        Odometry filtered marker publisher
    odom : Odometry
        gt odometry
    odom_noised : Odometry
        Noised odometry
    odom_filtered : Odometry
        Odometry filtered by Kalman Filter
    last_odom : Odometry
        Prev gt odometry
    last_odom_noised : Odometry
        Prev noised odometry
    last_odom_filtered : Odometry
        Prev odometry filtered by Kalman Filter
    origin_frame: str
        Marker frame
    path_len : int
        Number of path points
    pMarker : Marker
        gt odometry marker
    pMarker_filtered: Marker
        filtered odometry marker
    pMarker_noised : Marker
        noised odometry marker
    timer : ros::Timer
        Timer for publishing markers
    """
    def __init__(self):
        super().__init__('visualizer')

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
        self.path_ground_truth_pub = self.create_publisher(
            Marker, 
            '/' + 'path_ground_truth', 
            1)
        self.path_noised_pub = self.create_publisher(
            Marker, 
            '/' + 'path_noised', 
            1)
        self.path_filtered_pub = self.create_publisher(
            Marker, 
            '/' + 'path_filtered', 
            1)
        self.odom = Odometry()
        self.odom_noised = Odometry()
        self.odom_filtered = Odometry()
        self.last_odom = Odometry()
        self.last_odom_filtered = Odometry()
        self.last_odom_noised = Odometry()
        self.origin_frame = 'odom'
        self.path_len_ = 0
        
        # Set marker headers, colors, scale
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

        self.timer = self.create_timer(0.1, self.timer_callback)

    def odom_callback(self, msg):
        """
        GT odometry callback
        @ parameters
        msg : Odometry
            gt odometry
        """
        self.odom = msg

    def odom_filtered_callback(self, msg):
        """
        Kalman filtered odometry callback
        @ parameters
        msg : Odometry
            Filtered odometry
        """
        self.odom_filtered = msg
    
    def odom_noised_callback(self, msg):
        """
        Noised odometry callback
        @ parameters
        msg : Odometry
            Noised odometry
        """
        self.odom_noised = msg
    
    def publish_pose_marker(self):
        """
        Publish all markers
        @ parameters
        """
        #Marker for gt odometry
        self.pMarker.header.stamp = self.get_clock().now().to_msg()
        self.pMarker.id = self.path_len_
        self.pMarker.pose.position = self.odom.pose.pose.position
        self.pMarker.points.append(self.last_odom.pose.pose.position)
        self.pMarker.points.append(self.odom.pose.pose.position)
        self.pMarker.lifetime = Duration(seconds=0).to_msg()
        #Marker for filtered odometry
        self.pMarker_filtered.header.stamp = self.get_clock().now().to_msg()
        self.pMarker_filtered.id = self.path_len_
        self.pMarker_filtered.pose.position = self.odom_filtered.pose.pose.position
        self.pMarker_filtered.points.append(self.last_odom_filtered.pose.pose.position)
        self.pMarker_filtered.points.append(self.odom_filtered.pose.pose.position)
        self.pMarker_filtered.lifetime = Duration(seconds=0).to_msg()
        #Marker for nosied odometry
        self.pMarker_noised.header.stamp = self.get_clock().now().to_msg()
        self.pMarker_noised.id = self.path_len_
        self.pMarker_noised.pose.position = self.odom_noised.pose.pose.position
        self.pMarker_noised.points.append(self.last_odom_noised.pose.pose.position)
        self.pMarker_noised.points.append(self.odom_noised.pose.pose.position)
        self.pMarker_noised.lifetime = Duration(seconds=0).to_msg()

        self.path_ground_truth_pub.publish(self.pMarker)
        self.path_filtered_pub.publish(self.pMarker_filtered)
        self.path_noised_pub.publish(self.pMarker_noised)

        self.path_len_ = self.path_len_ + 1
        self.last_odom = self.odom 
        self.last_odom_filtered = self.odom_filtered 
        self.last_odom_noised = self.odom_noised 
        
    def timer_callback(self):
        """Publish markers"""
        self.publish_pose_marker()

def main():
    rclpy.init()
    path_viz_ = Visualizer()
    rclpy.spin(path_viz_)
    rclpy.shutdown()

if __name__ == '__main__':
	main()