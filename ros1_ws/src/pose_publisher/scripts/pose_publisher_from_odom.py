#! /usr/bin/env python3
import rospy
import numpy as np
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock

publish_clock = rospy.get_param('~clock', default=False)
odom_topic = rospy.get_param('odom_topic', default='odom')
clock_publisher = rospy.Publisher('/clock', Clock, latch=True, queue_size=100)
clock_msg = Clock()
pose_publisher = rospy.Publisher('/pose_with_covariance', PoseWithCovarianceStamped, latch=True, queue_size=100)
pose_msg = PoseWithCovarianceStamped()


def odom_callback(msg):
    pose_msg.header = msg.header
    pose_msg.pose = msg.pose
    cov = list(msg.pose.covariance)
    for i in range(len(cov)):
        cov[i] = min(cov[i], 0.1)
    pose_msg.pose.covariance = tuple(cov)
    pose_publisher.publish(pose_msg)
    if publish_clock:
        clock_msg.clock = msg.header.stamp
        clock_publisher.publish(clock_msg)


if __name__ == '__main__':
    rospy.init_node('pose_publisher')
    odom_subscriber = rospy.Subscriber(odom_topic, Odometry, odom_callback)
    rospy.spin()