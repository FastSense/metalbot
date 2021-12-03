#! /usr/bin/env python3
import rospy
import numpy as np
import tf
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from rosgraph_msgs.msg import Clock

clock_publisher = rospy.Publisher('/clock', Clock, latch=True, queue_size=100)
clock_msg = Clock()
pose_publisher = rospy.Publisher('/pose', PoseWithCovarianceStamped, latch=True, queue_size=100)
pose_msg = PoseWithCovarianceStamped()


def tf_callback(msg):
    if msg.transforms[0].header.frame_id != 'odom_frame':
        return
    if msg.transforms[0].child_frame_id != 'camera_pose_frame':
        return
    clock_msg.clock = msg.transforms[0].header.stamp
    clock_publisher.publish(clock_msg)
    pose_msg.header = msg.transforms[0].header
    pose_msg.pose.pose.position = msg.transforms[0].transform.translation
    pose_msg.pose.pose.orientation = msg.transforms[0].transform.rotation
    pose_msg.pose.covariance = list(np.eye(6).ravel())
    pose_publisher.publish(pose_msg)


def odom_callback(msg):
    pose_msg.header = msg.header
    pose_msg.pose = msg.pose
    clock_msg.clock = msg.header.stamp
    pose_publisher.publish(pose_msg)
    clock_publisher.publish(clock_msg)


if __name__ == '__main__':
    rospy.init_node('clock_publisher')
    tf_subscriber = rospy.Subscriber('/tf', TFMessage, tf_callback)
    #odom_subscriber = rospy.Subscriber('/camera/odom/sample', Odometry, odom_callback)
    rospy.spin()