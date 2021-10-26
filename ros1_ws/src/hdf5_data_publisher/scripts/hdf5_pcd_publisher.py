#! /usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
import h5py
import numpy as np
import tf
import cv2

rospy.init_node('hdf5_pcd_publisher')

# Read data from HDF5
hdf5_data_file = '/home/kirill/DTL_project/rosbot_ws/src/oakd_rosbags/rosbot_gazebo_msgs3.hdf5'
with h5py.File(hdf5_data_file, 'r') as f:
	positions = np.array(f['odom_position'])
	rotations = np.array(f['odom_rotation'])
	pose_stamps = np.array(f['odom_stamp'])
	pcd_data = np.array(f['pcd_data'])
	pcd_stamps = np.array(f['pcd_stamp'])
print(pcd_data.shape)

# Synchronize poses to pointclouds
j = 0
positions_sync = np.zeros((pcd_data.shape[0], 3))
rotations_sync = np.zeros((pcd_data.shape[0], 4))
for i in range(len(pcd_stamps)):
	while j < len(pose_stamps) and pose_stamps[j] < pcd_stamps[i]:
		j += 1
	if j == 0:
		positions_sync[i] = positions[j]
		rotations_sync[i] = rotations[j]
	elif j == len(pose_stamps):
		positions_sync[i] = positions[j - 1]
		rotations_sync[i] = rotations[j - 1]
	else:
		alpha = (pcd_stamps[i] - pose_stamps[j - 1]) / (pose_stamps[j] - pose_stamps[j - 1])
		positions_sync[i] = alpha * positions[j] + (1 - alpha) * positions[j - 1]
		rotations_sync[i] = alpha * rotations[j] + (1 - alpha) * rotations[j - 1]

# Initialize publishers
rate = rospy.Rate(10)
pose_publisher = rospy.Publisher('/pose', PoseWithCovarianceStamped, latch=True, queue_size=100)
pcd_publisher = rospy.Publisher('/points', PointCloud2, latch=True, queue_size=100)

# Initialize pose message
pose_msg = PoseWithCovarianceStamped()
pose_msg.header.frame_id = 'odom'
pose_msg.pose.covariance = list(np.eye(6).ravel() * 0.05)
tf_broadcaster = tf.TransformBroadcaster()

# Initialize pointcloud message
pcd_msg = PointCloud2()
pcd_msg.header.frame_id = 'points'
pcd_msg.height = 1
pcd_msg.width = pcd_data.shape[1] // 32
pcd_msg.fields = [
	PointField(name='x', offset=0, datatype=7, count=1),
	PointField(name='y', offset=4, datatype=7, count=1),
	PointField(name='z', offset=8, datatype=7, count=1),
	PointField(name='rgb', offset=16, datatype=7, count=1)
]
pcd_msg.is_bigendian = False
pcd_msg.point_step = 32
pcd_msg.row_step = pcd_data.shape[1]

# Publish pose and pcd with specified rate
for i in range(len(pcd_data)):
	if rospy.is_shutdown():
		break

	cur_time = rospy.Time.now()
	print('Publish data at time {}'.format(cur_time.to_sec()))

	# Pose
	pose_msg.header.stamp = cur_time
	pose_position = Point()
	pose_position.x, pose_position.y, pose_position.z = positions_sync[i]
	pose_msg.pose.pose.position = pose_position
	pose_orientation = Quaternion()
	pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w = rotations_sync[i]
	pose_msg.pose.pose.orientation = pose_orientation
	pose_publisher.publish(pose_msg)

	# TF
	tf_broadcaster.sendTransform(positions_sync[i], rotations_sync[i],
		                         cur_time,
		                         'base_link', 'odom')

	# Pointcloud
	pcd_msg.header.stamp = cur_time
	pcd_msg.data = list(pcd_data[i])
	pcd_publisher.publish(pcd_msg)

	rate.sleep()