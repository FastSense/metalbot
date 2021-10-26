#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from cv_bridge import CvBridge 	
import h5py
import numpy as np
import tf
import cv2
import yaml

rospy.init_node('hdf5_data_publisher')
path_to_hdf5 = rospy.get_param('~path_to_hdf5', 'default.hdf5')
publish_depth = rospy.get_param('~publish_depth', True)
publish_rgb = rospy.get_param('~publish_rgb', True)
publish_pcd = rospy.get_param('~publish_pcd', True)
publish_pose = rospy.get_param('~publish_pose', True)
publish_camera_info = rospy.get_param('~publish_camera_info', True)
publish_tf = rospy.get_param('~publish_tf', True)
pcd_with_rgb = rospy.get_param('~pcd_with_rgb', True)
depth_topic = rospy.get_param('~depth_topic', 'depth')
pose_topic = rospy.get_param('~pose_topic', 'pose')
pcd_topic = rospy.get_param('~pcd_topic', 'points')
rgb_topic = rospy.get_param('~rgb_topic', 'image')
camera_info_topic = rospy.get_param('~camera_info_topic', 'camera_info')
camera_info_file = rospy.get_param('~camera_info_file', 'camera_info.yaml')
fps = rospy.get_param('~fps', 30)

# Read depth maps and poses
#fx = 430
#baseline = 0.075
print('Loading data from file {}'.format(path_to_hdf5))
with h5py.File(path_to_hdf5, 'r') as f:
	stamps = np.array(f['stamp'])
	if publish_depth:
		depths = np.array(f['depth'])
	if publish_rgb:
		rgbs = np.array(f['rgb'])
		if rgbs.ndim == 4:
			rgbs = rgbs[..., ::-1]
	if publish_pcd:
		pcd_lengths = np.array(f['pcd_lengths'])
		pcds = []
		for i in range(len(pcd_lengths)):
			pcds.append(np.array(f['pcd'][i][:pcd_lengths[i]]))
	if publish_pose:
		positions = np.array(f['position'])
		rotations = np.array(f['rotation'])
		pose_stamps = np.array(f['pose_stamp'])
print('Data loaded')

if not (publish_depth or publish_rgb or publish_pose or publish_pcd):
	print('Nothing to publish. Please set one of publish_depth, publish_rgb, publish_pcd or publish_pose as True')
	exit(0)

if publish_tf and not publish_pose:
	print('Unable to publish tf without known position!')
	publish_tf = False

# Synchronize poses to images
if publish_pose:
	positions_sync = []
	rotations_sync = []
	j = 0
	for i in range(len(stamps)):
		while j < len(pose_stamps) and pose_stamps[j] < stamps[i]:
			j += 1
		if j == len(pose_stamps):
			positions_sync.append(positions[-1])
			rotations_sync.append(rotations[-1])
		if j == 0:
			positions_sync.append(positions[0])
			rotations_sync.append(rotations[0])
		else:
			alpha = (stamps[i] - pose_stamps[j - 1]) / (pose_stamps[j] - pose_stamps[j - 1])
			positions_sync.append(alpha * positions[j] + (1 - alpha) * positions[j - 1])
			rotations_sync.append(alpha * rotations[j] + (1 - alpha) * rotations[j - 1])


# Initialize publishers
rate = rospy.Rate(fps)
if publish_depth:
	depth_publisher = rospy.Publisher(depth_topic, Image, latch=True, queue_size=100)
if publish_rgb:
	rgb_publisher = rospy.Publisher(rgb_topic, Image, latch=True, queue_size=100)
if publish_pcd:
	pcd_publisher = rospy.Publisher(pcd_topic, PointCloud2, latch=True, queue_size=100)
if publish_camera_info:
	camera_info_publisher = rospy.Publisher(camera_info_topic, CameraInfo, latch=True, queue_size=100)
if publish_pose:
	pose_publisher = rospy.Publisher('/pose', PoseWithCovarianceStamped, latch=True, queue_size=100)
bridge = CvBridge()

# Initialize camera info message
with open(camera_info_file, 'r') as f:
	camera_params = yaml.load(f)
if publish_camera_info:
	camera_info_msg = CameraInfo()
	camera_info_msg.header.frame_id = 'camera_link'
	camera_info_msg.height = camera_params['image_height']
	camera_info_msg.width = camera_params['image_width']
	camera_info_msg.distortion_model = camera_params['distortion_model']
	camera_info_msg.D = camera_params['distortion_coefficients']['data']
	camera_info_msg.K = camera_params['camera_matrix']['data']
	camera_info_msg.R = camera_params['rectification_matrix']['data']
	camera_info_msg.P = camera_params['projection_matrix']['data']

# Initialize depth message
if publish_depth:
	if depths.dtype == np.uint8:
		depth_encoding = '8UC1'
	elif depths.dtype == np.uint16:
		depth_encoding = '16UC1'
	else:
		depth_encoding = '32FC1'
	_, H, W = depths.shape
	depth_msg = Image()
	depth_msg.header.frame_id = 'base_scan'
	depth_msg.height = H
	depth_msg.width = W
	depth_msg.encoding = depth_encoding
	depth_msg.is_bigendian = 0
	depth_msg.step = W * 2

# Initialize right message
if publish_rgb:
	if rgbs.ndim == 4:
		rgb_encoding = '8UC3'
	else:
		rgb_encoding = '8UC1'
	rgb_msg = Image()
	rgb_msg.header.frame_id = 'camera_link'
	rgb_msg.height = camera_info_msg.height
	rgb_msg.width = camera_info_msg.width
	rgb_msg.encoding = rgb_encoding
	rgb_msg.is_bigendian = 0
	rgb_msg.step = camera_info_msg.width
	if rgbs.ndim == 4:
		rgb_msg.step *= rgbs.shape[-1]

# Initialize pose message
if publish_pose:
	pose_msg = PoseWithCovarianceStamped()
	pose_msg.header.frame_id = 'odom'
	pose_msg.pose.covariance = list(np.eye(6).ravel() * 0.05)
if publish_tf:
	tf_broadcaster = tf.TransformBroadcaster()

# Initialize point cloud message
if publish_pcd:
	pcd_msg = PointCloud2()
	pcd_msg.header.frame_id = 'points'
	pcd_msg.height = 1
	field_x = PointField(name='x', offset=0, datatype=7, count=1)
	field_y = PointField(name='y', offset=4, datatype=7, count=1)
	field_z = PointField(name='z', offset=8, datatype=7, count=1)
	field_rgb = PointField(name='rgb', offset=16, datatype=7, count=1)
	pcd_msg.is_bigendian = False
	if pcd_with_rgb:
		pcd_msg.fields = [field_x, field_y, field_z, field_rgb]
		pcd_msg.point_step = 20
	else:
		pcd_msg.fields = [field_x, field_y, field_z]
		pcd_msg.point_step = 16

# Publish depths, camera info, and poses with specified rate
for i in range(len(stamps)):
	if rospy.is_shutdown():
		break
	cur_time = rospy.Time.now()
	print('Publish data at time {}'.format(cur_time.to_sec()))

	# Camera info
	if publish_camera_info:
		camera_info_msg.header.stamp = cur_time
		camera_info_publisher.publish(camera_info_msg)

	# Depth
	if publish_depth:
		depth_msg.header.stamp = cur_time
		depth_msg.data = bridge.cv2_to_imgmsg(depths[i], encoding=depth_encoding).data
		depth_publisher.publish(depth_msg)

	# RGB image
	if publish_rgb:
		rgb_msg.header.stamp = cur_time
		rgb_msg.data = bridge.cv2_to_imgmsg(rgbs[i], encoding=rgb_encoding).data
		rgb_publisher.publish(rgb_msg)

	# Pose
	if publish_pose:
		pose_msg.header.stamp = cur_time
		pose_position = Point()
		pose_position.x, pose_position.y, pose_position.z = positions_sync[i]
		pose_msg.pose.pose.position = pose_position
		pose_orientation = Quaternion()
		pose_orientation.x, pose_orientation.y, pose_orientation.z, pose_orientation.w = rotations_sync[i]
		pose_msg.pose.pose.orientation = pose_orientation
		pose_publisher.publish(pose_msg)
	
	# Point cloud
	if publish_pcd:
		pcd_msg.header.stamp = cur_time
		pcd_msg.width = len(pcds[i]) // pcd_msg.point_step
		pcd_msg.row_step = pcd_msg.width * pcd_msg.point_step
		pcd_msg.data = list(pcds[i])
		pcd_publisher.publish(pcd_msg)

	# TF
	if publish_tf:
		tf_broadcaster.sendTransform(positions_sync[i], rotations_sync[i],
			                         cur_time,
			                         'base_link', 'odom')

	rate.sleep()