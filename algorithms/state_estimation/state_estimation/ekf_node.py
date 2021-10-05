import cv2
import numpy as np
from scipy.spatial.transform import Rotation
from argparse import Namespace

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros

# from .ekf.diff4 import Filter2D
# from .ekf.diff10 import Filter as Filter25D
# import state_estimation.ekf.space12
# from .ekf.space12 import SpaceKF12 as Filter3D
import state_estimation.ekf.diff4.filter as filter2d
import state_estimation.ekf.diff10.filter as filter25d
import state_estimation.ekf.space12.filter as filter3d
from perception_msgs.msg import OdoFlow
from optical_flow.stereo_camera import StereoCamera


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_3d')

        self.bridge = CvBridge()

        self.tf2_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Declare parameters
        self.declare_parameter('mode', '2d')
        self.declare_parameter('use_nn_model', 1)
        self.declare_parameter('sensors', 'imu, odometry')
        self.declare_parameter('period', 0.3)
        self.declare_parameter('vel_std', 1.0)
        self.declare_parameter('rot_vel_std', 1.0)
        self.declare_parameter('acc_std', 0.0)

        

        # Kalman filter parameters
        mode = self.get_parameter('mode').get_parameter_value().string_value
        use_nn_model = self.get_parameter('use_nn_model').get_parameter_value().bool_value
        self.sensors = self.get_parameter('sensors').get_parameter_value().string_value.replace(" ", "").split(",")
        self.period = self.get_parameter('period').get_parameter_value().double_value
        vel_std = self.get_parameter('vel_std').get_parameter_value().double_value
        rot_vel_std = self.get_parameter('rot_vel_std').get_parameter_value().double_value
        acc_std = self.get_parameter('rot_vel_std').get_parameter_value().double_value


        # # Get camera parameters
        # self.stereo = None

        if mode =='2d':
            self.filter = filter2d.Filter2D(self.period, 2, 0, vel_std, rot_vel_std, use_nn_model)
        elif mode == '2.5d':
            self.filter = filter25d.Filter(self.period, vel_std, rot_vel_std)
        elif mode == '3d':
            self.filter = filter3d.SpaceKF12(self.period, vel_std, rot_vel_std)

        if 'odometry' in self.sensors:
            self.odom_sub = self.create_subscription(
                Odometry,
                'odom_noised',
                self.odometry_callback,
                10,
            )
        if 'imu' in self.sensors:
            self.imu_sub = self.create_subscription(
                Imu,
                'imu',
                self.imu_callback,
                10,
            )
        if 'optical_flow' in self.sensors:
            self.create_subscription(
                OdoFlow,
                'odom_flow',
                self.odometry_flow_callback,
                10,
            )
            self.create_subscription(
                CameraInfo,
                'rectified_camera_info',
                self.calibration_callback,
                10,
            )
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.control_callback,
            15,
        )
        self.pose_publisher = self.create_publisher(
            Odometry,
            'odom_filtered',
            10,
        )

        # Create timer
        self.create_timer(self.period, self.step)

        # Buffers for measurements
        self.imu_buffer = None
        self.imu_count = 0
        self.odom_buffer = None
        self.odom_flow_buffer = None
        self.control = None

        # TF listener
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=1))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def odometry_callback(self, msg):
        self.odom_buffer = msg

    def imu_callback(self, msg):
        if not 'oakd' in msg.header.frame_id:
            # print('Wrong imu frame:', msg.header.frame_id)
            return

        if self.imu_buffer is None:
            self.imu_buffer = msg
            self.imu_count = 1
        else:
            self.imu_buffer.angular_velocity.x += msg.angular_velocity.x
            self.imu_buffer.angular_velocity.y += msg.angular_velocity.y
            self.imu_buffer.angular_velocity.z += msg.angular_velocity.z
            self.imu_buffer.linear_acceleration.x += msg.linear_acceleration.x
            self.imu_buffer.linear_acceleration.y += msg.linear_acceleration.y
            self.imu_buffer.linear_acceleration.z += msg.linear_acceleration.z
            self.imu_count += 1

    def odometry_flow_callback(self, msg):
        self.odom_flow_buffer = msg
    
    def calibration_callback(self, msg):
        if self.stereo is None:
            M1 = np.array(msg.k).reshape([3, 3])
            M2 = M1
            T = [msg.p[3] / msg.k[0], msg.p[7] / msg.k[0], msg.p[11] / msg.k[0]]
            R = np.array(msg.r).reshape([3, 3])
            print('T', T)
            self.stereo = StereoCamera(
                M1=M1, M2=M2, R=R, T=T, image_h=msg.height, image_w=msg.width
            )
            self.stereo.change_dimensions_(128, 128)
    
    def control_callback(self, msg):
        """
        Callback from /cmd_vel topic with control commands
        @ parameters
        msg: Twist
        """
        self.control = np.array([msg.linear.x, msg.angular.z])


    def step(self):
        '''
        EKF predict and update step
        '''
        # Predict
        self.filter.predict(dt=self.period, control=self.control)
        # Update
        if self.imu_buffer is not None:
            self.update_imu(self.imu_buffer)
            self.imu_buffer = None
        # if self.odom_buffer is not None:
        #     self.update_odom(self.odom_buffer)
            # self.odom_buffer = None
        # if self.odom_flow_buffer is not None:
        #     self.update_odom_flow(self.odom_flow_buffer)
        #     self.odom_flow_buffer = None
        # # Publish
        self.publish_pose()

    def update_imu(self, msg):
        '''
        Update filter state using IMU message
        '''
        rot_vel, rot_vel_R, acc, acc_R, extrinsic = self.prepare_imu_data(msg)
        # Update
        self.filter.update_acc(acc, acc_R, extrinsic=extrinsic)
        self.filter.update_rot_vel(rot_vel, rot_vel_R, extrinsic=extrinsic)

    def prepare_imu_data(self, msg):
        # Make KF-compatible measurements
        rot_vel = np.empty(3)
        rot_vel[0] = msg.angular_velocity.x / self.imu_count
        rot_vel[1] = msg.angular_velocity.y / self.imu_count
        rot_vel[2] = msg.angular_velocity.z / self.imu_count
        rot_vel_R = np.array(msg.angular_velocity_covariance).reshape([3, 3])
        acc = np.empty(3)
        acc[0] = msg.linear_acceleration.x / self.imu_count
        acc[1] = msg.linear_acceleration.y / self.imu_count
        acc[2] = msg.linear_acceleration.z / self.imu_count
        acc_R = np.array(msg.linear_acceleration_covariance).reshape([3, 3])

        # Get extrinsics from tf
        extrinsic = self.get_extrinsic(msg.header.frame_id, 'base_link')
        return rot_vel, rot_vel_R, acc, acc_R, extrinsic

    # def update_odom_flow(self, msg):
    #     '''
    #     Update filter state using flow odometry message
    #     '''
    #     if self.stereo is None:
    #         print('waiting for camera parameters...')
    #         return

    #     z = np.vstack([msg.flow_x, msg.flow_y, msg.delta_depth]).transpose() # [N, 3]
    #     pixels = np.vstack([msg.x, msg.y]).transpose() # [N, 2]
    #     R = np.diag(msg.covariance_diag)

    #     # Get extrinsics from tf
    #     extrinsic = self.get_extrinsic(msg.header.frame_id, 'base_link')

    #     self.tracker.update_flow(
    #         z,
    #         self.period,
    #         msg.depth,
    #         pixels,
    #         R,
    #         self.stereo.M1,
    #         self.stereo.M1_inv,
    #         extrinsic=extrinsic,
    #     )

    def update_odom(self, msg):
        odom, R, extrinsic = self.prepare_odom_data(msg)
        self.filter.update_odometry(odom, R, extrinsic=extrinsic)

    def prepare_odom_data(self, msg):
        odom = np.array([
            self.odom.twist.twist.linear.x,
            self.odom.twist.twist.angular.z,
        ])
        R = np.array([
            [self.odom.twist.covariance[0], self.odom.twist.covariance[5]],
            [self.odom.twist.covariance[30], self.odom.twist.covariance[35]],
        ])

        # Get extrinsics from tf
        extrinsic = self.get_extrinsic(msg.child_frame_id, 'base_link')
        return odom, R, extrinsic
    
    # def update_odom(self, msg):
    #     '''
    #     Update filter state using odometry message
    #     '''
    #     # Make KF-compatible measurements
    #     odom = np.array([
    #         msg.twist.twist.angular.x,
    #         msg.twist.twist.angular.y,
    #         msg.twist.twist.angular.z,
    #         msg.twist.twist.linear.x,
    #         msg.twist.twist.linear.y,
    #         msg.twist.twist.linear.z,
    #     ])
    #     R = np.array(msg.twist.covariance).reshape([6, 6])

    #     # Get extrinsics from tf
    #     extrinsic = self.get_extrinsic('oakd_left', 'base_link')

    #     # Update
    #     self.tracker.update_odometry(odom, R, delta_t=1, extrinsic=extrinsic)

    # def get_extrinsic(self, frame1, frame2):
    #     '''
    #     Parameters:
    #     frame1 (str): tf frame
    #     frame2 (str): tf frame

    #     Returns:
    #     np.array of shape [3, 4]: rotation-translation matrix between two tf frames.
    #     '''
    #     while True:
    #         try:
    #             # t = self.get_clock().now()
    #             # rclpy.spin_once(self)
    #             t = Namespace(seconds=0, nanoseconds=0)
    #             trans = self.tf_buffer.lookup_transform(frame1, frame2, t, rclpy.duration.Duration(seconds=10))
    #             # print(f"Got transform! {frame1} -> {frame2}")
    #             break
    #         except tf2_ros.LookupException:
    #             # rclpy.spin_once(self)
    #             print(f"Retrying to get transform {frame1} -> {frame2}", self.get_clock().now())

    #     tr = np.array([
    #         [trans.transform.translation.x],
    #         [trans.transform.translation.y],
    #         [trans.transform.translation.z],
    #     ])
    #     rot_q = np.array([
    #         trans.transform.rotation.x,
    #         trans.transform.rotation.y,
    #         trans.transform.rotation.z,
    #         trans.transform.rotation.w,
    #     ])
    #     rot = Rotation.from_quat(rot_q).as_matrix()
    #     extrinsic = np.concatenate([rot, tr], 1)
    #     return extrinsic

    # def publish_pose(self):
    #     # Make odometry message
    #     msg = Odometry()
    #     msg.header.stamp = self.get_clock().now().to_msg()
    #     msg.header.frame_id = 'map'
    #     msg.child_frame_id = 'base_link'
    #     # Position
    #     msg.pose.pose.position.x = self.tracker.pos[0]
    #     msg.pose.pose.position.y = self.tracker.pos[1]
    #     msg.pose.pose.position.z = self.tracker.pos[2]
    #     # Angle
    #     msg.pose.pose.orientation.w = self.tracker.q[0]
    #     msg.pose.pose.orientation.x = self.tracker.q[1]
    #     msg.pose.pose.orientation.y = self.tracker.q[2]
    #     msg.pose.pose.orientation.z = self.tracker.q[3]
    #     # Pose & angle covariance
    #     msg.pose.covariance = self.tracker.get_pose_covariance()
    #     # Velocity
    #     msg.twist.twist.linear.x = self.tracker.vel[0]
    #     msg.twist.twist.linear.y = self.tracker.vel[1]
    #     msg.twist.twist.linear.z = self.tracker.vel[2]
    #     # Angular velocity
    #     msg.twist.twist.angular.x = self.tracker.rot_vel[0]
    #     msg.twist.twist.angular.y = self.tracker.rot_vel[1]
    #     msg.twist.twist.angular.z = self.tracker.rot_vel[2]
    #     # Vel & angvel covariance
    #     msg.twist.covariance = self.tracker.get_twist_covariance()
    #     # Publish
    #     self.pose_publisher.publish(msg)

    #     # Broadcast tf2
    #     t = tf2_ros.TransformStamped()
    #     t.header = msg.header
    #     t.child_frame_id = 'base_link'
    #     t.transform.translation.x = self.tracker.pos[0]
    #     t.transform.translation.y = self.tracker.pos[1]
    #     t.transform.translation.z = self.tracker.pos[2]
    #     t.transform.rotation.w = self.tracker.q[0]
    #     t.transform.rotation.x = self.tracker.q[1]
    #     t.transform.rotation.y = self.tracker.q[2]
    #     t.transform.rotation.z = self.tracker.q[3]
    #     self.tf2_broadcaster.sendTransform(t)

def main(args=None):
    print('Hi from ekf_3d.')

    rclpy.init(args=args)

    node = EKFNode()
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
