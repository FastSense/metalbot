import cv2
import nnio
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import tf2_ros
import message_filters

from perception_msgs.msg import OdoFlow
from .stereo_camera import StereoCamera


class FlowOdomNode(Node):
    """
    This node subscribes to stereo camera images and depth and publishes optical flow measurement for Kalman Filter.
    Currently works only with OAK-D camera.
    Measurement is defined in perception_msgs.msg.OdoFlow
    """
    def __init__(self):
        super().__init__('flow_odom')

        self.bridge = CvBridge()

        # Declare parameters
        self.declare_parameter('network_path', 'http://192.168.194.51:8345/flow/2021.09.28_flow_sv/flow_sv_op12.onnx')

        # Get camera parameters
        self.stereo = None

        # Subscribe to camera topics
        self.create_subscription(
            CameraInfo,
            'rectified_camera_info',
            self.calibration_callback,
            10,
        )
        self.create_subscription(
            Image,
            'left_rect',
            self.left_rect_callback,
            10,
        )
        self.create_subscription(
            Image,
            'depth',
            self.depth_callback,
            10,
        )

        # Publisher
        self.odom_publisher = self.create_publisher(
            OdoFlow,
            'odom_flow',
            10,
        )

        # Flow neural network
        network_path = self.get_parameter('network_path').get_parameter_value().string_value
        # self.network = nnio.ONNXModel(network_path)
        self.network = nnio.OpenVINOModel(
            network_path.replace('_op12.onnx', '_fp16.bin'),
            network_path.replace('_op12.onnx', '_fp16.xml'),
            device='GPU'
        )
        self.preprocess = nnio.Preprocessing(
            resize=(256, 256),
            dtype='float32',
            divide_by_255=True,
            channels_first=True,
            batch_dimension=True,
        )

        # Buffer for images
        self.left_msg = None
        self.depth_msg = None
        self.last_pair = None
        self.img_prev = None
        self.depth_prev = None
        self.depth_std_prev = None
        self.sec_prev = None

    def left_rect_callback(self, msg):
        self.left_msg = msg
        # print('Left', msg.header.stamp)
        self.check_pair()

    def depth_callback(self, msg):
        # print('Depth', msg.header.stamp)
        self.depth_msg = msg
        self.check_pair()

    def check_pair(self):
        if self.left_msg is None or self.depth_msg is None:
            return
        time_left = self.left_msg.header.stamp.sec + self.left_msg.header.stamp.nanosec * 1e-9
        time_depth = self.depth_msg.header.stamp.sec + self.depth_msg.header.stamp.nanosec * 1e-9
        threshold = 0.01
        if abs(time_left - time_depth) < threshold:
            self.last_pair = self.left_msg, self.depth_msg, time_left
            self.publish_odometry()

    def publish_odometry(self):
        if self.last_pair is None or self.stereo is None:
            return

        # Prepare inputs
        img_left = self.bridge.imgmsg_to_cv2(self.last_pair[0])
        img_left = cv2.cvtColor(img_left, cv2.COLOR_GRAY2RGB)
        img_left = self.preprocess(img_left)
        depth = self.bridge.imgmsg_to_cv2(self.last_pair[1])
        sec = self.last_pair[2]

        # Get depth
        depth = cv2.resize(depth, (128, 128)) / 1000
        mask = depth < 0.5
        depth = depth.clip(0.5)
        baseline = np.linalg.norm(self.stereo.T)
        depth_std = depth**2 / (430 * baseline**2)

        # Remember previous values
        if self.sec_prev is None:
            self.img_prev = img_left
            self.depth_prev = depth
            self.depth_std_prev = depth_std
            self.sec_prev = sec
            return

        print('Pair:', sec - self.sec_prev)

        # Get optical flow
        pair_preprocessed = np.concatenate([
            img_left, # [1, 3, H, W]
            self.img_prev, # [1, 3, H, W]
        ], 1) # [1, 6, H, W]
        flow = self.network(pair_preprocessed)
        flow = flow[0].transpose(1, 2, 0)

        # Get depth error
        mask = mask * 100
        max_flow = np.sqrt((flow**2).sum(2)).max()
        side = max(int(max_flow), 1) + 1
        mask[:side] = 10
        mask[-side:] = 10
        mask[:, :side] = 10
        mask[:, -side:] = 10
        depth_sum_err = depth_std + mask

        if self.depth_prev is None:
            self.depth_prev = depth
            self.depth_std_prev = depth_std

        # Shoot random points
        N = 300
        K = 30
        xs = np.random.randint(0, flow.shape[1], size=N)
        ys = np.random.randint(0, flow.shape[0], size=N)
        errors = depth_sum_err[ys, xs]
        top_k = np.argsort(errors)[:K]
        xs = xs[top_k]
        ys = ys[top_k]

        # Compose measurement
        flows = flow[ys, xs] # [N, 2]
        depths = depth[ys, xs] # [N]
        xs_source = (xs + np.round(flows[:, 0]).astype(int)).clip(0, flow.shape[1] - 1)
        ys_source = (ys + np.round(flows[:, 1]).astype(int)).clip(0, flow.shape[0] - 1)
        delta_depth = self.depth_prev[ys_source, xs_source] - depths # [N]

        # Compose covariance
        flow_std = 2
        depth_variance = (
            (depth_std[ys, xs] * flow_std)**2
            +
            (self.depth_std_prev[ys_source, xs_source] * flow_std)**2
            +
            (
                (self.depth_prev[ys_source, xs_source] - self.depth_prev[ys_source - 1, xs_source])**2
                +
                (self.depth_prev[ys_source, xs_source] - self.depth_prev[ys_source, xs_source - 1])**2
            ) * 0.5
        )
        variance = np.zeros([K * 3])
        variance[::3] = flow_std**2 + mask[ys, xs]
        variance[1::3] = variance[::3]
        variance[2::3] = depth_variance
        
        # # Generate not random points
        # K = 2
        # xs = np.array([depth.shape[1] // 4, 3 * depth.shape[1] // 4])
        # ys = np.array([depth.shape[0] // 2, depth.shape[0] // 2])
        # flows = np.array([[1, 0], [-1, 0]])
        # depths = np.ones([K])
        # delta_depth = np.zeros([K])
        # variance = np.ones([K * 3]) * flow_std

        # Make odometry message
        msg = OdoFlow()
        msg.header.stamp = self.last_pair[0].header.stamp
        msg.header.frame_id = self.last_pair[0].header.frame_id
        msg.child_frame_id = self.last_pair[0].header.frame_id
        msg.delta_t = sec - self.sec_prev
        msg.x = [int(x) for x in xs]
        msg.y = [int(y) for y in ys]
        msg.flow_x = [float(flow) for flow in flows[:, 0]]
        msg.flow_y = [float(flow) for flow in flows[:, 1]]
        msg.depth = [float(d) for d in depths]
        msg.delta_depth = [float(dd) for dd in delta_depth]
        msg.covariance_diag = [float(v) for v in variance]
        self.odom_publisher.publish(msg)

        # Remember previous values
        self.img_prev = img_left
        self.depth_prev = depth
        self.depth_std_prev = depth_std
        self.sec_prev = sec

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


def main(args=None):
    print('Hi from odometry.')

    rclpy.init(args=args)

    node = FlowOdomNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
