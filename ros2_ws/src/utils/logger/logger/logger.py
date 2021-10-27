import os
import pandas as pd
from matplotlib import pyplot as plt
import rclpy
import numpy as np
from argparse import Namespace

from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import tf2_ros

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from logger.utils import convert_ros2_time_to_float
from logger.create_graphs import build_general_graph_for_rosbot
from scipy.spatial.transform import Rotation


class Logger(Node):
    """
    Class for logging the state of the rosbot
    Node for logging the state of the robot,
    kinematic model (optional) and neural network
    model (optional), control and time stamps
    :Attributes:
        :first_tick: (bool) srue if it is first callbcak
        :init_time: (float) node start time (time of the first callback)
        :curr_control: (list) current control [u_v, u_w]
        :output_path: (str) Absolute path to the directory
         where the logged data will be saved
        :control_topic: (str) nam of the control topic (/cmd_vel)
        :parent_frame: (str) name of the origin tf frame
        :kinetic_model_frame: (str) name of the kinematic model tf frame
        :nn_model_frame: (str) name of the NN model tf frame
        :robot_state: (pandas.DataFrame) container for rosbot state
        :kinetic_model_state: (pandas.DataFrame) container for
         kinematic model state
        :nn_model_state: (pandas.DataFrame) container for NN model state
        :robot_control: (pandas.DataFrame) container for rosbot control
        :time: (list) container for time stamps
        :odom_sub: subscriber to /odom topic
        :control_sub: subscriber to control topic
    """

    def __init__(self):
        """
        """
        super().__init__('logger')

        self.init_parameters()
        self.get_node_parametes()
        self.init_subs()
        self.init_containers()

        self.first_tick = True
        self.init_time = None
        self.curr_control = list()
        self.curr_velocity = Twist()
        self.tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=0.01))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.srv = self.create_service(
            Empty, 'shutdown_logger', self.shutdown_logger_callback)
        rclpy.get_default_context().on_shutdown(self.on_shutdown)

    def init_parameters(self):
        """
        Declares node parameters
        """
        self.declare_parameter('output_path', "")
        self.declare_parameter('control_topic', '/cmd_vel')
        self.declare_parameter('parent_frame', 'odom')
        self.declare_parameter('kinetic_model_frame', 'model_link')
        self.declare_parameter('nn_model_frame', 'nn_model_link')
        self.declare_parameter('tf_topic', '/tf')
        self.declare_parameter('use_odom_topic', True)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('velocity_topic', '/velocity')

    def get_node_parametes(self):
        """
        Gets node parameters
        """
        self.output_path = self.get_parameter(
            'output_path').get_parameter_value().string_value
        self.control_topic = self.get_parameter(
            'control_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter(
            'parent_frame').get_parameter_value().string_value
        self.kinetic_model_frame = self.get_parameter(
            'kinetic_model_frame').get_parameter_value().string_value
        self.nn_model_frame = self.get_parameter(
            'nn_model_frame').get_parameter_value().string_value
        self.use_odom_topic = self.get_parameter(
            'use_odom_topic').get_parameter_value().bool_value
        self.odom_topic = self.get_parameter(
            'odom_topic').get_parameter_value().string_value
        self.velocity_topic = self.get_parameter(
            'velocity_topic').get_parameter_value().string_value
        self.tf_topic = self.get_parameter(
            'tf_topic').get_parameter_value().string_value

    def init_containers(self):
        """
        Declares containers for logged data
        """
        self.robot_state = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z',
            ]
        )
        self.kinetic_model_state = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z',
            ]
        )
        self.nn_model_state = pd.DataFrame(
            columns=[
                'x', 'y', 'z', 'roll', 'pitch', 'yaw',
                'v_x', 'v_y', 'v_z', 'w_x', 'w_y', 'w_z',
            ]
        )
        self.robot_control = pd.DataFrame(
            columns=[
                'v_x', 'w_z'
            ]
        )
        self.time = list()

    def init_subs(self):
        """
        Declares node subscribers
        """
        self.control_sub = self.create_subscription(
            Twist,
            self.control_topic,
            self.control_callback,
            1
        )
        self.control_sub

        if self.use_odom_topic:
            self.odom_sub = self.create_subscription(
                Odometry,
                self.odom_topic,
                self.odom_callback,
                1
            )
            self.odom_sub
        else:
            self.velocity_sub = self.create_subscription(
                Twist,
                self.velocity_topic,
                self.velocity_callback,
                1
            )
            self.velocity_sub

    def velocity_callback(self, vel_msg):
        """
        Callback on velocity msg, update current velocity
        :Args:
            :vel_msg: (geometry_msgs.msg.Twist) velpcity msg
        """
        if (len(self.curr_control) == 0):
            return

        from_frame_rel = 'camera_pose_frame'
        to_frame_rel = 'odom_frame'
        try:
            self.curr_velocity = vel_msg
            curr_time = convert_ros2_time_to_float(
                self.get_clock().now().seconds_nanoseconds()
            )
            # update time container
            self.time.append(curr_time - self.init_time)
            print(curr_time - self.init_time)

            self.robot_control.loc[len(self.robot_control)] = self.curr_control
            # update robot_state container
            rosbot_velocities = self.curr_velocity
            rosbot_pose = Pose()

            trans = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time()
            )
            # print(trans.transform.translation.x, trans.transform.translation.y)

            rosbot_pose.position.x = trans.transform.translation.x
            rosbot_pose.position.y = trans.transform.translation.y
            rosbot_pose.position.z = trans.transform.translation.z

            rosbot_pose.orientation.x = trans.transform.rotation.x
            rosbot_pose.orientation.y = trans.transform.rotation.y
            rosbot_pose.orientation.z = trans.transform.rotation.z
            rosbot_pose.orientation.w = trans.transform.rotation.w

            x = rosbot_pose.position.x
            y = rosbot_pose.position.y
            z = rosbot_pose.position.z
            rpy = Rotation.from_quat([
                np.float(rosbot_pose.orientation.x),
                np.float(rosbot_pose.orientation.y),
                np.float(rosbot_pose.orientation.z),
                np.float(rosbot_pose.orientation.w)]
            ).as_euler('xyz')
            rpy = list(rpy)

            v_x = rosbot_velocities.linear.x  # Linear velocity
            v_y = rosbot_velocities.linear.y
            v_z = rosbot_velocities.linear.z

            w_x = rosbot_velocities.angular.x
            w_y = rosbot_velocities.angular.y
            w_z = rosbot_velocities.angular.z  # YAW velocity

            last_row = len(self.robot_state)
            self.robot_state.loc[last_row] = [x, y, z] + \
                rpy + [v_x, v_y, v_z, w_x, w_y, w_z]

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return

    def odom_callback(self, odom_msg):
        """
        Callback on odom message
        Robot position, current time and control are logged
        :Args:
            :odom_msg: (nav_msgs.msg.Odometry): odom msg
        """

        if (len(self.curr_control) == 0):
            return

        curr_time = convert_ros2_time_to_float(
            self.get_clock().now().seconds_nanoseconds()
        )
        # update time container
        self.time.append(curr_time - self.init_time)
        # update control container
        self.robot_control.loc[len(self.robot_control)] = self.curr_control
        # update robot_state container
        rosbot_pose = odom_msg.pose.pose
        rosbot_velocities = odom_msg.twist.twist
        x = rosbot_pose.position.x
        y = rosbot_pose.position.y
        z = rosbot_pose.position.z
        rpy = Rotation.from_quat([
            np.float(rosbot_pose.orientation.x),
            np.float(rosbot_pose.orientation.y),
            np.float(rosbot_pose.orientation.z),
            np.float(rosbot_pose.orientation.w)]
        ).as_euler('xyz')
        rpy = list(rpy)

        v_x = rosbot_velocities.linear.x  # Linear velocity
        v_y = rosbot_velocities.linear.y
        v_z = rosbot_velocities.linear.z

        w_x = rosbot_velocities.angular.x
        w_y = rosbot_velocities.angular.y
        w_z = rosbot_velocities.angular.z  # YAW velocity

        last_row = len(self.robot_state)
        self.robot_state.loc[last_row] = [x, y, z] + \
            rpy + [v_x, v_y, v_z, w_x, w_y, w_z]

    def control_callback(self, control):
        """
        Updates the current control
        Args:
            :control: (geometry_msgs.msg.Twist) control msg
        """
        if self.first_tick:
            self.first_tick = False
            self.init_time = convert_ros2_time_to_float(
                self.get_clock().now().seconds_nanoseconds()
            )

        self.curr_control = [control.linear.x, control.angular.z]

    def save_collected_data_to_csv(self):
        """
        Saves logged data in csv format
        """
        # if not os.path.exists(self.output_path):
        #     os.makedirs(self.output_path)

        self.robot_state.to_csv(
            path_or_buf=os.path.join(self.output_path, "rosbot_state.csv"),
            sep=' ',
            index=False
        )

        self.kinetic_model_state.to_csv(
            path_or_buf=os.path.join(
                self.output_path, "kinematic_model_state.csv"),
            sep=' ',
            index=False
        )

        self.nn_model_state.to_csv(
            path_or_buf=os.path.join(self.output_path, "nn_model_state.csv"),
            sep=' ',
            index=False
        )

        self.robot_control.to_csv(
            path_or_buf=os.path.join(self.output_path, "control.csv"),
            sep=' ',
            index=False
        )

        pd.DataFrame(data=self.time, columns=['t']).to_csv(
            path_or_buf=os.path.join(self.output_path, "time.csv"),
            sep=' ',
            index=False
        )

    def shutdown_logger_callback(self):
        """
        Callback for the shutdown_logger service, 
        turns off the logger node
        """
        rclpy.try_shutdown()

    def on_shutdown(self):
        """
        A function that is executed when a node shutdown.
        Plots a graph of all collected data, saves it in csv format.
        """

        if not os.path.exists(self.output_path):
            os.makedirs(self.output_path)

        data_plots = build_general_graph_for_rosbot(
            robot_state_df=self.robot_state,
            control_df=self.robot_control,
            time_list=self.time,
            save_to_png=True,
            path=self.output_path
        )
        self.save_collected_data_to_csv()

        self.get_logger().warn("Output path = {}".format(self.output_path))


def main():
    """
    Declares the logger node.
    Node works 
    """
    rclpy.init()
    logger = Logger()

    try:
        rclpy.spin(logger)
    except:
        pass

    logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
