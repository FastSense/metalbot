import rclpy
from rclpy.node import Node
import cv2
from nav_msgs.msg import Odometry
import numpy as np
import random

class Noiser(Node):
    """
    ROS node for adding noise to ground truth odometry
    -----------------------------
    Attributes:
    odom_sub: ros::Subscriber
        Subscriber to /odom topic for ground truth odometry
    pose_pub: ros::Publisher
        Publisher od noised odometry
    odom_noised: Odometry
        Noised odometry
    x_sum: float
        Integrated x coordinate
    y_sum: float
        Integrated y coordinate
    theta_sum: float
        Integrated yaw angle
    dt: float
        Time step corresponding to /odom publication frequency
    lin_vel_noise_std: float
        Standart deviation of linear velocity noise
    rot_vel_noise_std: float
        Standart deviation of angular velocity noise
    """
    def __init__(self):
        super().__init__('odom_noiser')

        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odometry_callback,
            10)
        self.pose_pub = self.create_publisher(Odometry, '/odom_noised', 10)
        self.odom_noised = Odometry()
        self.x_sum = 0
        self.y_sum = 0
        self.theta_sum = 0
        self.dt = 0.1
        self.lin_vel_noise_std = 0.5
        self.rot_vel_noise_std = 0.1

    def odometry_callback(self, msg):
        self.odom_noised.header = msg.header
        self.odom_noised.child_frame_id = msg.child_frame_id
        self.odom_noised.pose.covariance = msg.pose.covariance
        self.odom_noised.twist.covariance = msg.twist.covariance
        # Noise adding process
        vel_noised_x = (msg.twist.twist.linear.x + 
                        random.gauss(0, self.lin_vel_noise_std))
        vel_noised_theta = (msg.twist.twist.angular.z + 
                            random.gauss(0, self.rot_vel_noise_std))
        # Integration of velocities
        self.theta_sum = self.theta_sum + vel_noised_theta * self.dt
        self.x_sum = self.x_sum + vel_noised_x * np.cos(self.theta_sum) * self.dt
        self.y_sum = self.y_sum + vel_noised_x * np.sin(self.theta_sum) * self.dt
        self.odom_noised.pose.pose.position.x = self.x_sum 
        self.odom_noised.pose.pose.position.y = self.y_sum 
        self.odom_noised.pose.pose.position.z = msg.pose.pose.position.z
        self.odom_noised.pose.pose.orientation = msg.pose.pose.orientation
        self.odom_noised.pose.pose.orientation.z = self.theta_sum
        self.odom_noised.twist.twist.linear.x = vel_noised_x
        self.odom_noised.twist.twist.linear.y = msg.twist.twist.linear.y
        self.odom_noised.twist.twist.linear.z = msg.twist.twist.linear.z
        self.odom_noised.twist.twist.angular.x = msg.twist.twist.angular.x
        self.odom_noised.twist.twist.angular.y = msg.twist.twist.angular.y
        self.odom_noised.twist.twist.angular.z = vel_noised_theta
        self.pose_pub.publish(self.odom_noised)

def main():
    rclpy.init()
    noiser = Noiser()
    try:
        rclpy.spin(noiser)
    except KeyboardInterrupt:
        rclpy.logerr("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
