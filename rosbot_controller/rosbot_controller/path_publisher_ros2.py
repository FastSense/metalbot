import os
from geometry_msgs.msg import PoseStamped, Quaternion, Pose, PoseArray
from nav_msgs.msg import Path


import numpy as np
import rclpy
import math
import time


def SinTrajGenerator(msg: Path, step, a=1.0, f=1.0, reverse=False):
    K = -1 if reverse == True else 1
    x_ar = np.arange(0, 2*np.pi * K, step * K, dtype=float)   # start,stop,step
    y_ar = float(a) * np.sin(float(f) * x_ar)
    yaw_arr = float(a) * float(f) * np.cos(float(f) * x_ar)

    for i in range(len(x_ar)):
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = x_ar[i]
        ps.pose.position.y = y_ar[i]
        ps.pose.position.z = 0.0
        ps.pose.orientation = euler_to_quaternion(
            yaw=math.atan(yaw_arr[i]), roll=0, pitch=0)
        msg.poses.append(ps)


    return msg


def parse_sin_traj(traj_type):
    traj_type = traj_type.strip().split('sin')  # []
    period = float(traj_type[-1])
    amplitude = float(traj_type[0].split("_")[-1])
    reverse = traj_type[0].split("_")[0] == 'reverse'
    return amplitude, period, reverse


def euler_to_quaternion(yaw, pitch, roll):
    """
    Args:
        yaw: yaw angle
        pitch: pitch angle
        roll: roll angle
    Return:
        quaternion [qx, qy, qz, qw]
    """
    qx = np.sin(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) - np.cos(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)
    qy = np.cos(roll / 2) * np.sin(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.cos(
        pitch / 2) * np.sin(yaw / 2)
    qz = np.cos(roll / 2) * np.cos(pitch / 2) * np.sin(yaw / 2) - np.sin(roll / 2) * np.sin(
        pitch / 2) * np.cos(yaw / 2)
    qw = np.cos(roll / 2) * np.cos(pitch / 2) * np.cos(yaw / 2) + np.sin(roll / 2) * np.sin(
        pitch / 2) * np.sin(yaw / 2)

    return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main():
    rclpy.init(args=None)
    node = rclpy.create_node("path_pub")
    node.get_logger().info("path_pub init")
    traj_type = "2.0sin2.0"

    path_topic_name = "/path"

    path_pub = node.create_publisher(Path, path_topic_name, 5)

    msg = Path()

    msg.header.frame_id = "odom"

    msg.poses = []

    step = 0.1

    print("TRY PARSE", traj_type)
    amplitude, freq, reverse = parse_sin_traj(traj_type)
    msg = SinTrajGenerator(msg, step, amplitude, freq, reverse)
    # print(msg)

    path_pub.publish(msg)
    return


if __name__ == '__main__':
    main()
