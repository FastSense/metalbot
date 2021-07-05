from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path
import pathlib
import numpy as np
import rclpy
import math
import time


def IsValidTrajType(traj_type):
    return 'sin' in traj_type or traj_type in ('from_file', 'polygon') or 'spiral' in traj_type


def parse_plan(file_name):
    """
    Parse path from file
    """
    edges = list()
    if file_name == '':
        print("Error file_name")
        return 1
    with (pathlib.Path(file_name)).open() as f:
        for line in f:
            p = line.replace('\n', '')
            if p:
                p = p.split(" ")
                edges.append((float(p[0]), float(p[1])))

    return edges


def edges_to_points(edges):
    points = list()
    p1 = (0.0, 0.0)

    for edge in edges:
        p2 = edge
        k = (p2[1] - p1[1]) / (p2[0] - p1[0])
        b = (p1[1]*p2[0] - p2[1]*p1[0]) / (p2[0] - p1[0])
        x = p1[0]
        y = k*x + b
        points.append((x, y))
        step = abs(p2[0] - p1[0])/10
        if p2[0] > p1[0]:
            while (x < p2[0]):
                x += step
                if (x > p2[0]):
                    break
                y = k*x + b
                points.append((x, y))
        else:
            while (x > p2[0]):
                x -= step
                if (x < p2[0]):
                    break
                y = k*x + b
                points.append((x, y))
        p1 = p2

    return points


def FromFileTrajGenerator(msg: Path, move_plan):
    if move_plan is None:
        print("Move plan was not specified")
        return 1

    edges = parse_plan(move_plan)
    points = edges_to_points(edges)

    for p in points:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x = p[0]
        ps.pose.position.y = p[1]
        ps.pose.position.z = 0.0
        msg.poses.append(ps)

    return msg


def SinTrajGenerator(msg: Path, step: float, a=1.0, f=1.0, reverse=False):
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


def PolygonTrajGenerator(msg: Path, step: float):

    p_edges = [(2.0, -0.1), (2.1, 1.9),  (0.1, 2.0), (0, 0)]  # square

    points = edges_to_points(p_edges)

    for p in points:
        ps = PoseStamped()
        ps.header = msg.header

        ps.pose.position.x = p[0]
        ps.pose.position.y = p[1]
        ps.pose.position.z = 0.0
        ps.pose.orientation = euler_to_quaternion(
            yaw=math.atan2(p[1], p[0]), roll=0, pitch=0)
        msg.poses.append(ps)
    return msg


def SpiralTrajGenerator(msg: Path, step: float, amplitude):

    key_points = []
    if amplitude > 0:
        k = 1
    else:
        k = -1
    amplitude = abs(amplitude)
    f = 0
    while 1:
        r = step * math.exp(f*0.1)
        x = k * r * math.cos(f)
        y = r * math.sin(f)
        if abs(x) > amplitude or abs(y) > amplitude:
            points = edges_to_points(key_points)
            for p in points:
                ps = PoseStamped()
                ps.header = msg.header
                ps.pose.position.x = p[0]
                ps.pose.position.y = p[1]
                ps.pose.position.z = 0.0
                msg.poses.append(ps)
            return msg
        else:
            key_points.append([x, y])
            f += 0.1


def parse_sin_traj(traj_type: str):
    traj_type = traj_type.strip().split('sin')  # []
    period = float(traj_type[-1])
    amplitude = float(traj_type[0].split("_")[-1])
    reverse = traj_type[0].split("_")[0] == 'reverse'
    return amplitude, period, reverse


def parse_spiral_traj(traj_type: str):
    coef = traj_type.split('spiral')
    amp = coef[0] if coef[0] != '' or coef[0] is None else 1.0
    return float(amp)


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
    node.declare_parameter('traj_type')
    node.declare_parameter('move_plan')

    traj_type = node.get_parameter(
        'traj_type').get_parameter_value().string_value
    move_plan = node.get_parameter(
        'move_plan').get_parameter_value().string_value
    if not IsValidTrajType(traj_type):
        node.get_logger().info("Not valid type of trajectory")
        return 1
    
    path_pub = node.create_publisher(Path, "/path", 5)
    msg = Path()
    msg.header.frame_id = "odom"
    msg.poses = []
    step = 0.1
    print("Parse ", traj_type)
    if "sin" in traj_type:
        amplitude, freq, reverse = parse_sin_traj(traj_type)
        msg = SinTrajGenerator(msg, step, amplitude, freq, reverse)
    elif "spiral" in traj_type:
        amplitude = parse_spiral_traj(traj_type)
        msg = SpiralTrajGenerator(msg, step, amplitude)
    elif traj_type == "polygon":
        msg = PolygonTrajGenerator(msg, step)
    elif traj_type == "from_file":
        print(f"file_name = {move_plan}")
        msg = FromFileTrajGenerator(msg, move_plan)

    while path_pub.get_subscription_count() < 1:
        time.sleep(0.5)
    time.sleep(0.1)

    path_pub.publish(msg)

    time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()
    return


if __name__ == '__main__':
    main()
