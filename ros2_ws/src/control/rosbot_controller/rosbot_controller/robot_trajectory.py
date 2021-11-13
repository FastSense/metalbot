from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pathlib
import numpy as np
import math
import time
import errno
import os
from enum import Enum
from scipy.spatial.transform import Rotation
from rosbot_controller.rosbot_2D import Goal

# https://axiacore.com/blog/mathematical-expression-evaluator-python-524/
# https://nerdparadise.com/programming/parsemathexpr


class TrajectoryTypes(Enum):
    sin = 'sin'
    polygon = 'polygon'
    spiral = 'spiral'
    points = 'points'


class Trajectory():
    """

    """

    def __init__(self, path=Path(), step=0.1, frame="odom", length=4.0):
        """
        """
        self.points_ = path
        self.step_ = step
        self.points_.header.frame_id = frame
        self.length = length
        self.valid_trajectories = TrajectoryTypes
        self.default_polygonal_waypoints = [
            (0, 0), (1.5, 0.0), (1.5, 1.5),  (0.0, 1.5), (0, 0)
        ]

    def set_step(self, new_step: float):
        """
        """
        self.step_ = new_step

    def set_frame(self, frame_name: str):
        """
        """
        self.points_.header.frame_id = frame_name

    def get_path(self):
        """
        """
        return self.points_

    def set_path(self, path: Path):
        """
        """
        self.points_ = path

    def parse_string(self, input_str):
        """
        """
        traj_type = self.get_traj_type(input_str)
        if traj_type is None:
            print("!!!!!!!!!!!!")

        if traj_type == self.valid_trajectories.polygon:
            self.create_polygon_trajectory(input_str)
        elif traj_type == self.valid_trajectories.sin:
            self.create_sin_trajectory(input_str)
        elif traj_type == self.valid_trajectories.spiral:
            self.create_spiral_trajectory(input_str)
        elif traj_type == self.valid_trajectories.spiral:
            self.create_trajectory_from_points(input_str)

    def get_traj_type(self, input_string):
        """
        """
        for traj in self.valid_trajectories:
            if traj.value in input_string:
                return traj
        return None

    def create_sin_trajectory(self, input_str):
        """
        """
        input_str = input_str.split(self.valid_trajectories.sin.value)

        ampl, freq = [float(k) if k != '' else 1.0 for k in input_str]

        x_seq = np.arange(0, self.length + self.step_, self.step_, dtype=float)
        y_seq = ampl * np.sin(freq * x_seq)
        yaw_seq = ampl * freq * np.cos(freq * x_seq)

        for i in range(x_seq.size):
            self.add_point_to_path(x_seq[i], y_seq[i], yaw_seq[i])

    def parse_move_plan(self, mp_path):
        """
        """
        waypoints = list()
        with (pathlib.Path(mp_path)).open() as f:
            for line in f:
                waypoints.append(
                    self.str_to_waypoint(line)
                )

        self.fill_path_with_waypoints(waypoints)

    def str_to_waypoint(self, line):
        """
        """
        line = [float(l) for l in line.split(" ")]
        return Goal(line[0], line[1])

    def fill_path_with_waypoints(self, waypoints):
        """
        """
        prev_wp = waypoints[0]

        for next_wp in waypoints[1:]:

            line_along_yaxis = self.is_line_along_yaxis(prev_wp, next_wp)

            if not line_along_yaxis:
                k, b = self.calculate_line_coefficients(prev_wp, next_wp)

            x = prev_wp.x
            y = prev_wp.y if line_along_yaxis else k * x + b
            yaw_quat = self.calculate_yaw_quat_for_points(prev_wp, next_wp)
            self.add_point_to_path(x, y, yaw_quat)

            if not line_along_yaxis:
                if next_wp.x > prev_wp.x:
                    while (x < next_wp.x):
                        x += self.step_
                        y = k * x + b
                        self.add_point_to_path(x, y, yaw_quat)
                else:
                    while (x > next_wp.x):
                        x -= self.step_
                        y = k * x + b
                        self.add_point_to_path(x, y, yaw_quat)
            else:
                if next_wp.y > prev_wp.y:
                    while (y < next_wp.y):
                        y = y + self.step_
                        self.add_point_to_path(x, y, yaw_quat)
                else:
                    while (y > next_wp.y):
                        y = y - self.step_
                        self.add_point_to_path(x, y, yaw_quat)
            prev_wp = next_wp

    def add_point_to_path(self, x, y, yaw_quat):
        """
        """
        if isinstance(yaw_quat, float):
            yaw_quat = Rotation.from_euler('z', yaw_quat, degrees=False)
            yaw_quat = list(yaw_quat.as_quat())

        point = PoseStamped()
        point.header = self.points_.header
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0.0
        point.pose.orientation.x = yaw_quat[0]
        point.pose.orientation.y = yaw_quat[1]
        point.pose.orientation.z = yaw_quat[2]
        point.pose.orientation.w = yaw_quat[3]
        self.points_.poses.append(point)

    def calculate_line_coefficients(self, prev_point, next_point):
        """
        """
        k = (next_point.y - prev_point.y) / (next_point.x - prev_point.x)

        b = (prev_point.y*next_point.x - next_point.y *
             prev_point.x) / (next_point.x - prev_point.x)
        return k, b

    def calculate_yaw_quat_for_points(self, prev_point, next_point):
        """
        """
        yaw = math.atan2(next_point.y - prev_point.y,
                         next_point.x - prev_point.x)
        yaw_quat = Rotation.from_euler('z', yaw, degrees=False)
        yaw_quat = list(yaw_quat.as_quat())
        return yaw_quat

    def is_line_along_yaxis(self, prev_point, next_point):
        """
        """
        return next_point.x - prev_point.x == 0

    def Set_polygonal_path(self, waypoints_list=None):
        """
        """
        waypoints = list()
        if waypoints_list is None:
            waypoints_list = self.default_polygonal_waypoints

        for point in waypoints_list:
            waypoints.append(Goal(point[0], point[1]))

        self.fill_path_with_waypoints(waypoints)

        # def SinTrajGenerator(self, a=1.0, f=1.0, reverse=False):
        #     """
        #     Generating a sinus trajectory (a*sin(f*t))
        #     Args:
        #         a: amplitude of a sinus
        #         f: phase of a sinus

        #     """
        #     K = -1 if reverse == True else 1
        #     x_ar = np.arange(0, 2*np.pi * K, self.step * K,
        #                      dtype=float)   # start,stop,step
        #     yar = float(a) * np.sin(float(f) * x_ar)
        #     yaw_arr = float(a) * float(f) * np.cos(float(f) * x_ar)

        #     for i in range(len(x_ar)):
        #         ps = PoseStamped()
        #         ps.header = self.msg.header
        #         ps.pose.position.x = x_ar[i]
        #         ps.pose.position.y = yar[i]
        #         ps.pose.position.z = 0.0
        #         goal_quaternion = list(Rotation.from_euler(
        #             'z', math.atan(yaw_arr[i]), degrees=False
        #         ).as_quat())
        #         ps.pose.orientation.x = goal_quaternion[0]
        #         ps.pose.orientation.y = goal_quaternion[1]
        #         ps.pose.orientation.z = goal_quaternion[2]
        #         ps.pose.orientation.w = goal_quaternion[3]
        #         self.msg.poses.append(ps)

        # def SpiralTrajGenerator(self, amplitude):
        #     """
        #     Generating a spiral trajectory
        #     Args:
        #         amplitude: amplitude of spiral

        #     """
        #     keypoints = []
        #     if amplitude > 0:
        #         k = 1
        #     else:
        #         k = -1
        #     amplitude = abs(amplitude)
        #     f = 0
        #     while 1:
        #         r = self.step * math.exp(f*0.1)
        #         x = k * r * math.cos(f)
        #         y = r * math.sin(f)
        #         if abs(x) > amplitude or abs(y) > amplitude:
        #             points = self.edges_to_points(keypoints)
        #             for p in points:
        #                 ps = PoseStamped()
        #                 ps.header = self.msg.header
        #                 ps.pose.position.x = p[0]
        #                 ps.pose.position.y = p[1]
        #                 ps.pose.position.z = 0.0
        #                 self.msg.poses.append(ps)
        #             return
        #         else:
        #             keypoints.append([x, y])
        #             f += 0.1

        # def parse_sin_traj(self):
        #     """
        #     Parsing sinus trajectory

        #     """
        #     traj_type = self.traj_type.strip().split('sin')  # []
        #     period = float(traj_type[-1])
        #     amplitude = float(traj_type[0].split("_")[-1])
        #     reverse = traj_type[0].split("_")[0] == 'reverse'
        #     return amplitude, period, reverse

        # def parse_spiral_traj(self):
        #     """
        #     Parsing spiral trajectory

        #     """
        #     coef = self.traj_type.split('spiral')
        #     amp = coef[0] if coef[0] != '' or coef[0] is None else 1.0
        #     return float(amp)
