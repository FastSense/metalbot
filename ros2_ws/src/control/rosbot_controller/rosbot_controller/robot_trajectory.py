from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pathlib
import numpy as np
import math
import time
import errno
import os
from scipy.spatial.transform import Rotation
from rosbot_controller.rosbot_2D import Goal

# https://axiacore.com/blog/mathematical-expression-evaluator-python-524/
# https://nerdparadise.com/programming/parsemathexpr


class TrajectoryType():
    pass


class Trajectory():
    """

    """

    def __init__(self, path=Path(), step=0.1, frame="odom"):
        """
        """
        self.points_ = path
        self.step_ = step
        self.points_.header.frame_id = frame
        self.default_polygonal_way_points = [
            (2.0, 0.0), (2.0, 2.0),  (0.0, 2.0), (0, 0)
        ]
        self.default_sin = "1.0sin1.0"

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
        pass

    def parse_move_plan(self, mp_path):
        """
        """
        way_points = list()
        with (pathlib.Path(mp_path)).open() as f:
            for line in f:
                way_points.append(
                    self.str_to_way_point(line)
                )

        self.fill_path_with_way_points(way_points)

    def str_to_way_point(self, line):
        """
        """
        line = [float(l) for l in line.split(" ")]
        return Goal(line[0], line[1])

    def fill_path_with_way_points(self, way_points):
        """
        """
        prev_wp = way_points[0]

        for next_wp in way_points[1:]:

            line_along_y_axis = self.is_line_along_y_axis(prev_wp, next_wp)
            if not line_along_y_axis:
                k, b = self.calculate_line_coefficients(prev_wp, next_wp)

            x = prev_wp.x
            y = prev_wp.y if line_along_y_axis else k * x + b
            yaw_quat = self.calculate_yaw_quat_for_points((prev_wp, next_wp))
            self.add_point_to_path(x, y, yaw_quat)

            step = min(abs(next_wp.x - prev_wp.x), self.step)
            if not line_along_y_axis:
                if next_wp.x > prev_wp.x:
                    while (x < next_wp.x-step):
                        x += step
                        y = k * x + b
                        self.add_point_to_path(x, y, yaw_quat)
                else:
                    while (x > next_wp.x+step):
                        x -= step
                        y = k * x + b
                        self.add_point_to_path(x, y, yaw_quat)
            else:
                if next_wp.y > prev_wp.y:
                    while (y < next_wp.y-step):
                        y += step
                        self.add_point_to_path(x, y, yaw_quat)
                else:
                    while (y > next_wp.y+step):
                        y -= step
                        self.add_point_to_path(x, y, yaw_quat)

            prev_wp = next_wp

    def add_point_to_path(self, x, y, yaw_quat):
        """
        """
        point = PoseStamped()
        point.header = self.msg.header
        point.pose.position.x = x
        point.pose.position.y = y
        point.pose.position.z = 0.0
        point.pose.orientation.x = yaw_quat[0]
        point.pose.orientation.y = yaw_quat[1]
        point.pose.orientation.z = yaw_quat[2]
        point.pose.orientation.w = yaw_quat[3]
        self.points_.poses.append(point)

    def calculate_line_coefficients(prev_point, next_point):
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

    def is_line_along_y_axis(self, prev_point, next_point):
        """
        """
        return next_point.x - prev_point.x == 0

    def Set_polygonal_path(self, way_points_list=None):
        """
        """
        way_points = list()
        if way_points_list is None:
            way_points_list = self.default_polygonal_way_points

        for point in way_points_list:
            way_points.append(Goal(point[0], point[1]))

        self.fill_path_with_way_points(way_points)

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
        #     y_ar = float(a) * np.sin(float(f) * x_ar)
        #     yaw_arr = float(a) * float(f) * np.cos(float(f) * x_ar)

        #     for i in range(len(x_ar)):
        #         ps = PoseStamped()
        #         ps.header = self.msg.header
        #         ps.pose.position.x = x_ar[i]
        #         ps.pose.position.y = y_ar[i]
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
        #     key_points = []
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
        #             points = self.edges_to_points(key_points)
        #             for p in points:
        #                 ps = PoseStamped()
        #                 ps.header = self.msg.header
        #                 ps.pose.position.x = p[0]
        #                 ps.pose.position.y = p[1]
        #                 ps.pose.position.z = 0.0
        #                 self.msg.poses.append(ps)
        #             return
        #         else:
        #             key_points.append([x, y])
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
