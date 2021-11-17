import math
import pathlib
import numpy as np
from enum import Enum
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from rosbot_controller.rosbot_2D import Goal
from rosbot_controller.rosbot_2D import RobotState


class TrajectoryTypes(Enum):
    """
    Class for representing all
    possible types of trajectories
    """
    sin = 'sin'
    polygon = 'polygon'
    spiral = 'spiral'
    points = 'points'


class Trajectory():
    """
    Class for representing and creating trajectories
    :fields:
        :points_: (Path) - trajectory
        :step_: (float) - step between points in trajectory
        :length: (float) - trajectory length (only for sin)
        :valid_trajectories: (TrajectoryTypes) - all possible trajectories
        :default_polygonal_points: (numpy array) - default edges for polygonal trajectory
    """

    def __init__(self, start_point=RobotState(),
                 path=Path(), step=0.1, frame="odom", length=3.0):
        """
        : Args:
            : path: - default path
            : step: - step between points in trajectory
            : frame: - TF frame of th trajectory
            : length: - trajectory length (only for sin)
        """
        self.start_point = start_point
        self.points_ = path
        self.step_ = step
        self.points_.header.frame_id = frame
        self.length = length
        self.valid_trajectories = TrajectoryTypes
        self.default_polygonal_points = np.array(
            [(0, 0), (1.0, 0.0), (1.0, 1.0),  (0.0, 1.0), (0, 0)])

    def set_step(self, new_step: float):
        """
        Set the step size of the trajectory
        : Args:
            : new_step: - new value of the step
        """
        self.step_ = new_step

    def set_frame(self, frame_name: str):
        """
        Set the TF frame of the trajectory
        : Args:
            : frame_name: - new name of the TF frame
        """
        self.points_.header.frame_id = frame_name

    def set_path(self, path: Path):
        """
        """
        self.points_ = path

    def get_path(self):
        """
        Return the trajectory
        : Return:
            : points_: - (Path), trajectory points
        """
        return self.points_

    def from_string(self, input_str: str):
        """
        Gets the type of trajectory from the passed string.
        Depending on the type of trajectory,
        calls the required method to fill the trajectory with points.
        : Args:
            : input_str: - trajectory name with parameters
        """
        traj_type = self.get_traj_type(input_str)
        if traj_type is None:
            raise Exception("Unknown trajectory!")

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
        Return one of the possible trajectories.
        If the trajectory is unknown, returns None
        : Return:
            : traj: one of the types of trajectories from TrajectoryTypes
        """
        for traj in self.valid_trajectories:
            if traj.value in input_string:
                return traj
        return None

    def create_polygon_trajectory(self, input_str: str):
        """
        Parse string with "polygon" trajectory.
        Fills the path with points
        : Args:
            : input_str: - trajectory type with parameters.
            For exmaple: polygon, 1.5polygon or 3.0polygon,
            format: $SIDE_SIZE + polygon
        """
        side_size = input_str.split(self.valid_trajectories.polygon.value)[0]
        side_size = float(side_size) if side_size != '' else 1.0
        polygon_points = self.default_polygonal_points * \
            side_size + [self.start_point.x, self.start_point.y]
        waypoints = list()
        for point in polygon_points:
            wp = Goal(point[0], point[1], self.start_point.yaw)
            waypoints.append(wp)

        self.fill_path_with_waypoints(waypoints)

    def create_sin_trajectory(self, input_str):
        """
        Parse string with "sin" trajectory.
        Fills the path with points
        : Args:
            : input_str: - trajectory type with parameters.
            For exmaple: sin, 1.5sin or 3.0sin0.5,
            format: $Amplitude + sin + frequency
        """
        input_str = input_str.split(self.valid_trajectories.sin.value)
        ampl, freq = [float(k) if k != '' else 1.0 for k in input_str]

        x_seq = np.arange(0, self.length + self.step_, self.step_, dtype=float)
        y_seq = ampl * np.sin(freq * x_seq)
        yaw_seq = ampl * freq * np.cos(freq * x_seq)

        x_seq = x_seq + self.start_point.x
        y_seq = y_seq + self.start_point.y
        yaw_seq = yaw_seq + self.start_point.yaw
        for i in range(x_seq.size):
            self.add_point_to_path(x_seq[i], y_seq[i], yaw_seq[i])

    def create_spiral_trajectory(self, input_str):
        """
        Parse string with "spiral" trajectory.
        Fills the path with points
        : Args:
            : input_str: - trajectory type with parameters.
            For exmaple: spiral, 1.0spiral or 3.0spiral,
            format: $Amplitude + spiral
        """
        ampl = input_str.split(self.valid_trajectories.spiral.value)[0]
        ampl = float(ampl) if ampl != '' else 1.0

        f = 0.
        x, y = self.start_point.x, self.start_point.y
        ampl_x = ampl + abs(x)
        while (ampl_x > abs(x)):
            r = self.step_ * math.exp(f*0.1)
            x = r * math.cos(f) + self.start_point.x
            y = r * math.sin(f) + self.start_point.y
            f += 0.1
            self.add_point_to_path(x, y, 0.)

    def from_move_plan(self, mp_path):
        """
        Parse move plan (.txt doc with waypoints)
        Fills the path with points
        : Args:
            :mp_path: - absolute path to the file
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
        Convert string to waypoint(type=Goal)
        : Args:
            : line: input string 'x y'
        : Return:
            : Goal(x, y):
        """
        line = [float(l) for l in line.split(" ")]
        x = line[0] + self.start_point.x
        y = line[1] + self.start_point.y
        return Goal(x, y)

    def fill_path_with_waypoints(self, waypoints):
        """
        Fill trajectory from a list with waypoints.
        Also add extra helpful points between waypoints.
        : Args:
            : waypoints: (List[Goal]) - list with waypoints
        """
        prev_wp = waypoints[0]
        for next_wp in waypoints[1:]:
            line_along_yaxis = self.is_line_along_y_axis(prev_wp, next_wp)
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
        Add one point to the path
        : Args:
            : x: - (float) x coord
            : y: - (float) y coord
            : yaw_quat: - (float or list[qaternion]) yaw angle
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
        Сalculate line coefficients K and B
        y = K*x + B
        : Args:
            : prev_point: (Goal) - previous waypoint
            : next_point: (Goal) - next waypoint
        : Return:
            : k: (float) - slope
            : b: (float) - some number
        """
        k = (next_point.y - prev_point.y) / (next_point.x - prev_point.x)

        b = (prev_point.y*next_point.x - next_point.y *
             prev_point.x) / (next_point.x - prev_point.x)
        return k, b

    def calculate_yaw_quat_for_points(self, prev_point, next_point):
        """
        Calculates the yaw angle that the robot
        must roll over to move between waypoints
        : Args:
            : prev_point: (Goal) - previous waypoint
            : next_point: (Goal) - next waypoint
        : Return:
            : yaw_quat: (quaternion) - yaw angle
        """
        yaw = math.atan2(next_point.y - prev_point.y,
                         next_point.x - prev_point.x)
        yaw_quat = Rotation.from_euler('z', yaw, degrees=False)
        yaw_quat = list(yaw_quat.as_quat())
        return yaw_quat

    def is_line_along_y_axis(self, prev_point, next_point):
        """
        Returns true if a straight line through
        two points lies along the Y-axis
        : Args:
            : prev_point: (Goal) - previous waypoint
            : next_point: (Goal) - next waypoin
        """
        return next_point.x - prev_point.x == 0
