import numpy as np
from nav_msgs.msg import Odometry

class ErrorEstimator():
    """
    Class for estimation error of Kalman filter relative to ground truth
    ---------------------------
    Attributes:
    ate_distance: float
        Absolute translation error
    ate_distance_sqr: float
        Absolute translation error squared
    ate_mean: float
    ate_std: float
    ate_num_points: int
        Number of points considered
    """
    def __init__(self):
        self.ate_distance = 0
        self.ate_distance_sqr = 0
        self.ate_mean = 0
        self.ate_std = 0
        self.num_points = 0

    def evaluate_ate(self):
        self.ate_mean = self.ate_distance / self.num_points
        self.ate_std = np.sqrt(self.ate_distance_sqr / (self.num_points))
        return self.ate_mean, self.ate_std

    def compute_curr_ate(self, odom_gt, odom_filtered):
        pose_gt = np.array([odom_gt.pose.pose.position.x, 
                            odom_gt.pose.pose.position.y])
        pose_filtered = np.array([odom_filtered.pose.pose.position.x, 
                                  odom_filtered.pose.pose.position.y])
        dst_sqr = (
            (pose_gt[0] - pose_filtered[0])**2
            +
            (pose_gt[1] - pose_filtered[1])**2
        )
        self.ate_distance += np.sqrt(dst_sqr)
        self.ate_distance_sqr += dst_sqr
        self.num_points += 1
        print(self.ate_distance)
