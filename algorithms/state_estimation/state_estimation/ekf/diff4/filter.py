#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import math
import scipy

from state_estimation_2d.model import *
from state_estimation_2d.measurement import *

"""
State vector:
0. x
1. y
2. V_parallel       (velocity along the robot direction)
3. yaw
4. yaw_vel

Measurements:
Odometry:
0. V_parallel
1. yaw_vel

IMU:
0. a_normal         (acceleration orthogonal to the robot direction)
1. yaw_vel
"""

class Filter2D:
    """
    Class with Extended Kalman Filter implementation
    Theory: https://homes.cs.washington.edu/~todorov/courses/cseP590/readings/tutorialEKF.pdf
    ----------------------------
    Attributes:
    z_odom: np.array
        Odometry measurement vector
    z_imu: np.array
        Imu measurement vector
    R_odom: np.array
        Odometry measurement noise covariance matrix
    R_imu: np.array
        Imu measurement noise covariance matrix
    Q: np.array
        Model noise covariance matrix
    x_opt: np.array
        Computed optimal state vector
    P_opt: np.array
        Computed covariance matrix for state vector
    dt: float
        Time step
    control:
        Vector of controls from /cmd_vel
    v: np.array
        Previous linear velocity control
    w: np.array
        Previous angular velocity control
    eps_w: float
        Threshold for angular velocity
    """
    def __init__(self, x_init, P_init, dt, v_var, w_var):
        self.x_opt = x_init
        self.P_opt = P_init
        self.dt = dt
        self.control = np.zeros(2)
        self.eps_w = 0.001
        
        # Process noise
        Q_rot = np.array([
            [0.333 * dt**3, 0.5 * dt**2],
            [  0.5 * dt**2,          dt],
        ]) * w_var
        Q_vel = np.array([dt]) * v_var
        self.Q = scipy.linalg.block_diag(
            np.zeros((2,2)),
            Q_vel,
            Q_rot,
        )

    def predict_by_nn_model(self, model, control):
        """
        Kalman filter predict step
        @ parameters
        model: 
            Pretrained NN control model
        @ return 
        x_opt:
            State vector after predict step
        P_opt:
            Covariance matrix after predict step
        """
        model_input = np.array([[self.v, self.w, control[0], control[1], self.dt]], 
                                dtype=np.float32)
        model_output = model(model_input)
        # Predicted velocity control
        self.v, self.w = float(model_output[0][0]), float(model_output[0][1])
        self.predict()

    def predict_by_naive_model(self, control):
        self.v = control[0]
        self.w = control[1]
        self.predict()

    def predict(self):
        """
        Kalman filter predict step equations using dynamic model
        """

        if abs(self.w) > self.eps_w:
            rho = self.v / self.w

            # step 1. Calc new robot position relative to its previous pose
            x_r = rho * math.sin(self.w * self.dt)
            y_r = rho * (1 - math.cos(self.w * self.dt))

            # step 2. Transfrom this point to map fixed coordinate system taking into account
            # current robot pose
            self.pos[0] += x_r * math.cos(self.yaw) - y_r * math.sin(self.yaw)
            self.pos[1] += x_r * math.sin(self.yaw) + y_r * math.cos(self.yaw)
            self.yaw += self.w * self.dt
        else:
            self.pos[0] += self.v * self.dt * math.cos(self.yaw)
            self.pos[1] += self.v * self.dt * math.sin(self.yaw)
        
        self.P_opt = self.predict_covariance()
    
    def predict_covariance(self):
        """Computes covariance matrix after predict step"""
        P_predict = np.zeros((5, 5))
        F = transform_jacobian(self.x_opt, self.dt)
        P_predict = F @ self.P_opt @ F.T + self.Q
        return P_predict

    def update_odom(self, z_odom, R_odom):
        """ Update state vector using odometry measurements"""
        H = get_jacobian_odom(self.x_opt)   
        y = z_odom - H @ self.x_opt
        G = H @ self.P_opt @ H.T + R_odom
        K = self.P_opt @ H.T @ inv(G)
        I = np.eye(5)
        self.P_opt = (I - K @ H) @ self.P_opt
        self.x_opt = self.x_opt + K @ y

    def update_imu(self, z_imu, R_imu):
        """ Update state vector using imu measurements"""
        H = get_jacobian_imu(self.x_opt) 
        y = z_imu - imu(self.x_opt)
        G = H @ self.P_opt @ H.T + R_imu
        K = self.P_opt @ H.T @ inv(G)
        I = np.eye(5)
        self.P_opt = (I - K @ H) @ self.P_opt
        self.x_opt = self.x_opt + K @ y
    
    @property
    def v(self):
        return self.x_opt[2]
    @v.setter
    def v(self, value):
        self.x_opt[2] = value
    
    @property
    def w(self):
        return self.x_opt[4]
    @w.setter
    def w(self, value):
        self.x_opt[4] = value

    @property
    def pos(self):
        return self.x_opt[:2]
    @pos.setter
    def pos(self, value):
        self.x_opt[:2] = value

    @property
    def yaw(self):
        return self.x_opt[3]
    @yaw.setter
    def yaw(self, value):
        self.x_opt[3] = value
