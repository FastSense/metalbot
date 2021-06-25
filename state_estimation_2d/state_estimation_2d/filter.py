#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import math

from state_estimation_2d.model import *
from state_estimation_2d.measurement import *

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
    def __init__(self, x_init, P_init, R_odom, R_imu, Q, dt = 0.1):
        self.z_odom = np.zeros(3)
        self.z_imu = np.zeros(2)
        self.R_odom = R_odom
        self.R_imu = R_imu
        self.Q = Q
        self.x_opt = x_init
        self.P_opt = P_init
        self.dt = dt
        self.control = np.zeros(2)
        self.v = 0
        self.w = 0
        self.eps_w = 0.001

    def set_control(self, msg):
        """
        Set velocity control from /cmd_vel
        @ parameters
        msg: Twist
            Velocity
        """
        self.control[0] = msg.linear.x
        self.control[1] = msg.angular.z

    def set_odometry(self, z):
        """
        Set odometry measurement vector
        @ parameters:
        z: np.array
            Odometry measurement vector
        """
        self.z_odom = z
    
    def set_imu(self, z):
        """
        Set imu measurement vector
        @ parameters:
        z: np.array
            Imu measurement vector
        """
        self.z_imu = z

    def update_state_by_nn_model(self, model):
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
        model_input = np.array([[self.v, self.w, self.control[0], self.control[1], self.dt]], 
                                dtype=np.float32)
        model_output = model(model_input)
        # Predicted velocity control
        self.v, self.w = float(model_output[0][0]), float(model_output[0][1])
        vel_vector = np.array([self.v, self.w])
        self.update_state_by_model(vel_vector)
        return self.x_opt, self.P_opt

    def update_state_by_model(self, control):
        """
        Kalman filter predict step equations using dynamic model
        @ parameters:
        control : control vector from NN model
        """
        v = control[0]
        w = control[1]

        if abs(w) > self.eps_w:
            rho = v / w

            # step 1. Calc new robot position relative to its previous pose
            x_r = rho * math.sin(w * self.dt)
            y_r = rho * (1 - math.cos(w * self.dt))

            # step 2. Transfrom this point to map fixed coordinate system taking into account
            # current robot pose
            self.x_opt[0] += x_r * math.cos(self.x_opt[4]) - y_r * math.sin(self.x_opt[4])
            self.x_opt[1] += x_r * math.sin(self.x_opt[4]) + y_r * math.cos(self.x_opt[4])
            self.x_opt[4] += w * self.dt
        else:
            self.x_opt[0] += v * self.dt * math.cos(self.x_opt[4])
            self.x_opt[1] += v * self.dt * math.sin(self.x_opt[4])
        
        self.x_opt[2] = control[0]
        self.x_opt[3] = control[1]
        self.P_opt = self.predict_covariance()
    
    def predict_covariance(self):
        """Computes covariance matrix after predict step"""
        P_predict = np.zeros((6, 6))
        J_f = transform_jacobian(self.x_opt, self.dt)
        P_predict = J_f @ self.P_opt @ J_f.T + self.Q
        return P_predict

    def update(self):
        """ Kalman Filter update step"""
        self.update_odom()
        self.update_imu()
        return self.x_opt, self.P_opt

    def update_odom(self):
        """ Update state vector using odometry measurements"""
        H = get_jacobian_odom(self.x_opt)   
        y = self.z_odom - H @ self.x_opt
        G = H @ self.P_opt @ H.T + self.R_odom
        K = self.P_opt @ H.T @ inv(G)
        I = np.eye(6)
        self.P_opt = (I - K @ H) @ self.P_opt
        self.x_opt = self.x_opt + K @ y

    def update_imu(self):
        """ Update state vector using imu measurements"""
        H = get_jacobian_imu(self.x_opt) 
        y = self.z_imu - imu(self.x_opt)
        G = H @ self.P_opt @ H.T + self.R_imu
        K = self.P_opt @ H.T @ inv(G)
        I = np.eye(6)
        self.P_opt = (I - K @ H) @ self.P_opt
        self.x_opt = self.x_opt + K @ y