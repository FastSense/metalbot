#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import math

from state_estimation_2d.model import *
from state_estimation_2d.measurement import *

class Filter2D:
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

    def update_state_by_model(self, control):
        """
        Updates robot state assuming that control has been active for dt seconds
        c : control vector of RobotControl type
        dt : time period in seconds
        """
        v = control[0]
        w = control[1]

        if abs(w) > self.eps_w:
            rho = v / w

            # step 1. Calc new robot position relative to its previous pose
            x_r = rho * math.sin(w * self.dt)
            y_r = rho * (1 - math.cos(w * self.dt))

            # step 2. Transfrom this point to map fixed coordinate system taking into account
            # current robot poseJ_h_imu
            self.x_opt[0] += x_r * math.cos(self.x_opt[4]) - y_r * math.sin(self.x_opt[4])
            self.x_opt[1] += x_r * math.sin(self.x_opt[4]) + y_r * math.cos(self.x_opt[4])
            self.x_opt[4] += w * self.dt
        else:
            self.x_opt[0] += v * self.dt * math.cos(self.x_opt[4])
            self.x_opt[1] += v * self.dt * math.sin(self.x_opt[4])
        
        self.x_opt[2] = control[0]
        self.x_opt[3] = control[1]
        self.P_opt = self.predict_covariance()

    def update_state_by_nn_model(self, model):
        """
        c : control vector of RobotControl type
        dt : time period in seconds
        """

        model_input = np.array([[self.v, self.w, self.control[0], self.control[1], self.dt]], 
                                dtype=np.float32)
        model_output = model(model_input)
        self.v, self.w = float(model_output[0][0]), float(model_output[0][1])
        vel_vector = np.array([self.v, self.w])
        self.update_state_by_model(vel_vector)
        return self.x_opt, self.P_opt

    def set_control(self, msg):
        self.control[0] = msg.linear.x
        self.control[1] = msg.angular.z

    def set_odometry(self, z):
        self.z_odom = z
    
    def set_imu(self, z):
        self.z_imu = z
    
    def predict_covariance(self):
        # Calculate covariance using model Jacobian, Q matrix and P_opt_prev
        P_predict = np.zeros((6, 6))
        J_f = transform_jacobian(self.x_opt, self.dt)
        P_predict = J_f @ self.P_opt @ J_f.T + self.Q
        return P_predict

    def update(self):
        self.update_odom()
        self.update_imu()
        return self.x_opt, self.P_opt

    def update_odom(self):
        H = get_jacobian_odom(self.x_opt)   
        y = self.z_odom - H @ self.x_opt
        G = H @ self.P_opt @ H.T + self.R_odom
        K = self.P_opt @ H.T @ inv(G)
        I = np.eye(6)
        self.P_opt = (I - K @ H) @ self.P_opt
        self.x_opt = self.x_opt + K @ y

    def update_imu(self):
        H = get_jacobian_imu(self.x_opt) 
        y = self.z_imu - imu(self.x_opt)
        G = H @ self.P_opt @ H.T + self.R_imu
        K = self.P_opt @ H.T @ inv(G)
        I = np.eye(6)
        self.P_opt = (I - K @ H) @ self.P_opt
        self.x_opt = self.x_opt + K @ y