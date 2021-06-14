#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv

from state_estimation_2d.model import *
from state_estimation_2d.measurement import *

class Filter2D:
    def __init__(self, x_init, P_init, R_odom, R_imu, Q, dt = 0.1):
        self.z_odom = np.zeros(6)
        self.z_imu = np.zeros(1)
        self.R_odom = R_odom
        self.R_imu = R_imu
        self.Q = Q
        self.x_opt = x_init
        self.P_opt = P_init
        self.dt = dt

    def set_odometry(self, z):
        self.z_odom = z
    
    def set_imu(self, z):
        self.z_imu = z

    def predict(self):
        x_predict = self.predict_next_state()
        P_predict = self.predict_covariance()
        return x_predict, P_predict
        # self.x_predict = self.model.get_next_state()
        # J = self.model.transform_jacobian()
        # self.P_predict = J @ self.P_opt @ J.T + self.Q

    def predict_next_state(self):
        # Get next state from model predictions using previous optimal state
        #Take previous optimal state
        pos = self.x_opt[0:2]
        vel = self.x_opt[2:4]
        acc = self.x_opt[4:6]
        angle = self.x_opt[6]
        w = self.x_opt[7]
        #Calculate next state using model equations
        next_pos = pos + vel * self.dt + 0.5 * acc * self.dt * self.dt
        next_vel = vel + self.dt * acc
        next_acc = acc
        next_angle = angle + w * self.dt
        next_w = w
        #Fill the state vector
        x_predict = np.zeros(8)
        x_predict[0:2] = next_pos
        x_predict[2:4] = next_vel
        x_predict[4:6] = next_acc
        x_predict[6] = next_angle
        x_predict[7] = next_w
        return x_predict
    
    def predict_covariance(self):
        # Calculate covariance using model Jacobian, Q matrix and P_opt_prev
        P_predict = np.zeros((8, 8))
        J_f = model.transform_jacobian(self.dt)
        P_predict = J_f @ self.P_opt @ J_f.T + self.Q
        return P_predict

    def update(self, x_predict, P_predict):
        self.update_odom(x_predict, P_predict)
        # self.update_imu(self.x_opt, self.P_opt)
        return self.x_opt, self.P_opt

    def update_odom(self, x_predict, P_predict):
        J_h_odom = measurement.get_jacobian_odom()   
        # In our measurement model H is equivalent to odometry Jacobian
        H = J_h_odom    
        y = self.z_odom - H @ x_predict
        G = J_h_odom @ P_predict @ J_h_odom.T + self.R_odom
        K = P_predict @ J_h_odom.T @ inv(G)
        I = np.zeros(8)
        self.P_opt = (I - K @ J_h_odom) @ P_predict
        self.x_opt = x_predict + K @ y

    def update_imu(self, x_predict, P_predict):
        # z = self.measurements.get_z_imu()
        J_h_imu = measurement.get_jacobian_imu()   # H is equivalent to odometry Jacobian
        H = J_h_imu
        y = self.z_imu - H @ x_predict
        G = J_h_imu @ P_predict @ J_h_imu.T + self.R_imu
        K = P_predict @ J_h_imu.T @ inv(G)
        I = np.zeros(8)
        self.P_opt = (I - K @ J_h_imu) @ P_predict
        self.x_opt = x_predict + K @ y