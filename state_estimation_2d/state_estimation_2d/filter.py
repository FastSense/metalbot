#!/usr/bin/env python
import numpy as np
from numpy.linalg import inv
import math

from state_estimation_2d.model import *
from state_estimation_2d.measurement import *

class RobotState():

    def __init__(self, x=0.0, y=0.0, yaw=0.0, vx=0.0, vy=0.0, w=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw

    def to_str(self):
        return "x -> {:.2f}, y -> {:.2f}, yaw -> {:.2f}".format(self.x, self.y, self.yaw)

class Filter2D:
    def __init__(self, x_init, P_init, R_odom, R_imu, Q, dt = 0.1):
        self.z_odom = np.zeros(3)
        self.z_imu = np.zeros(1)
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

    def update_state_by_nn_model(self, model):
        """
        c : control vector of RobotControl type
        dt : time period in seconds
        """

        model_input = np.array([[self.v, self.w, self.control[0], self.control[1], self.dt]], dtype=np.float32)
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

    def predict(self):
        x_predict = self.predict_next_state()
        P_predict = self.predict_covariance()
        self.x_opt = x_predict
        self.P_opt = P_predict
        return x_predict, P_predict
        # self.x_predict = self.model.get_next_state()
        # J = self.model.transform_jacobian()
        # self.P_predict = J @ self.P_opt @ J.T + self.Q

    def predict_next_state(self):
        # Get next state from model predictions using previous optimal state
        #Take previous optimal state
        pos = self.x_opt[0:2]
        vel = self.x_opt[2:4]
        #acc = self.x_opt[4:6]
        angle = self.x_opt[4]
        w = self.x_opt[5]
        #Calculate next state using model equations
        next_x = pos[0] + vel[0] * np.cos(angle) * self.dt
        next_y = pos[1] + vel[0] * np.sin(angle) * self.dt
        # next_pos = pos + vel * self.dt #+ 0.5 * acc * self.dt * self.dt
        next_vel = vel #+ self.dt * acc
        #next_acc = acc
        next_angle = angle + w * self.dt
        next_w = w
        #Fill the state vector
        x_predict = np.zeros(6)
        x_predict[0] = next_x
        x_predict[1] = next_y
        # x_predict[0:2] = next_pos
        x_predict[2:4] = next_vel
        # x_predict[4:6] = next_acc
        x_predict[4] = next_angle
        x_predict[5] = next_w
        return x_predict
    
    def predict_covariance(self):
        # Calculate covariance using model Jacobian, Q matrix and P_opt_prev
        P_predict = np.zeros((6, 6))
        J_f = transform_jacobian(self.x_opt, self.dt)
        P_predict = J_f @ self.P_opt @ J_f.T + self.Q
        return P_predict

    def update(self, x_predict, P_predict):
        self.update_odom(x_predict, P_predict)
        # self.update_imu(self.x_opt, self.P_opt)
        return self.x_opt, self.P_opt

    def update_odom(self, x_predict, P_predict):
        J_h_odom = get_jacobian_odom(self.x_opt)   
        # In our measurement model H is equivalent to odometry Jacobian
        H = J_h_odom    
        y = self.z_odom - H @ self.x_opt
        G = J_h_odom @ self.P_opt @ J_h_odom.T + self.R_odom
        K = self.P_opt @ J_h_odom.T @ inv(G)
        I = np.eye(6)
        self.P_opt = (I - K @ J_h_odom) @ self.P_opt
        self.x_opt = self.x_opt + K @ y
        # odom = np.array([self.z_odom[0], self.z_odom[1], self.z_odom[2], self.z_odom[3], 100000, 100000, self.z_odom[4], self.z_odom[5]])
        # print(odom - self.x_opt)

    def update_imu(self, x_predict, P_predict):
        # z = self.measurements.get_z_imu()
        J_h_imu = get_jacobian_imu()   # H is equivalent to odometry Jacobian
        H = J_h_imu
        y = self.z_imu - H @ self.x_opt
        G = J_h_imu @ self.P_opt @ J_h_imu.T + self.R_imu
        K = self.P_opt @ J_h_imu.T @ inv(G)
        I = np.eye(6)
        self.P_opt = (I - K @ J_h_imu) @ self.P_opt
        self.x_opt = self.x_opt + K @ y