import numpy as np
from numpy.linalg import inv

import Model2D
import Measurement2D

class Filter2D:
    def __init__(self, P_init, R, Q, dt = 1):
        self.model = Model2D(dt)
        self.measurements = Measurement2D()
        self.P = P_init
        self.R = R
        self.x_predict = np.zeros(8)
        self.P_predict = np.zeros((8, 8))
        self.x_opt = np.zeros(8)
        self.P_opt = np.zeros((8, 8))
        self.Q = Q
        self.P_opt = np.zeros((8, 8))

    def set_odometry(self):
        self.measurements.set_z_odom()
    
    def set_imu(self):
        self.measurements.set_z_imu()

    def predict(self):
        self.x_predict = self.model.get_next_state()
        J = self.model.transform_jacobian()
        self.P_predict = J @ self.P_opt @ J.T + self.Q

    def update_odom(self):
        x = self.model.x_predict(self.x_opt)
        z = self.measurements.get_z_odom()
        H = self.measurements.get_jacobian_odom()   # H is equivalent to odometry Jacobian
        y = z - H @ x
        G = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ inv(G)
        I = np.zeros(8)
        self.P_opt = (I - K @ H) @ self.P
        self.x_opt = x + K @ y
        return self.x_opt, self.P_opt

    def update_imu(self):
        x = self.x_predict
        z = self.measurements.get_z_imu()
        H = self.measurements.get_jacobian_imu()   # H is equivalent to odometry Jacobian
        y = z - H @ x
        G = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ inv(G)
        I = np.zeros(8)
        self.P_opt = (I - K @ H) @ self.P
        self.x_opt = x + K @ y
        return self.x_opt, self.P_opt

    def update_model(self):
        x = self.x_predict
        z = self.measurements.get_z_model()
        H = self.measurements.get_jacobian_model()   # H is equivalent to odometry Jacobian
        y = z - H @ x
        G = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ inv(G)
        I = np.zeros(8)
        self.P_opt = (I - K @ H) @ self.P
        self.x_opt = x + K @ y
        return self.x_opt, self.P_opt