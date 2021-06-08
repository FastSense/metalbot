import numpy as np


class Model2D:
    def __init__(self, Q, P_init, dt = 1):
        self.x_predict = np.zeros(8)
        self.Q = Q
        self.P_predict = P_init
        self.dt = dt
        self.J_f = np.eye(8)
        self.transform_jacobian()
    
    def set_state_vector(self, x):
        self.x = x

    def predict(self, x):
        self.predict_state(x)
        self.predict_covariance()
        return self.x_predict, self.P_predict

    def predict_state(self, x):
        #Take previous optimal state
        pos = x[0:2]
        vel = x[2:4]
        acc = x[4:6]
        angle = x[6]
        w = x[7]
        #Calculate next state using model equations
        next_pos = pos + vel * self.dt + 0.5 * acc * self.dt * self.dt
        next_vel = vel + self.dt * acc
        next_acc = acc
        next_angle = angle + w * self.dt
        next_w = w
        #Fill the state vector
        self.x_predict[0:2] = next_pos
        self.x_predict[2:4] = next_vel
        self.x_predict[4:6] = next_acc
        self.x_predict[6] = next_angle
        self.x_predict[7] = next_w
    
    def predict_covariance(self, P):
        self.P_predict = self.J_f @ P @ self.J_f.T + self.Q
    
    def transform_jacobian(self):
        self.J_f[0, 2] = self.dt
        self.J_f[1, 3] = self.dt
        self.J_f[2, 4] = self.dt
        self.J_f[3, 5] = self.dt
        self.J_f[6, 7] = self.dt
        self.J_f[0, 4] = 0.5 * self.dt * self.dt
        self.J_f[1, 5] = 0.5 * self.dt * self.dt