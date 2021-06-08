import numpy as np


class Model2D:
    def __init__(self, dt = 1):
        self.x = np.zeros(8)
        self.dt = dt
        self.J_f = np.eye(8)
        
    
    def set_state_vector(self, x):
        self.x = x

    def get_next_state(self):
        pos = self.x[0:2]
        vel = self.x[2:4]
        acc = self.x[4:6]
        angle = self.x[6]
        w = self.x[7]

        next_pos = pos + vel * self.dt + 0.5 * acc * self.dt * self.dt
        next_vel = vel + self.dt * acc
        next_acc = acc
        next_angle = angle + w * self.dt
        next_w = w

        next_state = np.zeros(8)
        next_state[0:2] = next_pos
        next_state[2:4] = next_vel
        next_state[4:6] = next_acc
        next_state[6] = next_angle
        next_state[7] = next_w

        return next_state
    
    def transform_jacobian(self):
        self.J_f[0, 2] = self.dt
        self.J_f[1, 3] = self.dt
        self.J_f[2, 4] = self.dt
        self.J_f[3, 5] = self.dt
        self.J_f[6, 7] = self.dt
        self.J_f[0, 4] = 0.5 * self.dt * self.dt
        self.J_f[1, 5] = 0.5 * self.dt * self.dt
        return self.J_f