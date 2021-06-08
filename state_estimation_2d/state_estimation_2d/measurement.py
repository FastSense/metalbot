import numpy as np

class Measurement2D:
    def __init__(self):
        self.z_odom = np.zeros(3)
        self.z_model = np.zeros(3)
        self.z_imu = 0
        self.J_odom = np.zeros((3, 8))
        self.J_model = np.zeros((3, 8))
        self.J_imu = np.zeros((1, 8))
    
    def set_z_odom(self, z):
        self.z_odom = z

    def set_z_model(self, z):
        self.z_model = z
    
    def set_z_imu(self, z):
        self.z_imu = z
    
    def get_jacobian_odom(self):
        self.J_odom[0, 2] = 1
        self.J_odom[1, 3] = 1
        self.J_odom[2, 7] = 1
        return self.J_odom
    
    def get_jacobian_model(self):
        self.J_model[0, 2] = 1
        self.J_model[1, 3] = 1
        self.J_model[2, 7] = 1
        return self.J_model
    
    def get_jacobian_imu(self):
        self.J_imu[0, 7] = 1
        return self.J_imu