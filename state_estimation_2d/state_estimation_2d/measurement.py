import numpy as np

"""
Measurement functions
"""

def get_jacobian_odom(x):
    """Odometry measurement jacobian"""
    H = np.zeros((3, 6))
    H[0, 2] = 1 
    H[1, 3] = 1
    H[2, 5] = 1  
    return H

def get_jacobian_imu(x):
    """Imu measurement jacobian"""
    H = np.zeros((2, 6))
    H[0, 2] = x[5]
    H[0, 5] = x[2]
    H[1, 5] = 1
    return H

def imu(x):
    """Imu measurement function"""
    z = np.zeros(2)
    z[0] = x[2] * x[5]
    z[1] = x[5]
    return z