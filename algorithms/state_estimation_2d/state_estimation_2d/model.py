import numpy as np

"""Model dynamics functions"""

def transform_jacobian(x, dt):
    """Dynamic model function jacobian"""
    J_f = np.eye(5)
    J_f[0, 2] = np.cos(x[3]) * dt
    J_f[0, 3] = -x[2] * np.sin(x[3]) * dt
    J_f[1, 2] = np.sin(x[3]) * dt
    J_f[1, 3] = x[2] * np.cos(x[3]) * dt
    J_f[3, 4] = dt
    return J_f