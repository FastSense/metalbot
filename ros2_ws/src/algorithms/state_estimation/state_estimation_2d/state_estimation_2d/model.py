import numpy as np


def transform_jacobian(x, dt):
    """Dynamic model function jacobian
    State vector x:
    0. x
    1. y
    2. V_parallel       (velocity along the robot direction)
    3. yaw
    4. yaw_vel
    """
    J_f = np.eye(5)
    J_f[0, 2] = np.cos(x[3]) * dt
    J_f[0, 3] = -x[2] * np.sin(x[3]) * dt
    J_f[1, 2] = np.sin(x[3]) * dt
    J_f[1, 3] = x[2] * np.cos(x[3]) * dt
    J_f[3, 4] = dt
    return J_f