import numpy as np
import scipy
from numba import njit
from scipy.spatial.transform import Rotation
from filterpy.common import Q_discrete_white_noise

from . import physics, geometry, measurent


GRAVITY = np.array([0, 0, 9.8])


class SpaceKF12:
    '''
    12-dimensional Extended Kalman Filter

    State of this filter is called `x` and consists of:
    0. x
    1. x_vel
    2. y
    3. y_vel
    4. z
    5. z_vel
    6. e_x
    7. w_x
    8. e_y
    9. w_y
    10. e_z
    11. w_z

    Here e_xyz is the Euclidian representation of the delta from the center quaternion.
    w_xyz is the rotation speed.
    Additionaly, this filter stores `q` - the center quaternion around which the neighborhood e is built.
    More about this here: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6339217/
    Normally, `e_xyz==0` after the predict step. This means that `q` gives us the current attitude.

    You can get current state either directly from `x` and `q` attributes or by using these properties:
    `pos`, `vel`, `rot_vel`.
    You can get yaw, pitch, roll by using property `euler`.
    You can get rotation matrix by using property `rot_matrix`.
    '''
    def __init__(
        self,
        dt,
        velocity_std,
        rot_vel_std,
    ):
        self.x = np.zeros(12)
        self.P = np.eye(12)
        # Idk why there are minuses , but it works:
        Q_e_w = np.array([
            [0.333 * dt**3, -0.5 * dt**2],
            [ -0.5 * dt**2,           dt],
        ]) * rot_vel_std**2
        self.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(dim=2, dt=dt, var=velocity_std**2, block_size=2),
            Q_e_w, Q_e_w, Q_e_w,
        )
        self.q = np.array([1., 0, 0, 0])
        self.dt = dt

    def predict(self, dt=None):
        # Reset manifold before each predict
        self.reset_manifold()
        # Prediction step
        dt = dt or self.dt
        F = physics.transition_jac(self.x, self.q, dt)
        self.x, self.q = physics.transition_function(self.x, self.q, dt)
        self.P = F @ self.P @ F.T + self.Q

    def update_linear(self, H, z, R):
        y = z - H @ self.x
        self.x, self.P = update(self.x, self.P, H, R, y)

    def update_acc(self, z, R, gravity=None, extrinsic=None):
        '''
        Update state by a measurement coming from on-board accelerometer.
        '''
        z_prior = measurent.static_vec(self.q, vec=gravity or GRAVITY)
        H = measurent.static_vec_jac(self.q, vec=gravity or GRAVITY)
        y = z - z_prior
        q_mat = geometry.quat_as_matrix(self.q)
        R_glob = q_mat.T @ R @ q_mat
        self.x, self.P = update(self.x, self.P, H, R_glob, y)

    def update_rot_vel(self, z, R, extrinsic=None):
        '''
        Update state by a measurement coming from on-board gyroscope.
        '''
        z_prior, H = measurent.rot_vel_local(self.x, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = update(self.x, self.P, H, R, y)

    def update_static_vec(self, z, R, vec):
        '''
        Update state by a measurement of some vector which should be constant in the world coordinates.
        For example, magnetic field.
        '''
        y = z - measurent.static_vec(self.q, vec)
        H = measurent.static_vec_jac(self.q, vec)
        self.x, self.P = update(self.x, self.P, H, R, y)

    def update_odometry(self, z, R, dt_btw_frames, dt_since_last_frame, extrinsic=None):
        '''
        Update state by a visual odometry measurement coming from a neural network.

        Parameters
        ----------
            z (np.array of shape [6]): Output of a neural network. Contains rotations and translations: `[rot_x, rot_y, rot_z, dx, dy, dz]`.
        '''
        H = measurent.odometry_jac(self.x, self.q, dt_btw_frames, dt_since_last_frame, extrinsic=extrinsic)
        self.update_linear(H, z, R)

    def reset_manifold(self):
        '''
        Reset the linearization center

        Don't worry about this function - it is called automatically before every predict step.
        '''
        self.x, self.q = geometry.reset_manifold(self.x, self.q)

    @property
    def pos(self):
        return self.x[0:5:2]
    @pos.setter
    def pos(self, value):
        self.x[0:5:2] = value

    @property
    def vel(self):
        return self.x[1:6:2]
    @vel.setter
    def vel(self, value):
        self.x[1:6:2] = value

    @property
    def rot_vel(self):
        return self.x[7::2]
    @rot_vel.setter
    def rot_vel(self, value):
        self.x[7::2] = value

    @property
    def euler(self):
        return Rotation.from_quat(self.q[[1,2,3,0]]).as_euler('xyz', degrees=False)

    @property
    def rot_matrix(self):
        return geometry.quat_as_matrix(self.q)


@njit
def update(x, P, H, R, y):
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    new_x = x + K @ y
    new_P = (np.eye(x.shape[0]) - K @ H) @ P
    return new_x, new_P



class SpaceKF15(SpaceKF12):
    '''
    15-dimensional Extended Kalman Filter
    State of this filter is called `x` and consists of:
    0. x
    1. x_vel
    2. x_acc
    3. y
    4. y_vel
    5. y_acc
    6. z
    7. z_vel
    8. z_acc
    9. e_x
    10. w_x
    11. e_y
    12. w_y
    13. e_z
    14. w_z

    Here e_xyz is the Euclidian representation of the delta from the center quaternion.
    w_xyz is the rotation speed.
    Additionaly, this filter stores `q` - the center quaternion around which the neighborhood e is built.
    More about this here: https://www.ncbi.nlm.nih.gov/pmc/articles/PMC6339217/
    Normally, `e_xyz==0` after the predict step. This means that `q` gives us the current attitude.

    You can get current state either directly from `x` and `q` attributes or by using these properties:
    `pos`, `vel`, `acc`, `rot_vel`.
    You can get yaw, pitch, roll by using property `euler`.
    You can get rotation matrix by using property `rot_matrix`.
    '''
    def __init__(
        self,
        dt,
        velocity_std,
        rot_vel_std,
    ):
        self.x = np.zeros(15)
        self.P = np.eye(15)
        # Idk why there are minuses , but it works:
        Q_e_w = np.array([
            [0.333 * dt**3, -0.5 * dt**2],
            [ -0.5 * dt**2,           dt],
        ]) * rot_vel_std**2
        self.Q = scipy.linalg.block_diag(
            Q_discrete_white_noise(dim=3, dt=dt, var=velocity_std**2, block_size=3),
            Q_e_w, Q_e_w, Q_e_w,
        )
        self.q = np.array([1., 0, 0, 0])
        self.dt = dt

    def update_acc(self, z, R, gravity=None, extrinsic=None):
        '''
        Update state by a measurement coming from on-board accelerometer.
        '''
        z_prior, H = measurent.acc_local(self.x, self.q, gravity=gravity or GRAVITY, extrinsic=extrinsic)
        y = z - z_prior
        q_mat = geometry.quat_as_matrix(self.q)
        R_glob = q_mat.T @ R @ q_mat
        self.x, self.P = update(self.x, self.P, H, R_glob, y)

    @property
    def pos(self):
        return self.x[0:7:3]
    @pos.setter
    def pos(self, value):
        self.x[0:7:3] = value

    @property
    def vel(self):
        return self.x[1:8:3]
    @vel.setter
    def vel(self, value):
        self.x[1:8:3] = value

    @property
    def acc(self):
        return self.x[2:9:3]
    @acc.setter
    def acc(self, value):
        self.x[2:9:3] = value

    @property
    def rot_vel(self):
        return self.x[10::2]
    @rot_vel.setter
    def rot_vel(self, value):
        self.x[10::2] = value