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
    1. y
    2. z
    3. x_vel
    4. y_vel
    5. z_vel
    6. e_x
    7. e_y
    8. e_z
    9. w_x
    10. w_y
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

    ACCEL_STD = 5.0

    def __init__(
        self,
        dt,
        velocity_std,
        rot_vel_std,
    ):
        self.x = np.zeros(12)
        self.P = np.eye(12)
        self.Q = np.zeros([12, 12])
        self.Q[3, 3] = dt * velocity_std**2
        self.Q[4, 4] = dt * velocity_std**2
        self.Q[5, 5] = dt * velocity_std**2
        self.Q[9, 9] = dt * rot_vel_std**2
        self.Q[10, 10] = dt * rot_vel_std**2
        self.Q[11, 11] = dt * rot_vel_std**2
        self.q = np.array([0., 0, 0, 1])
        self.dt = dt

    def predict(self, dt=None):
        # Reset manifold before each predict
        self.reset_manifold()
        # Prediction step
        dt = dt or self.dt
        F = physics.transition_jac(self.rot_vel, dt)
        self.pos, self.vel, self._epsilon, self.rot_vel, self.q = physics.transition_function(
            self.pos, self.vel, self._epsilon, self.rot_vel, self.q, dt
        )
        self.P = F @ (self.P + self.Q) @ F.T

    def update_linear(self, H, z, R):
        y = z - H @ self.x
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_acc(self, z, R, gravity=None, extrinsic=None):
        '''
        Update state by a measurement coming from on-board accelerometer.
        '''
        R_new = R + np.eye(3) * self.ACCEL_STD**2
        self.update_static_vec(z, R_new, vec=gravity or GRAVITY, extrinsic=extrinsic)

    def update_rot_vel(self, z, R, extrinsic=None):
        '''
        Update state by a measurement coming from on-board gyroscope.
        '''
        z_prior, H = measurent.rot_vel_local(self.rot_vel, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_static_vec(self, z, R, vec, extrinsic=None):
        '''
        Update state by a measurement of some vector which is known in the world coordinates.
        For example: magnetic field, gravity vector.
        '''
        z_prior, H = measurent.static_vec(self.q, vec, extrinsic=extrinsic)
        y = z - z_prior
        self.x, self.P = kalman_update(self.x, self.P, H, R, y)

    def update_flow(self, flows, delta_t, depths, pixels, R, camera_matrix, camera_matrix_inv, extrinsic=None, delay=0):
        '''
        Update state by a visual odometry measurement coming from a neural network.

        Parameters
        ----------
            flows (np.array of shape [N, 3]): Optical flow measurements at N given points
            delta_t (float): time step
            depths (np.array of shape [N]): Depth at N given points
            pixels (np.array of shape [N, 2]): x and y coordinates of image points, in pixel scale
            R (np.array of shape [3, 3]): error of measurements
            camera_matrix (np.array of shape [3, 3]): camera intrinsic matrix
            extrinsic (None or np.array of shape [3, 4]): optional. Camera pose relative to filter
            delay (float): seconds passed between measurement being taken and it being processed
        '''
        z = flows.flatten()
        z_prior, H = measurent.flow_odom12(self.vel, self.rot_vel, self.q, delta_t, depths, pixels, camera_matrix, camera_matrix_inv, extrinsic)
        y = z - z_prior
        F_noise = physics.transition_jac(self.rot_vel, -(delta_t + delay))
        dt_coef = (delta_t + delay) / self.dt
        noise = F_noise @ (self.Q * dt_coef) @ F_noise.T
        self.x, self.P = kalman_update(self.x, self.P, H, R, y, noise)

    def reset_manifold(self):
        '''
        Reset the linearization center

        Called automatically before every predict step.
        '''
        self._epsilon, self.q = geometry.reset_manifold(self._epsilon, self.q)

    @property
    def pos(self):
        return self.x[:3]
    @pos.setter
    def pos(self, value):
        self.x[:3] = value

    @property
    def vel(self):
        return self.x[3:6]
    @vel.setter
    def vel(self, value):
        self.x[3:6] = value

    @property
    def _epsilon(self):
        return self.x[6:9]
    @_epsilon.setter
    def _epsilon(self, value):
        self.x[6:9] = value

    @property
    def rot_vel(self):
        return self.x[9:]
    @rot_vel.setter
    def rot_vel(self, value):
        self.x[9:] = value

    @property
    def euler(self):
        return Rotation.from_quat(self.q).as_euler('xyz', degrees=False)

    @property
    def rot_matrix(self):
        return geometry.quat_as_matrix(self.q)

    def get_pose_covariance(self):
        '''
        Returns 6x6 covariance matrix of pose in the form a list with length 36:
        (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        '''
        indeces = [0, 1, 2, 6, 7, 8]
        mat = self.P[indeces][:, indeces]
        return list(mat.flatten())

    def get_twist_covariance(self):
        '''
        Returns 6x6 covariance matrix of velocity in the form a list with length 36:
        (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
        '''
        indeces = [3, 4, 5, 9, 10, 11]
        mat = self.P[indeces][:, indeces]
        return list(mat.flatten())


@njit
def kalman_update(x, P, H, R, y, noise=None):
    if noise is None:
        S = H @ P @ H.T + R
    else:
        S = H @ (P + noise) @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    new_x = x + K @ y
    new_P = (np.eye(x.shape[0]) - K @ H) @ P
    return new_x, new_P
