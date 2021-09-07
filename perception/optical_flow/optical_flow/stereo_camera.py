import numpy as np
import time


class StereoCamera:
    def __init__(
        self, M1, M2, R, T, image_h, image_w, d1=None, d2=None, **kwargs
    ):
        self.M1 = np.array(M1)
        self.M2 = np.array(M2)
        self.R = np.array(R)
        self.T = np.array(T)
        self.d1 = np.array(d1)
        self.d2 = np.array(d2)

        self.M1_inv = np.linalg.inv(M1)
        self.M2_inv = np.linalg.inv(M2)

        self.image_h = image_h
        self.image_w = image_w

        # Make pixel map
        # pixel_map: np.array with shape [H, W, 2], where first channel is x coordinate of pixel, and second is y
        self.pixel_map = self._compute_pixmap()

        # Make hyperbolic map
        # hyper_map: np.array with shape [H, W, 2] of x,y hyperbolic coordinates of each pixel
        self.hyper_map_1 = self.pix2hyper(self.pixel_map, 1)
        self.hyper_map_2 = self.pix2hyper(self.pixel_map, 2)

    def change_dimensions_(self, new_image_h, new_image_w):
        '''
        Recomputes camera parameters for new dimensions
        '''
        M = np.array([
            [new_image_w / self.image_w, 0, 0.5 * (new_image_w / self.image_w - 1)],
            [0, new_image_h / self.image_h, 0.5 * (new_image_h / self.image_h - 1)],
            [0, 0, 1]
        ])

        self.M1 = M @ self.M1
        self.M2 = M @ self.M2

        self.M1_inv = np.linalg.inv(self.M1)
        self.M2_inv = np.linalg.inv(self.M2)

        self.image_h = new_image_h
        self.image_w = new_image_w

        self.pixel_map = self._compute_pixmap()
        self.hyper_map_1 = self.pix2hyper(self.pixel_map, 1)
        self.hyper_map_2 = self.pix2hyper(self.pixel_map, 2)

    def _compute_pixmap(self):
        # Make pixel map
        # pixel_map: np.array with shape [H, W, 2], where first channel is x coordinate of pixel, and second is y
        px = np.arange(self.image_w)
        py = np.arange(self.image_h)
        px, py = np.meshgrid(px, py) # [image_h, image_w]
        pixel_map = np.concatenate([px[:, :, None], py[:, :, None]], 2)

        return pixel_map

    def hyper2pix(self, m):
        '''
        In linear cameras this is multiplication by camera matrix.

        Parameters:
        m: hyperbolic coordinates. Shape: [..., 2]
        Returns:
        p: image coordinates. Shape: [..., 2]
        '''
        ...

    def pix2hyper(self, p, cam_id):
        '''
        In linear cameras this is multiplication by inverted camera matrix.

        Parameters:
        p: image coordinates. Shape: [..., 2]
        Returns:
        m: hyperbolic coordinates. Shape: [..., 2]
        '''
        # Reshape and remember shape
        shape = p.shape
        assert shape[-1] == 2
        p = p.reshape(-1, 2).T # [2, ?]
        p = np.vstack([p, np.ones([1, p.shape[1]])])
        # Multiply by camera matrix
        if cam_id == 1:
            m = self.M1_inv[:2] @ p
        elif cam_id == 2:
            m = self.M2_inv[:2] @ p
        # Reshape
        m = m.T.reshape(shape)
        return m

    def hyper2pix_jac(self):
        '''
        In linear cameras this is camera matrix.
        '''
        ...

    def depth_from_flow(self, flow):
        # Compute hyperbolic maps
        m_t = self.hyper_map_1.reshape(-1, 2) # [N, 2]
        m_s = self.pix2hyper(self.pixel_map + flow, 2).reshape(-1, 2) # [N, 2]
        N = m_t.shape[0]

        # Add identity row
        m_t = np.concatenate([m_t, np.ones([N, 1])], 1) # [N, 3]
        m_s = np.concatenate([m_s, np.ones([N, 1])], 1) # [N, 3]

        # Rotate hyperbolic coordinates
        m_t_rot = (self.R @ m_t.T).T

        # Make linear equation parameters
        A = np.zeros([N, 3, 2])
        A[:, :, 0] = m_t_rot
        A[:, :, 1] = m_s
        B = -self.T[None].repeat(N, 0) # [N, 3]

        # Solve equation using least squares
        ATA = np.einsum('nki, nkj -> nij', A, A) # [N, 2, 2]
        ATB = np.einsum('nkj, nk -> nj', A, B) # [N, 2]
        X = np.linalg.solve(ATA, ATB) # [N, 2]

        # Get depth
        depth = X[:, 0] # z_t
        depth = depth.reshape([self.image_h, self.image_w])

        # Compute error
        AX = np.einsum('nik, nk -> ni', A, X) # [N, 3]
        err = ((AX - B)**2).sum(1)
        err = err.reshape([self.image_h, self.image_w])

        return depth, err
