"""Camera nhìn thẳng xuống: render ảnh thật + chiếu ngược pixel → mặt phẳng ngang."""
import math

import numpy as np
import pybullet as p

from . import config as C


class Camera:
    def __init__(self, eye, up=(0.0, 1.0, 0.0), width=C.CAM_W, height=C.CAM_H, fov=C.CAM_FOV,
                 noise_sigma=0.0, rng=None):
        self.eye = tuple(eye)
        target = (eye[0], eye[1], 0.0)
        self.w, self.h = width, height
        # upVector = +y: trục ngang của ảnh nằm dọc băng chuyền (+x)
        self.view = p.computeViewMatrix(self.eye, target, up)
        self.proj = p.computeProjectionMatrixFOV(fov=fov, aspect=width / height,
                                                 nearVal=0.1, farVal=2.0)
        # nội tham số: FOV của PyBullet là FOV dọc, pixel vuông
        self.fy = (height / 2) / math.tan(math.radians(fov) / 2)
        self.fx = self.fy
        self.cx, self.cy = width / 2, height / 2
        # ngoại tham số: view matrix xếp theo cột, world -> camera (quy ước OpenGL, nhìn theo -z)
        V = np.array(self.view, dtype=float).reshape(4, 4).T
        T_c2w = np.linalg.inv(V)
        self.R, self.o = T_c2w[:3, :3], T_c2w[:3, 3]
        self.noise_sigma = noise_sigma
        self.rng = rng or np.random.default_rng(0)
        self.last = None

    def capture(self, t):
        """Chụp 1 ảnh. Trả (rgb uint8 HxWx3, seg int HxW, t_capture)."""
        _, _, rgba, _, seg = p.getCameraImage(self.w, self.h, self.view, self.proj,
                                              renderer=p.ER_TINY_RENDERER)
        rgb = np.reshape(np.asarray(rgba, dtype=np.uint8), (self.h, self.w, 4))[:, :, :3]
        if self.noise_sigma > 0:
            noisy = rgb.astype(np.float32) + self.rng.normal(0, self.noise_sigma, rgb.shape)
            rgb = np.clip(noisy, 0, 255).astype(np.uint8)
        seg = np.reshape(np.asarray(seg), (self.h, self.w))
        self.last = (rgb, seg, t)
        return rgb, seg, t

    def pixel_to_plane(self, u, v, z_plane):
        """Pixel (u, v) — toạ độ liên tục, (0, 0) là góc trên trái ảnh — → (x, y) trên mặt z = z_plane."""
        d_cam = np.array([(u - self.cx) / self.fx, -(v - self.cy) / self.fy, -1.0])
        d = self.R @ d_cam
        s = (z_plane - self.o[2]) / d[2]
        pw = self.o + s * d
        return float(pw[0]), float(pw[1])

    def world_to_pixel(self, x, y, z):
        """Chiếu thuận (để vẽ overlay và kiểm tra)."""
        pc = self.R.T @ (np.array([x, y, z]) - self.o)
        return self.cx + self.fx * pc[0] / -pc[2], self.cy - self.fy * pc[1] / -pc[2]
