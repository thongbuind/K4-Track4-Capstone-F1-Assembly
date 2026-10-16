"""Detector: OracleDetector (từ segmentation mask) và chỗ cắm YoloDetector."""
import math
from typing import Callable, Protocol

import numpy as np

from . import config as C
from .shared_types import Detection


class Detector(Protocol):
    def detect(self, rgb: np.ndarray, seg: np.ndarray | None = None) -> list[Detection]: ...


def obb_from_points(pts):
    """OBB của tập điểm (N, 2) bằng PCA. Trả (cx, cy, w, h, angle, e1, e2, proj1)."""
    mean = pts.mean(axis=0)
    cov = np.cov((pts - mean).T)
    vals, vecs = np.linalg.eigh(cov)
    e1 = vecs[:, np.argmax(vals)]          # trục dài
    e2 = np.array([-e1[1], e1[0]])
    q1 = (pts - mean) @ e1
    q2 = (pts - mean) @ e2
    c = mean + e1 * (q1.min() + q1.max()) / 2 + e2 * (q2.min() + q2.max()) / 2
    w = q1.max() - q1.min() + 1.0
    h = q2.max() - q2.min() + 1.0
    angle = math.atan2(e1[1], e1[0])
    return c[0], c[1], w, h, angle, e1, e2, (pts - c) @ e1


def nose_heading(proj1, end_frac=0.2, min_contrast=0.15):
    """Đầu nào của thân xe là mũi, từ phân bố pixel dọc trục dài.

    Mũi hẹp hơn đuôi → ít pixel hơn ở 20% đầu đó. Trả (+1 nếu mũi ở phía +e1, -1 nếu phía -e1,
    heading_ok). Chênh lệch < 15% thì không chắc.
    """
    lo, hi = proj1.min(), proj1.max()
    span = hi - lo
    n_pos = np.count_nonzero(proj1 > hi - end_frac * span)
    n_neg = np.count_nonzero(proj1 < lo + end_frac * span)
    big = max(n_pos, n_neg, 1)
    ok = abs(n_pos - n_neg) / big >= min_contrast
    return (1 if n_pos < n_neg else -1), ok


class OracleDetector:
    """Đọc segmentation mask (không đọc pose) → OBB + nhiễu, giả lập 1 model tốt.

    Nhãn lớp lấy từ bảng body id → lớp (giống nhãn dataset). Bộ phận bị che quá nhiều thì bỏ.
    """

    def __init__(self, class_of: Callable[[int], str | None], camera, rng=None,
                 center_noise_px=2.0, angle_noise_deg=3.0, min_pixels=30, max_occlusion=0.6):
        self.class_of = class_of
        self.cam = camera
        self.rng = rng or np.random.default_rng(0)
        self.center_noise = center_noise_px
        self.angle_noise = math.radians(angle_noise_deg)
        self.min_pixels = min_pixels
        self.max_occlusion = max_occlusion

    def _expected_pixels(self, cls):
        """Diện tích nhìn từ trên (pixel) nếu không bị che."""
        area = 0.0
        for kind, dims, _ in C.PARTS[cls]["shapes"]:
            area += 4 * dims[0] * dims[1] if kind == "box" else math.pi * dims[0] ** 2
        dist = self.cam.eye[2] - C.PARTS[cls]["height"]
        return area * (self.cam.fx / dist) ** 2

    def detect(self, rgb, seg=None):
        dets = []
        ids = np.unique(seg)
        for bid in ids:
            cls = self.class_of(int(bid)) if bid >= 0 else None
            if cls is None:
                continue
            vs, us = np.nonzero(seg == bid)
            if len(us) < self.min_pixels:
                continue
            if 1 - len(us) / self._expected_pixels(cls) > self.max_occlusion:
                continue
            pts = np.stack([us + 0.5, vs + 0.5], axis=1).astype(float)
            cx, cy, w, h, angle, _, _, proj1 = obb_from_points(pts)
            heading_ok = True
            if cls == "body":
                sign, heading_ok = nose_heading(proj1)
                if sign < 0:
                    angle += math.pi           # angle luôn chỉ về phía mũi xe
            cx += self.rng.normal(0, self.center_noise)
            cy += self.rng.normal(0, self.center_noise)
            angle += self.rng.normal(0, self.angle_noise)
            dets.append(Detection(cls, 0.99, cx, cy, w, h, angle, heading_ok))
        return dets


class YoloDetector:
    """Chỗ cắm model thật (ultralytics YOLO-OBB). Chưa dùng: cần best.pt sau khi train."""

    CLASSES = ["body", "front_wing", "rear_wing", "wheel"]

    def __init__(self, weights, conf=0.4):
        from ultralytics import YOLO
        self.model = YOLO(weights)
        self.conf = conf

    def detect(self, rgb, seg=None):
        r = self.model.predict(rgb[..., ::-1], imgsz=640, conf=self.conf, verbose=False)[0]
        dets = []
        for (cx, cy, w, h, ang), k, cf in zip(r.obb.xywhr.tolist(), r.obb.cls.tolist(),
                                              r.obb.conf.tolist()):
            if w < h:                          # chuẩn hoá: w là trục dài
                w, h, ang = h, w, ang + math.pi / 2
            dets.append(Detection(self.CLASSES[int(k)], cf, cx, cy, w, h, ang))
        # TODO: hướng mũi xe từ mask màu đỏ trong OBB (giống nose_heading)
        return dets
