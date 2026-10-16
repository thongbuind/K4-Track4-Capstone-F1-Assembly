"""Máy QC: camera riêng chụp xe khi nó tới, đếm bộ phận ở đúng 6 slot.

QC không đọc map lắp ráp: chỉ dùng tracker để biết LÚC NÀO có xe tới và hướng mũi đã chốt;
bộ phận có mặt hay không là do ảnh quyết định. Ground truth (constraint trong sim) chỉ để kiểm chéo.
"""
import math
from dataclasses import dataclass, field

from . import config as C
from .assembly import find_chassis
from .parts import slot_world_pose


@dataclass
class QCResult:
    car_id: int
    t: float
    passed: bool
    missing: list = field(default_factory=list)
    gt_attached: int = 0             # số bộ phận thật sự đang gắn trên xe (kiểm chéo)


class QCStation:
    def __init__(self, camera, detector, tracker, world):
        self.cam, self.det, self.tracker, self.world = camera, detector, tracker, world
        self.checked = set()
        self.results: list[QCResult] = []

    def maybe_run(self, t):
        x_cam = self.cam.eye[0]
        for car in self.tracker.cars.values():
            if car.body.id in self.checked:
                continue
            bx, by, byaw = self.tracker.predict(car.body, t)
            if abs(bx - x_cam) <= 0.03:
                self.checked.add(car.body.id)
                self.results.append(self._inspect(car, t, bx, by, byaw))
                return self.results[-1]
        return None

    def _inspect(self, car, t, px, py, pyaw):
        rgb, seg, t_cap = self.cam.capture(t)
        meas = [(d, self.tracker._measure(d, self.cam,
                                          ref_yaw=pyaw if d.cls == "body" else None))
                for d in self.det.detect(rgb, seg)]
        bodies = [m for d, m in meas if d.cls == "body"]
        gt = self._gt_count(px, py)
        if not bodies:
            return QCResult(car.body.id, t, False, ["không thấy thân xe"], gt)
        bx, by, byaw = min(bodies, key=lambda m: math.hypot(m[0] - px, m[1] - py))
        used, missing = set(), []
        for slot, spec in C.SLOTS.items():
            sx, sy, _ = slot_world_pose(bx, by, byaw, slot)
            best, best_d = None, C.QC_MATCH_TOL
            for i, (d, (x, y, _)) in enumerate(meas):
                if d.cls == spec["cls"] and i not in used:
                    dist = math.hypot(x - sx, y - sy)
                    if dist < best_d:
                        best, best_d = i, dist
            if best is None:
                missing.append(slot)
            else:
                used.add(best)
        return QCResult(car.body.id, t, not missing, missing, gt)

    def _gt_count(self, x, y):
        ch = find_chassis(self.world, x, y)
        return sum(1 for parent in self.world.attached.values() if parent == ch) if ch else 0
