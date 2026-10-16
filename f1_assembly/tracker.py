"""Map bộ phận theo toạ độ đường băng (s, d, ψ) + dự đoán bằng odometry của băng chuyền.

Robot chỉ thấy thế giới qua module này: detection từ camera → track → pose dự đoán theo thời gian.
"""
import bisect
import math

from . import config as C
from .parts import obb_center_offset, slot_world_pose, wrap_pi
from .shared_types import Car, PartTrack, Target
from .world import belt_twist

L1 = C.STRAIGHT_END                      # hết đoạn thẳng
R = C.CURVE_R
L2 = L1 + R * math.pi / 2                # hết khúc cua
CX, CY = C.CURVE_CENTER


def frame(s):
    """Điểm c(s) trên đường tâm băng và hướng φ(s)."""
    if s <= L1:
        return s, 0.0, 0.0
    if s <= L2:
        a = (s - L1) / R
        return CX + R * math.sin(a), CY - R * math.cos(a), a
    return CX + R, CY + (s - L2), math.pi / 2


def to_world(s, d, psi):
    """(s, d, ψ) → (x, y, yaw). d > 0 là lệch sang trái chiều chạy."""
    cx, cy, phi = frame(s)
    return cx - d * math.sin(phi), cy + d * math.cos(phi), phi + psi


def from_world(x, y, yaw):
    """(x, y, yaw) → (s, d, ψ) trên băng chính; None nếu không nằm trên băng chính."""
    if 0.0 <= x <= L1 and abs(y) <= C.BELT_HALF_W:
        return x, y, yaw
    rx, ry = x - CX, y - CY
    r = math.hypot(rx, ry)
    if rx >= 0 and ry <= 0 and C.CURVE_R_IN <= r <= C.CURVE_R_OUT:
        a = math.atan2(rx, -ry)
        return L1 + R * a, R - r, yaw - a
    if C.SURPLUS_X[0] <= x <= C.SURPLUS_X[1] and C.SURPLUS_Y[0] <= y <= C.SURPLUS_Y[1]:
        return L2 + (y - CY), (CX + R) - x, yaw - math.pi / 2
    return None


class BeltOdometry:
    """Tích phân vận tốc encoder → quãng đường băng đã chạy s_belt(t)."""

    def __init__(self, keep_s=180.0):
        self.ts, self.ss = [0.0], [0.0]
        self.v_last = C.V_NOMINAL
        self.keep_s = keep_s

    def step(self, t, v_enc, dt):
        self.v_last = v_enc
        self.ts.append(t)
        self.ss.append(self.ss[-1] + v_enc * dt)
        if self.ts[-1] - self.ts[0] > self.keep_s + 10:     # bỏ lịch sử cũ
            cut = bisect.bisect_left(self.ts, self.ts[-1] - self.keep_s)
            del self.ts[:cut], self.ss[:cut]

    def s_at(self, t):
        i = bisect.bisect_right(self.ts, t)
        if i == 0:
            return self.ss[0]
        if i >= len(self.ts):
            return self.ss[-1] + self.v_last * (t - self.ts[-1])
        t0, t1, s0, s1 = self.ts[i - 1], self.ts[i], self.ss[i - 1], self.ss[i]
        return s0 + (s1 - s0) * (t - t0) / (t1 - t0) if t1 > t0 else s0


class Tracker:
    def __init__(self, camera, odo: BeltOdometry, roi_x=(0.0, 0.70), match_dist=0.03,
                 blend=0.5):
        self.cam = camera
        self.odo = odo
        self.roi_x = roi_x                # chỉ nhận detection trong vùng này (trước tầm với trạm 01)
        self.match_dist = match_dist
        self.blend = blend                # trọng số đo mới khi cập nhật vị trí track
        self.tracks: dict[int, PartTrack] = {}
        self.cars: dict[int, Car] = {}    # id track thân xe -> Car
        self._next_id = 1

    # --- ghi nhận ------------------------------------------------------------
    def _measure(self, det, cam, ref_yaw=None):
        """Detection (pixel) → (x, y, yaw) trên mặt trên của bộ phận.

        ref_yaw: nếu có, chọn trong {yaw, yaw + π} hướng gần ref_yaw nhất TRƯỚC khi bù tâm OBB
        (camera trạm không tự quyết hướng mũi: mask bị tay che làm đoán ngược đầu).
        """
        z_top = C.PARTS[det.cls]["height"]
        x, y = cam.pixel_to_plane(det.cx, det.cy, z_top)
        ux, uy = math.cos(det.angle), math.sin(det.angle)
        x2, y2 = cam.pixel_to_plane(det.cx + 20 * ux, det.cy + 20 * uy, z_top)
        yaw = math.atan2(y2 - y, x2 - x)      # yaw trục dài trong world
        if det.cls in ("front_wing", "rear_wing"):
            yaw -= math.pi / 2                # trục dài của cánh là trục y cục bộ (config.PARTS)
        if ref_yaw is not None and abs(wrap_pi(yaw + math.pi - ref_yaw)) < abs(wrap_pi(yaw - ref_yaw)):
            yaw += math.pi
        # tâm OBB ≠ gốc bộ phận khi hình không đối xứng (thân xe: lệch +3 cm về phía mũi)
        ox, oy = obb_center_offset(det.cls)
        x -= ox * math.cos(yaw) - oy * math.sin(yaw)
        y -= ox * math.sin(yaw) + oy * math.cos(yaw)
        return x, y, yaw

    def ingest(self, dets, t_cap, cam=None, roi_x=None, create=True, blend=None, classes=None,
               keep_heading=False):
        """Đưa detection của 1 ảnh vào map.

        Camera đầu vào: create=True (tạo track mới). Camera trạm: create=False, chỉ sửa pose
        track đã có — chủ yếu để bắt kịp thân xe bị va lệch trước khi lắp.
        """
        cam = cam or self.cam
        roi_x = roi_x or self.roi_x
        blend = self.blend if blend is None else blend
        for det in dets:
            if classes is not None and det.cls not in classes:
                continue
            x, y, yaw = self._measure(det, cam)
            if not (roi_x[0] <= x <= roi_x[1]):
                continue
            if det.cls != "body" and self._on_a_car(x, y, t_cap):
                continue                       # bộ phận đã lắp trên xe: không nhầm là vật rời
            sdp = from_world(x, y, yaw)
            if sdp is None:
                continue
            s, d, psi = sdp
            tr = self._match(det.cls, x, y, t_cap)
            if keep_heading and tr is not None and tr.id in self.cars:
                x, y, yaw = self._measure(det, cam, ref_yaw=self.predict(tr, t_cap)[2])
                sdp = from_world(x, y, yaw)
                if sdp is None:
                    continue
                s, d, psi = sdp
            if tr is None:
                if not create:
                    continue
                tr = PartTrack(self._next_id, det.cls, det.conf, s, d, psi, t_cap)
                self.tracks[tr.id] = tr
                self._next_id += 1
                if det.cls == "body":
                    self.cars[tr.id] = Car(tr, heading_ok=det.heading_ok)
                continue
            if tr.state not in ("on_belt", "claimed"):
                continue                       # đang bị kẹp / đã lắp: tay biết rõ hơn camera
            s_pred = self.s_now(tr, t_cap)
            tr.s0 = s_pred + blend * (s - s_pred)
            tr.d += blend * (d - tr.d)
            car = self.cars.get(tr.id)
            if keep_heading:
                pass                           # hướng đã chốt theo ref_yaw ở trên
            elif car is not None and not det.heading_ok:
                # không chắc đầu mũi từ ảnh này: lấy hướng tương đương gần hướng cũ nhất
                if abs(wrap_pi(psi + math.pi - tr.psi)) < abs(wrap_pi(psi - tr.psi)):
                    psi += math.pi
            elif car is not None:
                car.heading_ok = True
            tr.psi = wrap_pi(tr.psi + blend * wrap_pi(psi - tr.psi))
            tr.t0, tr.conf = t_cap, det.conf

    def _on_a_car(self, x, y, t):
        for car in self.cars.values():
            bx, by, _ = self.predict(car.body, t)
            if math.hypot(bx - x, by - y) < 0.16 and any(v is not None for v in car.slots.values()):
                return True
        return False

    def _match(self, cls, x, y, t):
        best, best_d = None, C.MATCH_DIST.get(cls, self.match_dist)
        for tr in self.tracks.values():
            if tr.cls != cls or tr.state in ("surplus", "missed"):
                continue
            px, py, _ = self.predict(tr, t)
            dist = math.hypot(px - x, py - y)
            if dist < best_d:
                best, best_d = tr, dist
        return best

    # --- dự đoán -------------------------------------------------------------
    def s_now(self, tr, t):
        return tr.s0 + self.odo.s_at(t) - self.odo.s_at(tr.t0)

    def predict(self, tr, t):
        return to_world(self.s_now(tr, t), tr.d, tr.psi)

    def target(self, tr, t):
        """Đích cho bộ điều khiển tay: pose dự đoán + vận tốc băng tại đó."""
        x, y, yaw = self.predict(tr, t)
        tw = belt_twist(x, y, self.odo.v_last) or (0.0, 0.0, 0.0)
        return Target(x, y, yaw, tw[0], tw[1], z=C.PARTS[tr.cls]["height"] / 2)

    def slot_target(self, car: Car, slot, t):
        """Đích đặt bộ phận vào slot của 1 xe đang chạy."""
        bx, by, byaw = self.predict(car.body, t)
        x, y, yaw = slot_world_pose(bx, by, byaw, slot)
        tw = belt_twist(x, y, self.odo.v_last) or (0.0, 0.0, 0.0)
        z = C.PARTS["body"]["height"] / 2 + C.SLOTS[slot]["pos"][2]   # tâm bộ phận khi đã lắp
        return Target(x, y, yaw, tw[0], tw[1], z=z)

    # --- truy vấn cho trạm ---------------------------------------------------
    def next_part(self, cls, x_lo, x_hi, t):
        """Bộ phận đúng lớp, chưa ai nhận, x dự đoán trong [x_lo, x_hi]; chọn cái sắp qua nhất."""
        best, best_x = None, -1e9
        for tr in self.tracks.values():
            if tr.cls != cls or tr.state != "on_belt" or tr.claimed_by is not None:
                continue
            if self.s_now(tr, t) > L1:
                continue
            x, _, _ = self.predict(tr, t)
            if x_lo <= x <= x_hi and x > best_x:
                best, best_x = tr, x
        return best

    def next_car(self, slot, x_lo, x_hi, t):
        """Xe đang lắp, chắc hướng mũi, slot còn trống, x của SLOT trong [x_lo, x_hi]."""
        best, best_x = None, -1e9
        for car in self.cars.values():
            if car.state != "assembling" or not car.heading_ok or car.slots[slot] is not None:
                continue
            x = self.slot_target(car, slot, t).x
            if x_lo <= x <= x_hi and x > best_x:
                best, best_x = car, x
        return best

    def claim(self, tr, station_id):
        if tr.claimed_by is not None or tr.state != "on_belt":
            return False
        tr.claimed_by, tr.state = station_id, "claimed"
        return True

    def unclaim(self, tr):
        tr.claimed_by, tr.state = None, "on_belt"

    def mark(self, tr, state):
        tr.state = state

    def attached(self, car: Car, slot, tr):
        car.slots[slot] = tr.id
        tr.state = "attached"
        if car.complete:
            car.state = "complete"

    def expire(self, t):
        """Track đã vào khúc cua mà chưa ai lấy → surplus; xoá track đã ra khỏi băng."""
        for tid, tr in list(self.tracks.items()):
            if tr.state in ("grasped", "attached"):
                continue
            s = self.s_now(tr, t)
            if s > L1 and tr.state in ("on_belt", "claimed") and tr.id not in self.cars:
                tr.state = "surplus"
            if s > L2 + (C.SURPLUS_Y[1] - CY) + 0.05:
                self.tracks.pop(tid)
                car = self.cars.pop(tid, None)
                if car is not None and car.state == "assembling":
                    car.state = "surplus"
