"""Thế giới PyBullet: băng chuyền rẽ nhánh, spawn theo loại, xoá vật ra khỏi dây chuyền."""
import math
import random

import pybullet as p
import pybullet_data

from . import config as C
from .parts import make_part


def belt_twist(x, y, v):
    """Vận tốc (vx, vy, ωz) băng chuyền ép lên vật tại (x, y); None nếu không ở trên băng."""
    if 0.0 <= x <= C.STRAIGHT_END and abs(y) <= C.BELT_HALF_W:
        return v, 0.0, 0.0
    rx, ry = x - C.CURVE_CENTER[0], y - C.CURVE_CENTER[1]
    if rx >= 0.0 and ry <= 0.0 and C.CURVE_R_IN <= math.hypot(rx, ry) <= C.CURVE_R_OUT:
        w = v / C.CURVE_R                       # quay cứng quanh tâm cua
        return -w * ry, w * rx, w
    if C.SURPLUS_X[0] <= x <= C.SURPLUS_X[1] and C.SURPLUS_Y[0] <= y <= C.SURPLUS_Y[1]:
        return 0.0, v, 0.0
    if C.QC_X[0] <= x <= C.QC_X[1] and abs(y) <= C.BELT_HALF_W:
        return v, 0.0, 0.0
    return None


class Belt:
    """Mặt băng có vận tốc bề mặt: ma sát của chính mặt băng kéo vật đi, như băng chuyền thật.

    Mỗi mặt băng là vật khối lượng 0. Trước mỗi bước sim đặt vận tốc cho nó (khúc cua: quay
    quanh tâm cua) để bộ giải tiếp xúc tính ma sát như mặt đang chạy; sau bước sim đặt lại vị trí
    cũ để nó không trôi đi. Không ép vận tốc từng vật, nên tay kẹp vật lên là vật tự rời băng.
    """

    def __init__(self, rng):
        self.rng = rng
        self.surfaces = set()
        self._home = {}                 # body id -> pose cố định
        self._twist1 = {}               # body id -> twist ứng với v = 1 m/s
        hz = C.BELT_THICK / 2
        # đoạn thẳng lắp ráp
        self._box((C.STRAIGHT_END / 2, 0, -hz), (C.STRAIGHT_END / 2, C.BELT_HALF_W, hz), 0)
        # cua: các hộp xếp theo cung, phủ góc từ -90° đến 0° quanh tâm cua
        cx, cy = C.CURVE_CENTER
        dth = (math.pi / 2) / C.CURVE_SEGMENTS
        half_t = C.CURVE_R_OUT * math.tan(dth / 2) + 0.005   # chồng nhẹ để không hở khe
        for i in range(C.CURVE_SEGMENTS):
            th = -math.pi / 2 + (i + 0.5) * dth
            pos = (cx + C.CURVE_R * math.cos(th), cy + C.CURVE_R * math.sin(th), -hz)
            self._box(pos, (half_t, C.BELT_HALF_W, hz), th + math.pi / 2)
        # lối Surplus
        sx = sum(C.SURPLUS_X) / 2
        sy = sum(C.SURPLUS_Y) / 2
        self._box((sx, sy, -hz), (C.BELT_HALF_W, (C.SURPLUS_Y[1] - C.SURPLUS_Y[0]) / 2, hz), 0)
        # băng QC
        qx = sum(C.QC_X) / 2
        self._box((qx, 0, -hz), ((C.QC_X[1] - C.QC_X[0]) / 2, C.BELT_HALF_W, hz), 0)

    def _box(self, pos, half, yaw):
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=half)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=half, rgbaColor=C.BELT_COLOR)
        bid = p.createMultiBody(0, col, vis, pos, p.getQuaternionFromEuler([0, 0, yaw]))
        p.changeDynamics(bid, -1, lateralFriction=C.BELT_FRICTION)
        self.surfaces.add(bid)
        self._home[bid] = (tuple(pos), p.getQuaternionFromEuler([0, 0, yaw]))
        self._twist1[bid] = belt_twist(pos[0], pos[1], 1.0)

    def speeds(self, speed_scale, slip):
        """(v_true, v_enc): vận tốc thật và vận tốc encoder báo về."""
        v_cmd = C.V_NOMINAL * speed_scale
        if v_cmd == 0.0:
            return 0.0, 0.0                 # băng dừng: encoder đứng yên, không trôi nhiễu
        return v_cmd * (1.0 - slip), v_cmd + self.rng.gauss(0.0, C.SIGMA_ENC)

    def pre_step(self, v_true):
        for bid, (vx, vy, wz) in self._twist1.items():
            p.resetBaseVelocity(bid, [vx * v_true, vy * v_true, 0.0], [0.0, 0.0, wz * v_true])

    def post_step(self):
        for bid, (pos, orn) in self._home.items():
            p.resetBasePositionAndOrientation(bid, pos, orn)


class World:
    def __init__(self, gui=True, seed=0):
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, C.GRAVITY)
        p.setTimeStep(C.DT)
        p.setPhysicsEngineParameter(numSolverIterations=C.SOLVER_ITERATIONS)
        if gui:
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
            p.resetDebugVisualizerCamera(cameraDistance=3.2, cameraYaw=0, cameraPitch=-70,
                                         cameraTargetPosition=[2.5, 0.4, 0])
        p.loadURDF("plane.urdf", [0, 0, C.FLOOR_Z])
        self.rng = random.Random(seed)
        self.belt = Belt(self.rng)
        self.parts = {}                 # body id -> lớp
        self.grasped = set()            # vật đang bị tay kẹp (để logic khác bỏ qua)
        self.attached = {}              # body id bộ phận -> body id thân xe
        self.counters = dict(bodies=0, surplus=0, output=0, dropped=0)
        self.t = 0.0
        self.step_count = 0
        self._auto_clock = 0.0
        self._auto_bag = []

    # --- spawn ------------------------------------------------------------
    def _free(self, cls, x, y):
        r = C.PART_RADIUS[cls]
        for bid, other in self.parts.items():
            (ox, oy, _), _ = p.getBasePositionAndOrientation(bid)
            if math.hypot(ox - x, oy - y) < r + C.PART_RADIUS[other]:
                return False
        return True

    def spawn(self, cls, allow_overlap=False):
        """Thả 1 bộ phận vào vùng INPUT. Trả body id, hoặc None nếu hết chỗ."""
        y_lim = C.SPAWN_Y_BODY if cls == "body" else C.SPAWN_Y
        for _ in range(C.SPAWN_TRIES):
            x = self.rng.uniform(*C.SPAWN_X)
            y = self.rng.uniform(-y_lim, y_lim)
            if allow_overlap or self._free(cls, x, y):
                break
        else:
            return None
        bid = make_part(cls, x, y, self.rng.uniform(-math.pi, math.pi))
        self.parts[bid] = cls
        if cls == "body":
            self.counters["bodies"] += 1
        return bid

    def auto_spawn(self, speed_scale, allow_overlap=False):
        """Thả theo tỉ lệ AUTO_RATIO, chu kỳ AUTO_PERIOD / tốc độ."""
        self._auto_clock += C.DT
        if self._auto_clock < C.AUTO_PERIOD / max(speed_scale, 1e-3):
            return
        self._auto_clock = 0.0
        if not self._auto_bag:
            self._auto_bag = [c for c, n in C.AUTO_RATIO.items() for _ in range(n)]
            self.rng.shuffle(self._auto_bag)
        cls = self._auto_bag[-1]
        if self.spawn(cls, allow_overlap) is not None:
            self._auto_bag.pop()

    # --- vòng đời vật -----------------------------------------------------
    def remove_exited(self):
        """Xoá vật ra Surplus (y > 1.6), ra OUTPUT (x > 5.0) hoặc rơi khỏi băng. Trả list (bid, lý do)."""
        gone = []
        for bid in list(self.parts):
            if bid in self.attached:
                continue                        # đi cùng thân xe
            (x, y, z), _ = p.getBasePositionAndOrientation(bid)
            if y > C.SURPLUS_Y[1]:
                reason = "surplus"
            elif x > C.QC_X[1]:
                reason = "output"
            elif z < C.FLOOR_Z + 0.02:
                reason = "dropped"
            else:
                continue
            self._remove_tree(bid)
            self.counters[reason] += 1
            gone.append((bid, reason))
        return gone

    def _remove_tree(self, bid):
        for child in [c for c, parent in self.attached.items() if parent == bid]:
            del self.attached[child]
            self.parts.pop(child, None)
            p.removeBody(child)
        self.grasped.discard(bid)
        self.parts.pop(bid, None)
        p.removeBody(bid)

    def step(self, speed_scale=1.0, slip=0.0):
        """1 bước sim. Trả v_enc (encoder) cho tracker."""
        v_true, v_enc = self.belt.speeds(speed_scale, slip)
        self.belt.pre_step(v_true)
        p.stepSimulation()
        self.belt.post_step()
        self.t += C.DT
        self.step_count += 1
        return v_enc

    def true_pose(self, bid):
        """Pose thật (x, y, yaw). CHỈ dùng cho metrics và demo thay tracker, robot thật không có."""
        (x, y, _), q = p.getBasePositionAndOrientation(bid)
        return x, y, p.getEulerFromQuaternion(q)[2]
