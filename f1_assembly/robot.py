"""Franka Panda: điều khiển như phần cứng (chỉ gửi góc khớp đích) + bộ điều khiển gắp / đặt.

Bộ điều khiển KHÔNG đọc pose thật của vật. Đích gắp / đặt đến từ hàm do bên ngoài truyền vào
(tracker trong hệ thống thật; ground truth chỉ trong demo_pick.py).
"""
import math
from typing import Callable

import pybullet as p

from . import config as C
from .parts import grip_yaw, wrap_half_pi, wrap_pi
from .shared_types import Target

TargetFn = Callable[[float], Target]


class PandaArm:
    ARM = list(range(7))                  # 7 khớp xoay
    FINGERS = (9, 10)                     # 2 ngón tịnh tiến, 0..0.04 m mỗi ngón
    EE = 11                               # link panda_grasptarget
    REST = [0.0, -0.3, 0.0, -2.2, 0.0, 2.0, 0.785]
    TORQUE = [87, 87, 87, 87, 12, 12, 12]   # N·m, theo thông số Panda thật
    # giới hạn cho IK: 7 khớp tay + 2 ngón (IK của PyBullet tính trên mọi khớp không cố định)
    LL = [-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973, 0.0, 0.0]
    UL = [2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973, 0.04, 0.04]

    def __init__(self, base_x, base_y, base_yaw):
        self.base = (base_x, base_y)
        self.base_yaw = base_yaw
        hx, hy, hz = C.PEDESTAL_HALF
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[hx, hy, hz]),
                          p.createVisualShape(p.GEOM_BOX, halfExtents=[hx, hy, hz],
                                              rgbaColor=[0.95, 0.55, 0.10, 1]),
                          [base_x, base_y, -hz])
        self.id = p.loadURDF(C.PANDA_URDF, [base_x, base_y, 0.0],
                             p.getQuaternionFromEuler([0, 0, base_yaw]), useFixedBase=True)
        # đặt tư thế ban đầu: chỉ lúc khởi tạo, không bao giờ khi đang chạy
        for j, q in zip(self.ARM, self.REST):
            p.resetJointState(self.id, j, q)
        for j in self.FINGERS:
            p.resetJointState(self.id, j, 0.04)
            p.changeDynamics(self.id, j, lateralFriction=C.FINGER_FRICTION)
        # 2 ngón luôn đối xứng, như ví dụ panda_sim của PyBullet
        gear = p.createConstraint(self.id, 9, self.id, 10, jointType=p.JOINT_GEAR,
                                  jointAxis=[1, 0, 0], parentFramePosition=[0, 0, 0],
                                  childFramePosition=[0, 0, 0])
        p.changeConstraint(gear, gearRatio=-1, erp=0.1, maxForce=50)
        self.jr = [u - l for l, u in zip(self.LL, self.UL)]
        self.rest_ik = self.REST + [0.02, 0.02]
        self.dt = C.DT * C.CONTROL_EVERY
        self.cmd = list(self.ee_pos())
        self.assist = None
        self.yaw_cmd = wrap_half_pi(base_yaw)   # yaw đầu gắp lệnh gần nhất; giữ nguyên khi không có đích mới
        self.set_gripper(0.08)

    # --- cảm biến nội tại (robot thật cũng có) --------------------------------
    def ee_pos(self):
        return p.getLinkState(self.id, self.EE, computeForwardKinematics=True)[4]

    def ee_pose(self):
        s = p.getLinkState(self.id, self.EE, computeForwardKinematics=True)
        return s[4], s[5]

    def gripper_width(self):
        return sum(p.getJointState(self.id, j)[0] for j in self.FINGERS)

    def home(self):
        """Vị trí chờ: cách đế HOME_DIST về phía băng, z = SAFE_Z."""
        return (self.base[0] + C.HOME_DIST * math.cos(self.base_yaw),
                self.base[1] + C.HOME_DIST * math.sin(self.base_yaw), C.SAFE_Z)

    def reach(self, x, y):
        return math.hypot(x - self.base[0], y - self.base[1])

    # --- lệnh ----------------------------------------------------------------
    def servo(self, goal, yaw, max_speed=C.EE_MAX_SPEED):
        """Đưa đầu gắp về goal (x, y, z), hướng xuống, xoay yaw. Gọi mỗi tick điều khiển."""
        step = max_speed * self.dt
        d = [g - c for g, c in zip(goal, self.cmd)]
        n = math.sqrt(sum(v * v for v in d))
        if n > step:
            d = [v * step / n for v in d]
        self.cmd = [c + v for c, v in zip(self.cmd, d)]
        self.yaw_cmd = yaw
        orn = p.getQuaternionFromEuler([math.pi, 0.0, yaw])
        q = p.calculateInverseKinematics(self.id, self.EE, self.cmd, orn,
                                         lowerLimits=self.LL, upperLimits=self.UL,
                                         jointRanges=self.jr, restPoses=self.rest_ik,
                                         maxNumIterations=50, residualThreshold=1e-4)
        for j in self.ARM:
            p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL, targetPosition=q[j],
                                    force=self.TORQUE[j], maxVelocity=C.JOINT_MAX_VEL,
                                    positionGain=C.POSITION_GAIN)

    def set_gripper(self, width, force=C.GRIP_FORCE):
        for j in self.FINGERS:
            p.setJointMotorControl2(self.id, j, p.POSITION_CONTROL,
                                    targetPosition=max(0.0, min(0.04, width / 2)), force=force)

    def body_between_fingers(self, exclude=()):
        """Body đang chạm CẢ 2 ngón (đọc contact của chính ngón tay)."""
        a = {c[2] for c in p.getContactPoints(bodyA=self.id, linkIndexA=self.FINGERS[0])}
        b = {c[2] for c in p.getContactPoints(bodyA=self.id, linkIndexA=self.FINGERS[1])}
        both = (a & b) - {self.id} - set(exclude)
        return next(iter(both), None)

    def add_assist(self, body):
        """Grasp assist: khoá vật vào đầu gắp ở pose tương đối hiện tại (không sửa cú gắp lệch)."""
        ee_p, ee_q = self.ee_pose()
        b_p, b_q = p.getBasePositionAndOrientation(body)
        inv_p, inv_q = p.invertTransform(ee_p, ee_q)
        rel_p, rel_q = p.multiplyTransforms(inv_p, inv_q, b_p, b_q)
        self.assist = p.createConstraint(self.id, self.EE, body, -1, p.JOINT_FIXED, [0, 0, 0],
                                         rel_p, [0, 0, 0], rel_q, [0, 0, 0, 1])
        p.changeConstraint(self.assist, maxForce=50)

    def drop_assist(self):
        if self.assist is not None:
            p.removeConstraint(self.assist)
            self.assist = None


class PickPlaceController:
    """Máy trạng thái gắp vật đang chạy rồi đặt vào đích (cũng có thể đang chạy).

    IDLE → INTERCEPT → DESCEND → GRIP → LIFT → HOLD → TRACK_PLACE → PLACE → RELEASE → RETRACT → IDLE
    """

    def __init__(self, arm: PandaArm, world, station_id, assist=True,
                 on_event: Callable[[str, dict], None] | None = None):
        self.arm = arm
        self.world = world
        self.sid = station_id
        self.use_assist = assist
        self.on_event = on_event or (lambda name, data: None)
        self.state = "IDLE"
        self.t_enter = 0.0
        self.cls = None
        self.pick_fn: TargetFn | None = None
        self.place_fn: TargetFn | None = None
        self.held = None              # body id đang kẹp (từ contact ngón tay)
        self.grip_rel = 0.0           # yaw đầu gắp − yaw vật lúc gắp
        self.grip_off = (0.0, 0.0)    # vị trí vật trong hệ đầu gắp (ước lượng lúc kẹp)
        self._stable_since = None

    # --- giao việc -------------------------------------------------------------
    @property
    def holding(self):
        return self.state in ("HOLD", "TRACK_PLACE", "PLACE")

    def assign_pick(self, cls, pick_fn: TargetFn, t):
        assert self.state == "IDLE"
        self.cls, self.pick_fn = cls, pick_fn
        self._go("INTERCEPT", t)

    def assign_place(self, place_fn: TargetFn, t):
        assert self.state == "HOLD"
        self.place_fn = place_fn
        self._go("TRACK_PLACE", t)

    # --- nội bộ -----------------------------------------------------------------
    def _go(self, state, t):
        self.state, self.t_enter = state, t
        self._stable_since = None

    def _elapsed(self, t):
        return t - self.t_enter

    def _grasp_z(self):
        return max(C.PARTS[self.cls]["height"] / 2, C.GRASP_Z_MIN) if self.cls in C.PARTS \
            else C.GRASP_Z_MIN

    def _lead(self, tg: Target):
        """Bù trễ bộ điều khiển: đích dịch trước theo vận tốc vật."""
        return tg.x + tg.vx * C.TAU_LAG, tg.y + tg.vy * C.TAU_LAG

    def _place_xy(self, tg: Target, yaw):
        """Điểm đặt đầu gắp sao cho VẬT (không phải đầu gắp) rơi đúng đích."""
        lx, ly = self._lead(tg)
        ox, oy = self.grip_off
        c, sn = math.cos(yaw), math.sin(yaw)
        return lx - (c * ox - sn * oy), ly - (sn * ox + c * oy)

    def _place_yaw(self, tg: Target):
        yaw = tg.yaw + self.grip_rel
        symmetric = self.cls in ("front_wing", "rear_wing", "wheel")
        return wrap_half_pi(yaw) if symmetric else wrap_pi(yaw)

    def _xy_err(self, tg: Target):
        ex, ey, _ = self.arm.ee_pos()
        return math.hypot(ex - tg.x, ey - tg.y)

    def _stable(self, ok, t, hold=0.2):
        if not ok:
            self._stable_since = None
            return False
        if self._stable_since is None:
            self._stable_since = t
        return t - self._stable_since >= hold

    def _fail(self, name, t, next_state="RETRACT"):
        self.on_event(name, dict(station=self.sid, cls=self.cls, t=t))
        self._go(next_state, t)

    def update(self, t):
        """Gọi mỗi tick điều khiển (60 Hz)."""
        arm, s = self.arm, self.state
        if s == "IDLE" or s == "HOLD":
            hx, hy, hz = arm.home()
            arm.servo((hx, hy, hz), arm.yaw_cmd)
            return

        if s == "INTERCEPT":
            tg = self.pick_fn(t)
            yaw = grip_yaw(self.cls, tg.yaw)
            self.grip_rel = wrap_pi(yaw - tg.yaw)
            lx, ly = self._lead(tg)
            arm.servo((lx, ly, C.HOVER_Z), yaw)
            if self._stable(self._xy_err(tg) < 0.01, t):
                self._go("DESCEND", t)
            elif self._elapsed(t) > 4.0:
                self._fail("pick_timeout", t, "IDLE")

        elif s == "DESCEND":
            tg = self.pick_fn(t)
            lx, ly = self._lead(tg)
            z = self._grasp_z()
            err = self._xy_err(tg)
            ez = arm.ee_pos()[2]
            # chỉ hạ khi đã thẳng hàng với vật; lệch thì giữ độ cao để ngón không quẹt hất vật
            z_goal = z if err < C.ALIGN_TOL else max(ez, z + 0.03)
            arm.servo((lx, ly, z_goal), grip_yaw(self.cls, tg.yaw), C.DESCEND_SPEED)
            if abs(ez - z) < 0.005 and err < C.GRIP_TOL:
                arm.set_gripper(0.0, C.GRIP_FORCE_CAR if self.cls == "car" else C.GRIP_FORCE)
                self._go("GRIP", t)
            elif self._elapsed(t) > 3.0:
                self._fail("pick_fail_descend", t)

        elif s == "GRIP":
            tg = self.pick_fn(t)
            lx, ly = self._lead(tg)
            arm.servo((lx, ly, self._grasp_z()), grip_yaw(self.cls, tg.yaw))
            if self._elapsed(t) < 0.3:
                return
            width = arm.gripper_width()
            body = arm.body_between_fingers(exclude=self.world.belt.surfaces)
            if width < 0.004 or body is None:
                arm.set_gripper(0.08)
                self._fail("pick_miss", t)
                return
            self.held = body
            self.world.grasped.add(body)
            # vật lệch bao nhiêu trong kẹp: đích (ước lượng) − đầu gắp, đổi sang hệ đầu gắp
            ex, ey, _ = arm.ee_pos()
            dx, dy = tg.x - ex, tg.y - ey
            c, sn = math.cos(arm.yaw_cmd), math.sin(arm.yaw_cmd)
            self.grip_off = (c * dx + sn * dy, -sn * dx + c * dy)
            if self.use_assist:
                arm.add_assist(body)
            self.on_event("grasped", dict(station=self.sid, cls=self.cls, t=t, body=body,
                                          width=width))
            self._go("LIFT", t)

        elif s == "LIFT":
            ex, ey, _ = arm.ee_pos()
            arm.servo((ex, ey, C.CARRY_Z), arm.yaw_cmd)
            if arm.gripper_width() < 0.004:
                self._release_held()
                self._fail("dropped_in_lift", t)
            elif arm.ee_pos()[2] > C.CARRY_Z - 0.01:
                self._go("HOLD", t)
            elif self._elapsed(t) > 1.5:
                self._go("HOLD", t)

        elif s == "TRACK_PLACE":
            tg = self.place_fn(t)
            yaw = self._place_yaw(tg)
            lx, ly = self._place_xy(tg, yaw)
            arm.servo((lx, ly, tg.z + 0.06), yaw)
            if arm.reach(tg.x, tg.y) > 0.78:
                self._fail("place_out_of_reach", t, "HOLD")
            elif self._stable(math.hypot(arm.ee_pos()[0] - lx, arm.ee_pos()[1] - ly) < 0.01, t):
                self._go("PLACE", t)
            elif self._elapsed(t) > 3.0:
                self._fail("place_timeout", t, "HOLD")

        elif s == "PLACE":
            tg = self.place_fn(t)
            yaw = self._place_yaw(tg)
            lx, ly = self._place_xy(tg, yaw)
            z = max(tg.z + 0.002, C.GRASP_Z_MIN)
            ez = arm.ee_pos()[2]
            z_goal = z if math.hypot(arm.ee_pos()[0] - lx, arm.ee_pos()[1] - ly) < C.ALIGN_TOL else max(ez, z + 0.03)
            arm.servo((lx, ly, z_goal), yaw, C.DESCEND_SPEED)
            if abs(ez - z) < 0.005 or self._elapsed(t) > 3.0:
                arm.set_gripper(0.08)
                self._go("RELEASE", t)

        elif s == "RELEASE":
            ex, ey, ez = arm.ee_pos()
            arm.servo((ex, ey, ez), arm.yaw_cmd)
            if self._elapsed(t) > 0.3:
                body = self.held
                tg = self.place_fn(t)
                self._release_held()
                self.on_event("released", dict(station=self.sid, cls=self.cls, t=t, body=body,
                                               target=tg))
                self._go("RETRACT", t)

        elif s == "RETRACT":
            ex, ey, ez = arm.ee_pos()
            if ez < C.SAFE_Z - 0.02:
                arm.servo((ex, ey, C.SAFE_Z), arm.yaw_cmd)
            else:
                hx, hy, hz = arm.home()
                arm.servo((hx, hy, hz), arm.yaw_cmd)
                if math.hypot(ex - hx, ey - hy) < 0.03 or self._elapsed(t) > 3.0:
                    self.pick_fn = self.place_fn = None
                    self._go("IDLE", t)

    def _release_held(self):
        self.arm.drop_assist()
        self.arm.set_gripper(0.08)
        if self.held is not None:
            self.world.grasped.discard(self.held)
        self.held = None
