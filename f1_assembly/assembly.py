"""Cơ cấu "bắt vít": bộ phận thả trong dung sai của slot thì được khoá vào thân xe.

Đây là phần vật lý của sim (giống chốt định vị của đồ gá thật), nên được đọc pose thật.
Robot và tracker không gọi pose thật; chỉ nhận kết quả attached / không.
"""
import math

import pybullet as p

from . import config as C
from .parts import wrap_pi


def _yaw(q):
    return p.getEulerFromQuaternion(q)[2]


def find_chassis(world, x, y, max_dist=0.25):
    """Thân xe gần điểm (x, y) nhất — cơ cấu lắp biết nó đang chạm vào xe nào."""
    best, best_d = None, max_dist
    for bid, cls in world.parts.items():
        if cls != "body" or bid in world.grasped:
            continue
        (bx, by, _), _ = p.getBasePositionAndOrientation(bid)
        d = math.hypot(bx - x, by - y)
        if d < best_d:
            best, best_d = bid, d
    return best


def try_attach(world, part_bid, chassis_bid, slot):
    """Trả (ok, err_pos_m, err_yaw_rad). ok → tạo constraint cố định bộ phận vào slot."""
    s = C.SLOTS[slot]
    c_pos, c_orn = p.getBasePositionAndOrientation(chassis_bid)
    q_pos, q_orn = p.getBasePositionAndOrientation(part_bid)
    inv_p, inv_q = p.invertTransform(c_pos, c_orn)
    rel_p, rel_q = p.multiplyTransforms(inv_p, inv_q, q_pos, q_orn)
    err_pos = math.hypot(rel_p[0] - s["pos"][0], rel_p[1] - s["pos"][1])
    if s["yaw_sym"] is None:                 # bánh tròn: bỏ qua yaw
        err_yaw = 0.0
    else:                                    # cánh đối xứng 180°
        err_yaw = abs(math.remainder(wrap_pi(_yaw(rel_q) - s["yaw"]), s["yaw_sym"]))
    if err_pos > C.ATTACH_TOL_POS or err_yaw > C.ATTACH_TOL_YAW:
        return False, err_pos, err_yaw
    # khoá đúng tại pose hiện tại: kéo về tâm slot sẽ tạo phản lực giật cả thân xe
    # (đo được: xe lệch 3–11 cm, xoay tới 20° sau mỗi lần lắp)
    cid = p.createConstraint(chassis_bid, -1, part_bid, -1, p.JOINT_FIXED, [0, 0, 0],
                             parentFramePosition=rel_p, childFramePosition=[0, 0, 0],
                             parentFrameOrientation=rel_q)
    p.changeConstraint(cid, maxForce=50)
    world.attached[part_bid] = chassis_bid
    return True, err_pos, err_yaw
