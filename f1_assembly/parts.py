"""Hình học 4 loại bộ phận + slot lắp trên thân xe."""
import math

import pybullet as p

from . import config as C


def _shape_args(shapes):
    kinds, half, radii, lengths, offsets = [], [], [], [], []
    for kind, dims, off in shapes:
        if kind == "box":
            kinds.append(p.GEOM_BOX)
            half.append(list(dims))
            radii.append(0.0)
            lengths.append(0.0)
        else:
            kinds.append(p.GEOM_CYLINDER)
            half.append([0.0, 0.0, 0.0])
            radii.append(dims[0])
            lengths.append(dims[1])
        offsets.append(list(off))
    return kinds, half, radii, lengths, offsets


def make_part(cls, x, y, yaw, z=None):
    """Tạo 1 bộ phận nằm trên băng, trả về body id."""
    spec = C.PARTS[cls]
    kinds, half, radii, lengths, offsets = _shape_args(spec["shapes"])
    col = p.createCollisionShapeArray(shapeTypes=kinds, halfExtents=half, radii=radii,
                                      lengths=lengths, collisionFramePositions=offsets)
    vis = p.createVisualShapeArray(shapeTypes=kinds, halfExtents=half, radii=radii,
                                   lengths=lengths, visualFramePositions=offsets)
    if z is None:
        z = spec["height"] / 2 + C.SPAWN_DROP
    bid = p.createMultiBody(baseMass=spec["mass"], baseCollisionShapeIndex=col,
                            baseVisualShapeIndex=vis, basePosition=[x, y, z],
                            baseOrientation=p.getQuaternionFromEuler([0, 0, yaw]))
    p.changeVisualShape(bid, -1, rgbaColor=spec["color"])
    p.changeDynamics(bid, -1, lateralFriction=C.PART_FRICTION)
    return bid


def grip_yaw(cls, part_yaw):
    """Yaw của đầu gắp để 2 ngón kẹp dọc trục grip_axis của bộ phận.

    Gripper đối xứng 180° nên đưa về [-π/2, π/2] để khớp 7 không xoay vòng.
    """
    axis_offset = 0.0 if C.PARTS[cls]["grip_axis"] == "x" else math.pi / 2
    return wrap_half_pi(part_yaw + axis_offset + C.GRIPPER_YAW_OFFSET)


def wrap_pi(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def wrap_half_pi(a):
    a = wrap_pi(a)
    if a > math.pi / 2:
        a -= math.pi
    elif a < -math.pi / 2:
        a += math.pi
    return a


def obb_center_offset(cls):
    """Tâm hình bao nhìn từ trên (cái camera đo được) so với gốc bộ phận, trong hệ bộ phận."""
    xs, ys = [], []
    for kind, dims, (ox, oy, _) in C.PARTS[cls]["shapes"]:
        hx, hy = (dims[0], dims[1]) if kind == "box" else (dims[0], dims[0])
        xs += [ox - hx, ox + hx]
        ys += [oy - hy, oy + hy]
    return (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2


def slot_world_pose(chassis_x, chassis_y, chassis_yaw, slot_name):
    """Pose (x, y, yaw) của tâm slot trong world, từ pose thân xe."""
    s = C.SLOTS[slot_name]
    sx, sy, _ = s["pos"]
    c, sn = math.cos(chassis_yaw), math.sin(chassis_yaw)
    return (chassis_x + c * sx - sn * sy,
            chassis_y + sn * sx + c * sy,
            chassis_yaw + s["yaw"])
