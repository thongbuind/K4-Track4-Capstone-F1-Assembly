"""Mọi hằng số của sim. Đơn vị: mét, giây, radian, kg."""
import math

# --- Sim -------------------------------------------------------------------
DT = 1.0 / 240.0              # bước sim
CONTROL_EVERY = 4             # bộ điều khiển tay chạy 240 / 4 = 60 Hz
GRAVITY = -9.81
SOLVER_ITERATIONS = 50
FLOOR_Z = -0.10               # mặt sàn; mặt băng chuyền ở z = 0

# --- Băng chuyền -----------------------------------------------------------
BELT_HALF_W = 0.25
BELT_THICK = 0.05
BELT_FRICTION = 0.8             # ma sát mặt băng; mặt băng có vận tốc bề mặt nên ma sát kéo vật đi
BELT_COLOR = (0.18, 0.18, 0.20, 1.0)

STRAIGHT_END = 3.0                    # đoạn thẳng lắp ráp: x ∈ [0, 3.0]
CURVE_CENTER = (3.0, 0.6)             # cua trái 90°
CURVE_R = 0.6                         # bán kính đường tâm
CURVE_R_IN = CURVE_R - BELT_HALF_W    # 0.35
CURVE_R_OUT = CURVE_R + BELT_HALF_W   # 0.85
CURVE_SEGMENTS = 12                   # số hộp xếp thành cung (chỉ để vật có chỗ đứng)
SURPLUS_X = (3.35, 3.85)              # lối Surplus đi lên +y
SURPLUS_Y = (0.6, 1.6)                # vật vượt y = 1.6 bị xoá + đếm Surplus
QC_X = (3.9, 5.0)                     # băng QC thẳng, cùng tốc độ

V_NOMINAL = 0.05              # m/s ở tốc độ 1.0x
SIGMA_ENC = 0.001             # nhiễu encoder (m/s)
SPEED_RANGE = (0.5, 2.0)

# --- Bộ phận -----------------------------------------------------------------
# shapes: list (kind, half_extents | (radius, height), offset trong hệ bộ phận)
# grip_axis: trục cục bộ của bộ phận mà 2 ngón kẹp dọc theo; grip_width: bề rộng theo trục đó
PARTS = {
    "body": dict(
        shapes=[("box", (0.15, 0.025, 0.015), (0.0, 0.0, 0.0)),
                ("box", (0.03, 0.015, 0.010), (0.18, 0.0, -0.005))],   # mũi xe ở +x
        height=0.03, mass=0.15, grip_width=0.05, grip_axis="y",
        color=(0.85, 0.10, 0.12, 1.0)),
    "front_wing": dict(
        shapes=[("box", (0.015, 0.07, 0.005), (0.0, 0.0, 0.0))],
        height=0.01, mass=0.03, grip_width=0.03, grip_axis="x",
        color=(0.10, 0.35, 0.90, 1.0)),
    "rear_wing": dict(
        shapes=[("box", (0.02, 0.06, 0.01), (0.0, 0.0, 0.0))],
        height=0.02, mass=0.04, grip_width=0.04, grip_axis="x",
        color=(0.98, 0.80, 0.10, 1.0)),
    "wheel": dict(
        shapes=[("cylinder", (0.025, 0.02), (0.0, 0.0, 0.0))],         # trục z, nằm dẹt
        height=0.02, mass=0.03, grip_width=0.05, grip_axis="x",
        color=(0.08, 0.08, 0.08, 1.0)),
}
PART_FRICTION = 0.8
# Bán kính bao (m) dùng khi kiểm tra chồng lúc spawn
PART_RADIUS = {"body": 0.21, "front_wing": 0.072, "rear_wing": 0.063, "wheel": 0.026}

# Slot lắp trong hệ thân xe (x về mũi, y sang trái), tâm bộ phận. yaw_sym: π = đối xứng 180°, None = bỏ qua yaw
# z chọn để đáy bộ phận cao hơn mặt băng 5 mm, không cọ băng khi xe chạy.
SLOTS = {
    "front_wing": dict(cls="front_wing", pos=(0.225, 0.0, -0.005), yaw=0.0, yaw_sym=math.pi),
    "rear_wing":  dict(cls="rear_wing",  pos=(-0.172, 0.0, 0.010), yaw=0.0, yaw_sym=math.pi),
    "FL": dict(cls="wheel", pos=(0.09, 0.055, 0.0), yaw=0.0, yaw_sym=None),
    "FR": dict(cls="wheel", pos=(0.09, -0.055, 0.0), yaw=0.0, yaw_sym=None),
    "RL": dict(cls="wheel", pos=(-0.10, 0.055, 0.0), yaw=0.0, yaw_sym=None),
    "RR": dict(cls="wheel", pos=(-0.10, -0.055, 0.0), yaw=0.0, yaw_sym=None),
}
ATTACH_TOL_POS = 0.015
ATTACH_TOL_YAW = math.radians(15)

# --- Spawn -------------------------------------------------------------------
SPAWN_X = (0.05, 0.65)
SPAWN_Y = 0.18
SPAWN_Y_BODY = 0.12           # thân xe gần giữa để bánh lắp vào vẫn trên băng
SPAWN_TRIES = 200
SPAWN_DROP = 0.005            # thả cao hơn mặt băng 5 mm
AUTO_RATIO = {"body": 1, "front_wing": 1, "rear_wing": 1, "wheel": 4}
AUTO_PERIOD = 2.5             # giây / bộ phận ở 1.0x; túi 7 món = đúng 1 bộ xe mỗi 17.5 s (1.5 s làm băng tắc)

# --- Robot (Franka Panda) ------------------------------------------------------
PANDA_URDF = "franka_panda/panda.urdf"
PEDESTAL_HALF = (0.10, 0.10, 0.05)   # bệ cao 0.10 m dưới đế robot
EE_MAX_SPEED = 0.5            # m/s, giới hạn tốc độ đích IK
DESCEND_SPEED = 0.15          # m/s khi hạ xuống gắp / đặt
ALIGN_TOL = 0.012             # lệch ngang tối đa (m) để được phép hạ z
GRIP_TOL = 0.008              # lệch ngang tối đa (m) để đóng ngón
JOINT_MAX_VEL = 2.0           # rad/s
POSITION_GAIN = 0.3
GRIP_FORCE = 20.0             # N
GRIP_FORCE_CAR = 40.0
FINGER_FRICTION = 1.5
GRIPPER_YAW_OFFSET = 0.0      # ψ0: lệch giữa yaw link EE và hướng mở ngón — đo 1 lần bằng thực nghiệm
TAU_LAG = 0.1                 # s, bù trễ khi bám vật đang chạy
HOVER_Z = 0.12
SAFE_Z = 0.20
HOME_DIST = 0.35             # vị trí chờ cách đế 0.35 m về phía băng (gần hơn thì Panda không với tới khi chúi xuống)
CARRY_Z = 0.15
GRASP_Z_MIN = 0.006           # không hạ đầu gắp thấp hơn mức này (tránh cọ băng)

# Trạm: x, y, base yaw, lớp gắp, slot đích
STATIONS = [
    dict(id=1, x=1.00, y=+0.40, yaw=-math.pi / 2, cls="front_wing", slot="front_wing"),
    dict(id=2, x=1.20, y=-0.40, yaw=+math.pi / 2, cls="rear_wing", slot="rear_wing"),
    dict(id=3, x=1.60, y=+0.40, yaw=-math.pi / 2, cls="wheel", slot="FL"),
    dict(id=4, x=1.80, y=-0.40, yaw=+math.pi / 2, cls="wheel", slot="FR"),
    dict(id=5, x=2.20, y=+0.40, yaw=-math.pi / 2, cls="wheel", slot="RL"),
    dict(id=6, x=2.40, y=-0.40, yaw=+math.pi / 2, cls="wheel", slot="RR"),
    dict(id=7, x=3.40, y=-0.42, yaw=+math.pi / 2, cls="car", slot=None),
]
TRANSFER_PICK = (2.95, 0.0)
TRANSFER_PLACE = (3.95, 0.0)

# --- Camera ----------------------------------------------------------------
CAM_IN_EYE = (0.35, 0.0, 1.00)
CAM_QC_EYE = (4.60, 0.0, 0.80)
# Bản nộp: chưa có tay 07, QC đặt trên băng chính ngay trước khúc cua
CAM_QC_MAIN_EYE = (2.75, 0.0, 0.80)
QC_MATCH_TOL = 0.02          # bộ phận lệch slot quá 2 cm thì QC coi là sai
# Băng dừng khi có tay ở các trạng thái này (stop-and-go)
STOP_STATES = ("DESCEND", "GRIP", "PLACE", "RELEASE")
# camera trên mỗi cặp trạm: chỉ cập nhật track đã có (không tạo mới), để sửa pose xe bị va lệch
CAM_STATION_EYES = [(1.10, 0.0, 0.80), (1.70, 0.0, 0.80), (2.30, 0.0, 0.80)]
CAM_STATION_W, CAM_STATION_H = 320, 240
CAM_STATION_EVERY = 120       # 240 / 120 = 2 Hz mỗi camera trạm
CAM_STATION_MAX_OCCLUSION = 0.2   # bỏ thân xe bị tay che > 20% (mask cụt làm lệch tâm OBB)
CAM_STATION_ROI = 0.40        # mỗi camera trạm chỉ nhận detection trong |x − x_cam| ≤ 0.40
MATCH_DIST = {"body": 0.25, "front_wing": 0.06, "rear_wing": 0.06, "wheel": 0.06}
CAM_W, CAM_H, CAM_FOV = 640, 480, 50.0
