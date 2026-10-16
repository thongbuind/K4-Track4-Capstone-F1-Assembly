"""Kiểu dữ liệu dùng chung giữa các module.

Đặt tên shared_types.py thay vì types.py để không che module `types` của thư viện chuẩn
khi chạy một file trong thư mục này trực tiếp.
"""
from dataclasses import dataclass, field


@dataclass
class Detection:
    """Một detection trong ảnh. Đơn vị pixel, góc radian."""
    cls: str                  # 'body' | 'front_wing' | 'rear_wing' | 'wheel'
    conf: float
    cx: float
    cy: float
    w: float
    h: float
    angle: float
    heading_ok: bool = True   # chỉ với body: đã xác định được đầu mũi


@dataclass
class PartTrack:
    """Một bộ phận trong map, toạ độ theo đường băng (s, d, ψ) tại lần thấy cuối."""
    id: int
    cls: str
    conf: float
    s0: float
    d: float
    psi: float
    t0: float
    state: str = "on_belt"    # on_belt | claimed | grasped | attached | surplus
    claimed_by: int | None = None
    true_body_id: int | None = None   # chỉ để đo sai số; robot KHÔNG được đọc


@dataclass
class Car:
    """Một thân xe + 6 slot."""
    body: PartTrack
    slots: dict[str, int | None] = field(default_factory=lambda: {
        "front_wing": None, "rear_wing": None, "FL": None, "FR": None, "RL": None, "RR": None})
    state: str = "assembling"  # assembling | complete | transferred | surplus
    heading_ok: bool = True    # đã chắc đầu mũi; chưa chắc thì các trạm bỏ qua xe này

    @property
    def complete(self) -> bool:
        return all(v is not None for v in self.slots.values())


@dataclass
class Target:
    """Đích cho bộ điều khiển tay: pose trên mặt phẳng + vận tốc (để bù trễ)."""
    x: float
    y: float
    yaw: float
    vx: float = 0.0
    vy: float = 0.0
    z: float = 0.0            # độ cao tâm đích so với mặt băng
