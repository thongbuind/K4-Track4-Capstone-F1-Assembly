"""Demo 1 Panda gắp bộ phận đang chạy trên băng rồi đặt lại xuống làn đối diện (cũng đang chạy).

Mặc định: đích gắp đến từ camera → OracleDetector → tracker (robot không đọc pose thật).
Pose thật chỉ dùng để đo sai số dự đoán lúc gắp. Cờ --gt quay về chế độ cũ (pose thật làm đích).

Chạy từ thư mục cha của f1_assembly:
    python -m f1_assembly.demo_pick --station 1 --seconds 60
"""
import argparse
import math
import statistics
import time

import numpy as np

from . import config as C
from .camera import Camera
from .detector import OracleDetector
from .robot import PandaArm, PickPlaceController
from .shared_types import Target
from .tracker import BeltOdometry, Tracker
from .world import World, belt_twist

CAPTURE_EVERY = 48            # 240 / 48 = 5 Hz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--station", type=int, default=1, help="1..6: tay lắp tương ứng")
    ap.add_argument("--seconds", type=float, default=60.0, help="thời gian sim")
    ap.add_argument("--speed", type=float, default=1.0, help="tốc độ băng (x lần)")
    ap.add_argument("--slip", type=float, default=0.0, help="trượt băng, 0.05 = 5%%")
    ap.add_argument("--every", type=float, default=6.0, help="spawn 1 bộ phận mỗi N giây sim")
    ap.add_argument("--no-assist", action="store_true", help="chỉ giữ vật bằng ma sát")
    ap.add_argument("--gt", action="store_true", help="dùng pose thật làm đích (bỏ qua camera)")
    ap.add_argument("--direct", action="store_true", help="không mở cửa sổ (chạy nhanh)")
    ap.add_argument("--video", default=None, help="ghi video MP4 (chỉ ở chế độ GUI)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    spec = C.STATIONS[args.station - 1]
    cls = spec["cls"]
    world = World(gui=not args.direct, seed=args.seed)
    arm = PandaArm(spec["x"], spec["y"], spec["yaw"])
    rng = np.random.default_rng(args.seed)
    cam = Camera(C.CAM_IN_EYE, rng=rng)
    detector = OracleDetector(world.parts.get, cam, rng=rng)
    odo = BeltOdometry()
    tracker = Tracker(cam, odo)

    stats = dict(spawned=0, grasped=0, placed=0, failed=0)
    errors_mm = []
    detect_ms = []
    current = {}                        # track / body đang được tay xử lý

    def on_event(name, data):
        if name == "grasped":
            stats["grasped"] += 1
            tr = current.get("track")
            if tr is not None:
                tracker.mark(tr, "grasped")
                # sai số dự đoán: so tracker với pose thật tại đúng lúc gắp (chỉ để đo)
                px, py, _ = tracker.predict(tr, data["t"])
                tx, ty, _ = world.true_pose(data["body"])
                errors_mm.append(1000 * math.hypot(px - tx, py - ty))
        elif name == "released":
            stats["placed"] += 1
        else:
            stats["failed"] += 1
            tr = current.get("track")
            if tr is not None and tr.state == "claimed":
                tracker.mark(tr, "missed")
        print(f"[t={data['t']:6.2f}s] trạm {data['station']:02d} {name} {data.get('cls')}")

    ctl = PickPlaceController(arm, world, spec["id"], assist=not args.no_assist,
                              on_event=on_event)
    v = C.V_NOMINAL * args.speed
    h = C.PARTS[cls]["height"]

    def gt_target(bid):
        def fn(t):                      # --gt: pose thật thay cho tracker
            x, y, yaw = world.true_pose(bid)
            tw = belt_twist(x, y, v) or (0.0, 0.0, 0.0)
            return Target(x, y, yaw, tw[0], tw[1], z=h / 2)
        return fn

    def drop_target(t0):
        # điểm đặt chạy cùng băng: làn đối diện của tay, lệch 0.10 m về hạ lưu, yaw = 0
        y_drop = -0.10 if spec["y"] > 0 else 0.10
        x0 = spec["x"] + 0.10

        def fn(t):
            ds = odo.s_at(t) - odo.s_at(t0)
            return Target(x0 + ds, y_drop, 0.0, odo.v_last, 0.0, z=h / 2)
        return fn

    if args.video and not args.direct:
        import pybullet as p
        p.resetDebugVisualizerCamera(cameraDistance=1.3, cameraYaw=20, cameraPitch=-55,
                                     cameraTargetPosition=[spec["x"] - 0.2, 0.0, 0.0])
        p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.video)
    tried = set()
    next_spawn = 0.0
    wall0 = time.time()
    while world.t < args.seconds:
        t = world.t
        if t >= next_spawn:
            if world.spawn(cls) is not None:
                stats["spawned"] += 1
            next_spawn = t + args.every
        v_enc = world.step(args.speed, args.slip)
        odo.step(world.t, v_enc, C.DT)
        t = world.t
        if not args.gt and world.step_count % CAPTURE_EVERY == 0:
            rgb, seg, t_cap = cam.capture(t)
            t0 = time.perf_counter()
            dets = detector.detect(rgb, seg)
            detect_ms.append(1000 * (time.perf_counter() - t0))
            tracker.ingest(dets, t_cap)
        if world.step_count % C.CONTROL_EVERY:
            continue
        x_lo, x_hi = spec["x"] - 0.30, spec["x"] + 0.10
        if ctl.state == "IDLE":
            current.clear()
            if args.gt:
                for bid, c in world.parts.items():
                    if c == cls and bid not in tried and bid not in world.grasped \
                            and x_lo <= world.true_pose(bid)[0] <= x_hi:
                        tried.add(bid)
                        ctl.assign_pick(cls, gt_target(bid), t)
                        break
            else:
                tr = tracker.next_part(cls, x_lo, x_hi, t)
                if tr is not None and tracker.claim(tr, spec["id"]):
                    current["track"] = tr
                    ctl.assign_pick(cls, lambda tt, tr=tr: tracker.target(tr, tt), t)
        elif ctl.state == "HOLD":
            ctl.assign_place(drop_target(t), t)
        ctl.update(t)
        tracker.expire(t)
        world.remove_exited()
        if not args.direct:
            ahead = world.t - (time.time() - wall0)
            if ahead > 0:
                time.sleep(ahead)

    if args.video and not args.direct:
        import pybullet as p
        p.disconnect()                       # đóng file video
    print("\nKết quả:", stats, "| bộ đếm world:", world.counters)
    if errors_mm:
        print(f"Sai số dự đoán lúc gắp: trung bình {statistics.mean(errors_mm):.1f} mm, "
              f"lớn nhất {max(errors_mm):.1f} mm (n = {len(errors_mm)})")
    if detect_ms:
        print(f"Detector: {statistics.mean(detect_ms):.1f} ms / ảnh (n = {len(detect_ms)}), "
              f"số track: {len(tracker.tracks)}")


if __name__ == "__main__":
    main()
