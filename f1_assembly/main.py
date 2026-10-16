"""Dây chuyền: camera → Oracle detector → tracker → 6 tay lắp phụ tùng lên thân xe đang chạy.

Chưa có tay 07 (chuyển xe). QC dùng camera trên băng chính trước Surplus; số liệu in ở cuối.

Chạy từ thư mục cha của f1_assembly:
    python -m f1_assembly.main --seconds 120 --direct
"""
import argparse
import math
import statistics
import time

import numpy as np

from . import config as C
from .assembly import find_chassis, try_attach
from .camera import Camera
from .detector import OracleDetector
import pybullet as p

from .qc import QCStation
from .robot import PandaArm, PickPlaceController
from .tracker import BeltOdometry, Tracker
from .world import World

CAPTURE_EVERY = 48            # 240 / 48 = 5 Hz


class Station:
    """1 tay lắp: gắp bộ phận lớp `cls` từ băng, giữ sẵn, lắp vào `slot` của xe kế tiếp."""

    def __init__(self, spec, world, tracker, stats, assist=True, log=print):
        self.spec = spec
        self.world = world
        self.tracker = tracker
        self.stats = stats
        self.log = log
        self.arm = PandaArm(spec["x"], spec["y"], spec["yaw"])
        self.ctl = PickPlaceController(self.arm, world, spec["id"], assist=assist,
                                       on_event=self.on_event)
        self.track = None             # bộ phận đang gắp / giữ
        self.car = None               # xe đang lắp vào

    def update(self, t):
        sp, ctl, tr = self.spec, self.ctl, self.tracker
        if ctl.state == "IDLE":
            self.track = self.car = None
            part = tr.next_part(sp["cls"], sp["x"] - 0.30, sp["x"] + 0.10, t)
            if part is not None and tr.claim(part, sp["id"]):
                self.track = part
                ctl.assign_pick(sp["cls"], lambda tt, part=part: tr.target(part, tt), t)
        elif ctl.state == "HOLD":
            car = tr.next_car(sp["slot"], sp["x"] - 0.25, sp["x"] + 0.05, t)
            if car is not None:
                tg = tr.slot_target(car, sp["slot"], t)
                if self.arm.reach(tg.x, tg.y) < 0.78:
                    self.car = car
                    ctl.assign_place(lambda tt, car=car: tr.slot_target(car, sp["slot"], tt), t)
        ctl.update(t)

    def on_event(self, name, data):
        sid, t = self.spec["id"], data["t"]
        if name == "grasped":
            self.stats["grasped"] += 1
            if self.track is not None:
                self.tracker.mark(self.track, "grasped")
                px, py, _ = self.tracker.predict(self.track, t)
                tx, ty, _ = self.world.true_pose(data["body"])      # chỉ để đo
                self.stats["pred_err_mm"].append(1000 * math.hypot(px - tx, py - ty))
        elif name == "released":
            tg = data["target"]
            chassis = find_chassis(self.world, tg.x, tg.y)
            ok, err_pos, err_yaw = (False, float("nan"), float("nan"))
            if chassis is not None and data["body"] is not None:
                ok, err_pos, err_yaw = try_attach(self.world, data["body"], chassis,
                                                  self.spec["slot"])
            self.stats["place_err_mm"].append(1000 * err_pos)
            if ok:
                self.stats["attached"] += 1
                self.tracker.attached(self.car, self.spec["slot"], self.track)
                if self.car.complete:
                    self.stats["assembled"] += 1
                    self.log(f"[t={t:6.2f}s] XE ĐỦ 6 BỘ PHẬN (track {self.car.body.id})")
            else:
                self.stats["attach_fail"] += 1
                self.tracker.mark(self.track, "missed")
            self.log(f"[t={t:6.2f}s] trạm {sid:02d} lắp {self.spec['slot']:10s} "
                     f"{'OK ' if ok else 'HỎNG'} lệch {1000 * err_pos:5.1f} mm "
                     f"{math.degrees(err_yaw):4.1f}°")
        else:
            self.stats["fail_" + name] = self.stats.get("fail_" + name, 0) + 1
            if name in ("pick_timeout",) and self.track is not None:
                self.tracker.unclaim(self.track)
            elif self.track is not None and self.track.state == "claimed":
                self.tracker.mark(self.track, "missed")
            if name in ("place_timeout", "place_out_of_reach"):
                self.car = None
            self.log(f"[t={t:6.2f}s] trạm {sid:02d} {name}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120.0)
    ap.add_argument("--speed", type=float, default=1.0)
    ap.add_argument("--slip", type=float, default=0.0)
    ap.add_argument("--no-assist", action="store_true")
    ap.add_argument("--direct", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="chỉ in tổng kết")
    ap.add_argument("--no-station-cams", action="store_true", help="tắt camera trạm (để so sánh)")
    ap.add_argument("--continuous", action="store_true",
                    help="băng chạy liên tục (mặc định: dừng khi có tay đang hạ xuống gắp / đặt)")
    ap.add_argument("--video", default=None, help="ghi video MP4 (chỉ ở chế độ GUI)")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    world = World(gui=not args.direct, seed=args.seed)
    rng = np.random.default_rng(args.seed)
    cam = Camera(C.CAM_IN_EYE, rng=rng)
    detector = OracleDetector(world.parts.get, cam, rng=rng)
    st_cams = [] if args.no_station_cams else [
        Camera(e, width=C.CAM_STATION_W, height=C.CAM_STATION_H, rng=rng)
        for e in C.CAM_STATION_EYES]
    st_dets = [OracleDetector(world.parts.get, c, rng=rng, min_pixels=15,
                              max_occlusion=C.CAM_STATION_MAX_OCCLUSION) for c in st_cams]
    odo = BeltOdometry()
    tracker = Tracker(cam, odo)
    stats = dict(grasped=0, attached=0, attach_fail=0, assembled=0,
                 pred_err_mm=[], place_err_mm=[])
    log = (lambda *a: None) if args.quiet else print
    stations = [Station(s, world, tracker, stats, not args.no_assist, log)
                for s in C.STATIONS if s["slot"] is not None]
    qc_cam = Camera(C.CAM_QC_MAIN_EYE, rng=rng)
    qc = QCStation(qc_cam, OracleDetector(world.parts.get, qc_cam, rng=rng), tracker, world)
    hud = None
    if not args.direct:
        if args.video:
            p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, args.video)
        for st in stations:
            p.addUserDebugText(f"{st.spec['id']:02d} {st.spec['slot']}",
                               [st.spec["x"], st.spec["y"] * 1.35, 0.35], [0, 0, 0], 1.2)
        p.addUserDebugText("INPUT CAMERA", [C.CAM_IN_EYE[0], 0.32, 0.25], [0.1, 0.3, 0.9], 1.2)
        p.addUserDebugText("QC", [C.CAM_QC_MAIN_EYE[0], 0.32, 0.25], [0.1, 0.6, 0.2], 1.4)
        p.addUserDebugLine([C.CAM_QC_MAIN_EYE[0], -0.25, 0.002], [C.CAM_QC_MAIN_EYE[0], 0.25, 0.002],
                           [0.1, 0.8, 0.2], 4)
    speed_now = args.speed

    wall0 = time.time()
    while world.t < args.seconds:
        world.auto_spawn(args.speed)
        v_enc = world.step(speed_now, args.slip)
        odo.step(world.t, v_enc, C.DT)
        t = world.t
        if world.step_count % CAPTURE_EVERY == 0:
            rgb, seg, t_cap = cam.capture(t)
            tracker.ingest(detector.detect(rgb, seg), t_cap)
        for k, (sc, sd) in enumerate(zip(st_cams, st_dets)):
            # so le với camera đầu vào để mỗi bước sim render tối đa 1 ảnh
            if world.step_count % C.CAM_STATION_EVERY == 12 * (k + 1):
                rgb, seg, t_cap = sc.capture(t)
                x0 = sc.eye[0]
                # chỉ sửa pose thân xe; bộ phận rời do camera đầu vào + tay tự lo
                tracker.ingest(sd.detect(rgb, seg), t_cap, cam=sc, create=False, blend=0.8,
                               classes={"body"}, keep_heading=True,
                               roi_x=(x0 - C.CAM_STATION_ROI, x0 + C.CAM_STATION_ROI))
        if world.step_count % C.CONTROL_EVERY == 0:
            for st in stations:
                st.update(t)
            # stop-and-go: băng dừng khi có tay đang hạ xuống gắp / đặt
            busy = any(st.ctl.state in C.STOP_STATES for st in stations)
            speed_now = 0.0 if (busy and not args.continuous) else args.speed
            res = qc.maybe_run(t)
            if res is not None:
                stats["qc_pass" if res.passed else "qc_fail"] = \
                    stats.get("qc_pass" if res.passed else "qc_fail", 0) + 1
                print(f"[t={t:6.2f}s] QC xe {res.car_id}: {'PASS' if res.passed else 'FAIL'}"
                      f" thiếu={res.missing} | thật đang gắn {res.gt_attached}/6")
            tracker.expire(t)
            world.remove_exited()
            if not args.direct and world.step_count % 240 == 0:
                hud = p.addUserDebugText(
                    f"Bodies {world.counters['bodies']}   Assembled {stats['assembled']}   "
                    f"QC passed {stats.get('qc_pass', 0)}   Surplus {world.counters['surplus']}   "
                    f"Belt {'STOP' if speed_now == 0 else 'RUN'}",
                    [0.0, -0.75, 0.5], [0, 0, 0], 1.5,
                    **({"replaceItemUniqueId": hud} if hud is not None else {}))
        if not args.direct:
            ahead = world.t - (time.time() - wall0)
            if ahead > 0:
                time.sleep(ahead)

    if not args.direct and args.video:
        p.disconnect()                   # đóng file video
    cars = list(tracker.cars.values())
    filled = [sum(v is not None for v in c.slots.values()) for c in cars]
    print("\n=== Tổng kết", f"{args.seconds:.0f} s sim ===")
    print("Bộ đếm world:", world.counters)
    print({k: v for k, v in stats.items() if not isinstance(v, list)})
    if stats["pred_err_mm"]:
        e = stats["pred_err_mm"]
        print(f"Sai số dự đoán lúc gắp: TB {statistics.mean(e):.1f} mm, max {max(e):.1f} mm (n={len(e)})")
    if stats["place_err_mm"]:
        e = [x for x in stats["place_err_mm"] if x == x]
        if e:
            print(f"Lệch khi thả vào slot: TB {statistics.mean(e):.1f} mm, max {max(e):.1f} mm (n={len(e)})")
    print(f"Xe còn trong map: {len(cars)}, số slot đã lắp mỗi xe: {filled}")
    agree = sum(1 for r in qc.results if r.passed == (r.gt_attached == 6))
    if qc.results:
        print(f"QC: {len(qc.results)} xe kiểm, PASS {sum(r.passed for r in qc.results)}, "
              f"khớp ground truth {agree}/{len(qc.results)}")


if __name__ == "__main__":
    main()
