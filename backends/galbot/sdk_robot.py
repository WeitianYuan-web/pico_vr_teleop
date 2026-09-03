"""Thin Galbot SDK wrapper: WBC EE stream (1.8+) or Motion IK + joint servo (1.7).

Galbot SDK must be on PYTHONPATH / LD_LIBRARY_PATH before this process starts
(see scripts/run_vr_teleop_galbot.sh). Import is lazy so --dry-run works
without a full SDK install.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from config import (
    ARM_CHAINS,
    ARM_JOINT_GROUPS,
    DEFAULT_EMBOSA_CONFIG,
    DEFAULT_TELEOP_MAX_RAD_S,
    DEFAULT_URDF_PATH,
    EE_FRAMES,
    SUPPORT_JOINT_NAMES,
    WBC_POSE_KEYS,
)


@dataclass
class EndEffectorPose:
    x: float
    y: float
    z: float
    qw: float
    qx: float
    qy: float
    qz: float

    def copy(self) -> "EndEffectorPose":
        return EndEffectorPose(
            x=self.x, y=self.y, z=self.z, qw=self.qw, qx=self.qx, qy=self.qy, qz=self.qz
        )

    def as_xyzw(self) -> list[float]:
        return [self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw]


def pose_from_xyzw(raw: list[float] | None) -> EndEffectorPose | None:
    if not raw or len(raw) < 7:
        return None
    x, y, z, qx, qy, qz, qw = (float(v) for v in raw[:7])
    n = float(np.sqrt(qw * qw + qx * qx + qy * qy + qz * qz))
    if n > 1e-12:
        qw, qx, qy, qz = qw / n, qx / n, qy / n, qz / n
    else:
        qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
    return EndEffectorPose(x=x, y=y, z=z, qw=qw, qx=qx, qy=qy, qz=qz)


def _status_name(status: Any) -> str:
    name = getattr(status, "name", None)
    if isinstance(name, str) and name:
        return name
    return str(status)


def _status_ok(status: Any, success_cls: Any | None) -> bool:
    if success_cls is not None and status == success_cls:
        return True
    return _status_name(status) in ("SUCCESS", "IN_PROGRESS")


def _pose_close(
    a: EndEffectorPose,
    b: EndEffectorPose,
    *,
    pos_m: float = 0.0015,
    quat_dot: float = 0.9995,
) -> bool:
    dx = a.x - b.x
    dy = a.y - b.y
    dz = a.z - b.z
    if dx * dx + dy * dy + dz * dz > pos_m * pos_m:
        return False
    dot = abs(a.qw * b.qw + a.qx * b.qx + a.qy * b.qy + a.qz * b.qz)
    return dot >= quat_dot


class GalbotSdkRobot:
    """Live GalbotRobot handle, or a no-hardware stub when dry_run=True."""

    def __init__(
        self,
        *,
        dry_run: bool = False,
        ee_wait_s: float = 5.0,
        teleop_max_rad_s: float = DEFAULT_TELEOP_MAX_RAD_S,
    ) -> None:
        self.dry_run = bool(dry_run)
        self.ee_wait_s = max(0.0, float(ee_wait_s))
        self.teleop_max_rad_s = max(0.05, float(teleop_max_rad_s))
        self.robot: Any = None
        self.motion: Any = None
        self._use_wbc: bool = False
        self._MotionStatus: Any = None
        self._ControlStatus: Any = None
        self.latest_ee: dict[str, EndEffectorPose | None] = {"left": None, "right": None}
        self.arm_joints: dict[str, list[float] | None] = {"left": None, "right": None}
        self.arm_velocities: dict[str, list[float] | None] = {"left": None, "right": None}
        self._last_cmd_error_t: float = 0.0
        self._last_cmd_error: str = ""
        self._last_wbc_raw: dict[str, Any] = {}
        self._last_motion_status: dict[str, str] = {}
        self._ik_lock = threading.Lock()
        self._ik_cv = threading.Condition(self._ik_lock)
        self._ik_stop = threading.Event()
        self._ik_thread: threading.Thread | None = None
        self._ik_seq: int = 0
        self._ik_targets: dict[str, EndEffectorPose] = {}
        self._servo_q: dict[str, list[float] | None] = {"left": None, "right": None}
        self._cmd_q: dict[str, list[float] | None] = {"left": None, "right": None}
        self._motion_lock = threading.Lock()
        self.local_ik: Any = None
        self._joint_batch_ok: bool = True
        self._urdf_path = os.environ.get("GALBOT_URDF") or DEFAULT_URDF_PATH
        self._support_warn: bool = False
        self._stream_stop = threading.Event()
        self._stream_thread: threading.Thread | None = None
        self._stream_sides: set[str] = set()
        self._cmd_vel: dict[str, list[float] | None] = {"left": None, "right": None}
        self.ik_track_err_m: dict[str, float] = {"left": 0.0, "right": 0.0}

    def init(self) -> None:
        if self.dry_run:
            self.latest_ee["left"] = EndEffectorPose(0.35, 0.30, 0.40, 1.0, 0.0, 0.0, 0.0)
            self.latest_ee["right"] = EndEffectorPose(0.35, -0.30, 0.40, 1.0, 0.0, 0.0, 0.0)
            self.arm_joints["left"] = [0.0] * 7
            self.arm_joints["right"] = [0.0] * 7
            self.arm_velocities["left"] = [0.0] * 7
            self.arm_velocities["right"] = [0.0] * 7
            print("[Galbot] dry-run：不连接机器人，使用占位 EE")
            return

        self._warn_embosa_config()
        try:
            from galbot_sdk.g1 import ControlStatus, GalbotRobot
        except ImportError as exc:
            raise RuntimeError(
                "无法 import galbot_sdk.g1。GBS 1.15 请装 SDK 1.7.3：\n"
                "  cd third_party/GalbotSDK-V1.7.3 && sudo ./install.sh "
                "--platform linux-x86_64-gcc940 --install-dir /opt/galbot-1.7.3 -y\n"
                "  source /opt/galbot-1.7.3/galbot_sdk/linux-x86_64-gcc940/setup.sh\n"
                "或设置 GALBOT_HOME 后用 ./scripts/run_vr_teleop_galbot.sh。\n"
                f"原始错误: {exc}"
            ) from exc

        self._ControlStatus = ControlStatus
        robot = GalbotRobot()
        if not robot.init():
            raise RuntimeError(
                "GalbotRobot.init() 失败。常见原因：\n"
                "  1) 未配置 /data/config/embosa_ip_config.json（PC/XCU/HPU）\n"
                "  2) 本机没有与机器人同网段的 IP（默认 PC 192.168.1.99）\n"
                "  3) 遥操作进程套了本机 FastDDS isolation（必须 unset FASTRTPS_*）\n"
                "  4) SDK 与机上 GBS 不匹配（1.7.x↔GBS 1.15；1.9.x↔GBS 1.17）"
            )
        self.robot = robot
        self._use_wbc = hasattr(robot, "get_wbc_end_effector_poses") and hasattr(
            robot, "set_end_effector_command"
        )
        if not self._use_wbc:
            from galbot_sdk.g1 import GalbotMotion, MotionStatus

            self._MotionStatus = MotionStatus
            self.motion = GalbotMotion()
            if not self.motion.init():
                raise RuntimeError("GalbotMotion.init() 失败（1.7 规划器末端需要它）")
            print(
                "[Galbot] 当前 SDK 无 WBC 流式末端（GBS 1.15 / SDK ≤1.7）。"
                "笛卡尔规划器执行会 FAULT；遥操作优先本地 pinocchio IK + "
                "set_joint_commands(t=0)。"
            )
            self._try_load_local_ik()
            # 规划器刚 init 时 get_end_effector_pose 常仍空；官方示例也会先 sleep。
            time.sleep(min(1.0, self.ee_wait_s))
        # Official examples sleep ~2s after init before state is populated.
        # 不能因为关节先到就提前结束：setup() 仍要用左右 EE 当 home。
        deadline = time.time() + self.ee_wait_s
        attempt = 0
        while True:
            attempt += 1
            self.refresh()
            ready = (
                self.latest_ee["left"] is not None and self.latest_ee["right"] is not None
            )
            if ready:
                break
            if time.time() >= deadline:
                break
            if attempt == 1:
                what = "WBC EE" if self._use_wbc else "Motion EE"
                print(
                    f"[Galbot] init 成功，等待 {what}（最多 {self.ee_wait_s:.0f}s）..."
                )
            time.sleep(0.2)
        if self.latest_ee["left"] is None and self.latest_ee["right"] is None:
            if not (self.arm_joints["left"] or self.arm_joints["right"]):
                raise RuntimeError(self._empty_ee_error())
            print(
                "[Galbot] 警告: 关节有数但 EE FK 仍空"
                f"（motion_status={getattr(self, '_last_motion_status', {})}），"
                "setup 若仍要 EE 会失败"
            )
        self._seed_servo_from_joints()
        self.sync_local_ik()
        if not self._use_wbc and self.local_ik is None:
            self._start_ik_worker()
        self._log_fk_compare()
        if self.local_ik is not None:
            self._start_joint_streamer()
        print(
            "[Galbot] SDK 已连接: "
            f"mode={'wbc' if self._use_wbc else ('local-ik' if self.local_ik else 'motion-ik-servo')} "
            f"L={'ok' if self.latest_ee['left'] is not None else 'none'} "
            f"R={'ok' if self.latest_ee['right'] is not None else 'none'} "
            f"(waited {attempt} polls)"
        )

    @property
    def uses_wbc(self) -> bool:
        return bool(self._use_wbc)

    def shutdown(self) -> None:
        self._stop_joint_streamer()
        self._stop_ik_worker()
        if self.robot is None:
            return
        try:
            if self._use_wbc:
                self.robot.clear_end_effector_command()
        except Exception:
            pass
        try:
            self.robot.request_shutdown()
            self.robot.wait_for_shutdown()
        except KeyboardInterrupt:
            pass
        except Exception:
            pass
        try:
            self.robot.destroy()
        except Exception:
            pass
        self.robot = None
        if self.motion is not None:
            try:
                destroy = getattr(self.motion, "destroy", None)
                if callable(destroy):
                    destroy()
            except Exception:
                pass
            self.motion = None

    def refresh(self, *, ee: bool = True, joints: bool = True) -> None:
        if self.dry_run or self.robot is None:
            return
        if ee:
            self._refresh_ee()
        if joints:
            self._refresh_joints()
            self.sync_local_ik()

    def _refresh_ee(self) -> None:
        if self._use_wbc:
            assert self.robot is not None
            poses = self.robot.get_wbc_end_effector_poses() or {}
            self._last_wbc_raw = {
                str(k): (list(v) if v is not None else []) for k, v in dict(poses).items()
            }
            for side, key in WBC_POSE_KEYS.items():
                self.latest_ee[side] = pose_from_xyzw(poses.get(key))
            return
        if self.motion is None:
            return
        with self._motion_lock:
            for side, frame in EE_FRAMES.items():
                try:
                    status, pose = self.motion.get_end_effector_pose(
                        end_effector_frame=frame,
                        reference_frame="base_link",
                    )
                except TypeError:
                    try:
                        status, pose = self.motion.get_end_effector_pose(frame, "base_link")
                    except Exception:
                        status, pose = None, None
                except Exception:
                    status, pose = None, None
                self._last_motion_status[side] = _status_name(status)
                if _status_ok(status, self._MotionStatus.SUCCESS if self._MotionStatus else None):
                    self.latest_ee[side] = pose_from_xyzw(
                        list(pose) if pose is not None else None
                    )

    def _refresh_joints(self) -> None:
        assert self.robot is not None
        for side, group in ARM_JOINT_GROUPS.items():
            try:
                states = self.robot.get_joint_states(joint_groups=[group]) or []
            except TypeError:
                try:
                    states = self.robot.get_joint_states([group], []) or []
                except Exception:
                    states = []
            except Exception:
                states = []
            if not states:
                self.arm_joints[side] = None
                self.arm_velocities[side] = None
                continue
            self.arm_joints[side] = [float(getattr(s, "position", 0.0)) for s in states]
            self.arm_velocities[side] = [float(getattr(s, "velocity", 0.0)) for s in states]

    def _try_load_local_ik(self) -> None:
        path = self._urdf_path
        if not os.path.isfile(path):
            print(
                f"[Galbot] 无本地 URDF ({path})，遥操作仍走 HPU IK（会卡）。"
                "从 HPU 拷贝 galbot_one_golf_cali.urdf 到 backends/galbot/assets/"
            )
            return
        try:
            from arm_ik import GalbotLocalArmIK

            self.local_ik = GalbotLocalArmIK(path)
        except Exception as exc:
            self.local_ik = None
            print(f"[Galbot] 本地 IK 加载失败，回退 HPU IK: {exc}")

    def _refresh_support_joints(self) -> None:
        if self.robot is None or self.local_ik is None:
            return
        names = list(SUPPORT_JOINT_NAMES)
        states: list[Any] = []
        try:
            states = list(self.robot.get_joint_states([], names) or [])
        except Exception:
            states = []
        if len(states) < len(names):
            try:
                states = list(self.robot.get_joint_states(["leg", "head"], []) or [])
            except Exception:
                states = []
        if not states:
            if not self._support_warn:
                self._support_warn = True
                print("[Galbot] 警告: 读不到腿/头关节，本地 FK 会偏，可能禁用本地 IK")
            return
        by_name: dict[str, float] = {}
        for st in states:
            n = getattr(st, "name", None) or getattr(st, "joint_name", None)
            if n:
                by_name[str(n)] = float(getattr(st, "position", 0.0))
        if by_name:
            self.local_ik.set_named_positions(list(by_name.keys()), list(by_name.values()))
            return
        vals = [float(getattr(s, "position", 0.0)) for s in states]
        n = min(len(vals), len(names))
        self.local_ik.set_named_positions(names[:n], vals[:n])

    def sync_local_ik(self) -> None:
        if self.local_ik is None:
            return
        self._refresh_support_joints()
        self.local_ik.set_arm("left", self.arm_joints.get("left"))
        self.local_ik.set_arm("right", self.arm_joints.get("right"))

    def fk_ee(self, side: str) -> EndEffectorPose | None:
        if self.local_ik is None:
            ee = self.latest_ee.get(side)
            return ee.copy() if ee is not None else None
        xyz, quat = self.local_ik.fk_xyz_wxyz(side)
        return EndEffectorPose(
            x=float(xyz[0]),
            y=float(xyz[1]),
            z=float(xyz[2]),
            qw=float(quat[0]),
            qx=float(quat[1]),
            qy=float(quat[2]),
            qz=float(quat[3]),
        )

    def _log_fk_compare(self) -> None:
        if self.local_ik is None:
            return
        self.sync_local_ik()
        bad = False
        for side in ("left", "right"):
            meas = self.latest_ee.get(side)
            if meas is None:
                continue
            xyz, _ = self.local_ik.fk_xyz_wxyz(side)
            err = float(
                np.linalg.norm(xyz - np.array([meas.x, meas.y, meas.z], dtype=float))
            )
            print(
                f"[Galbot] {side} 本地FK [{xyz[0]:+.3f},{xyz[1]:+.3f},{xyz[2]:+.3f}] "
                f"vs Motion [{meas.x:+.3f},{meas.y:+.3f},{meas.z:+.3f}] "
                f"Δ{err * 1000:.0f} mm"
            )
            if err > 0.05:
                print(
                    f"[Galbot] 警告: {side} 本地FK 与 Motion 差 {err * 1000:.0f} mm，"
                    "腿/头关节可能没读到；禁用本地 IK，回退 HPU"
                )
                bad = True
        if bad:
            self._stop_joint_streamer()
            self.local_ik = None
            if not self._use_wbc:
                self._start_ik_worker()
                print("[Galbot] 已回退 mode=motion-ik-servo")

    def send_ee_commands(self, desired: dict[str, EndEffectorPose]) -> bool:
        """Send dual-arm Cartesian targets (WBC stream or 1.7 IK+joint servo)."""
        if not desired:
            return True
        if self.dry_run:
            return True
        if self.robot is None:
            return False
        if self._use_wbc:
            return self._send_wbc(desired)
        return self._send_motion_ee(desired)

    def _send_wbc(self, desired: dict[str, EndEffectorPose]) -> bool:
        poses: list[list[float]] = []
        frames: list[str] = []
        for side in ("left", "right"):
            pose = desired.get(side)
            if pose is None:
                continue
            poses.append(pose.as_xyzw())
            frames.append(EE_FRAMES[side])
        if not poses:
            return True
        try:
            status = self.robot.set_end_effector_command(
                poses=poses,
                end_effector_frames=frames,
            )
        except Exception as exc:
            self._log_cmd_error(f"set_end_effector_command 异常: {exc}")
            return False
        if not _status_ok(status, self._ControlStatus.SUCCESS if self._ControlStatus else None):
            self._log_cmd_error(f"set_end_effector_command 失败: {_status_name(status)}")
            return False
        return True

    def _start_ik_worker(self) -> None:
        if self.dry_run or self._ik_thread is not None:
            return
        self._ik_stop.clear()
        self._ik_thread = threading.Thread(
            target=self._ik_worker,
            name="galbot-ik",
            daemon=True,
        )
        self._ik_thread.start()

    def _stop_ik_worker(self) -> None:
        self._ik_stop.set()
        with self._ik_cv:
            self._ik_cv.notify_all()
        thread = self._ik_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._ik_thread = None

    def _seed_servo_from_joints(self) -> None:
        with self._ik_lock:
            for side in ("left", "right"):
                q = self.arm_joints.get(side)
                if q and len(q) >= 7:
                    self._servo_q[side] = [float(v) for v in q[:7]]

    def _ik_worker(self) -> None:
        last_seq = -1
        last_solved: dict[str, EndEffectorPose] = {}
        while not self._ik_stop.is_set():
            with self._ik_cv:
                while not self._ik_stop.is_set() and self._ik_seq == last_seq:
                    self._ik_cv.wait(timeout=0.05)
                if self._ik_stop.is_set():
                    return
                snapshot = {side: pose.copy() for side, pose in self._ik_targets.items()}
                last_seq = self._ik_seq
            for side in ("left", "right"):
                if self._ik_stop.is_set():
                    return
                pose = snapshot.get(side)
                if pose is None:
                    continue
                prev = last_solved.get(side)
                if prev is not None and _pose_close(prev, pose):
                    continue
                q = self._ik_arm(side, pose)
                if q is None:
                    continue
                with self._ik_lock:
                    self._servo_q[side] = q
                last_solved[side] = pose

    def _send_motion_ee(self, desired: dict[str, EndEffectorPose]) -> bool:
        if self.robot is None:
            return False
        if self.local_ik is not None:
            qs: dict[str, list[float]] = {}
            for side in ("left", "right"):
                pose = desired.get(side)
                if pose is None:
                    continue
                q = self.local_ik.solve(
                    side,
                    np.array([pose.x, pose.y, pose.z], dtype=float),
                    np.array([pose.qw, pose.qx, pose.qy, pose.qz], dtype=float),
                )
                qs[side] = q
                self.ik_track_err_m[side] = float(
                    getattr(self.local_ik, "last_pos_err_m", 0.0)
                )
            self._post_stream_targets(qs)
            return True

        if self.motion is None:
            return False
        with self._ik_cv:
            self._ik_targets = {side: pose.copy() for side, pose in desired.items()}
            self._ik_seq += 1
            self._ik_cv.notify()
        qs = {}
        max_step = 0.04
        for side in ("left", "right"):
            if desired.get(side) is None:
                continue
            with self._ik_lock:
                target = self._servo_q.get(side)
            current = self._cmd_q.get(side) or self.arm_joints.get(side)
            if not target or not current or len(target) < 7 or len(current) < 7:
                if target and len(target) >= 7:
                    qs[side] = list(target[:7])
                    self._cmd_q[side] = qs[side]
                continue
            out = []
            for a, b in zip(current[:7], target[:7]):
                d = float(b) - float(a)
                if abs(d) > max_step:
                    b = float(a) + max_step * (1.0 if d > 0 else -1.0)
                out.append(float(b))
            qs[side] = out
            self._cmd_q[side] = out
        return self._send_streaming_joints(qs)

    def _post_stream_targets(self, qs: dict[str, list[float]]) -> None:
        with self._ik_lock:
            prev = set(self._stream_sides)
            self._stream_sides = {side for side in qs if qs.get(side)}
            for side, q in qs.items():
                if not q or len(q) < 7:
                    continue
                clamped = q
                if self.local_ik is not None:
                    clamped = self.local_ik.clamp_arm(side, q) or q
                self._servo_q[side] = [float(v) for v in clamped[:7]]
                if side not in prev or self._cmd_q.get(side) is None:
                    seed = self.arm_joints.get(side) or q
                    self._cmd_q[side] = [float(v) for v in seed[:7]]
                    self._cmd_vel[side] = [0.0] * 7

    def _start_joint_streamer(self) -> None:
        if self.dry_run or self._stream_thread is not None:
            return
        self._stream_stop.clear()
        self._stream_thread = threading.Thread(
            target=self._joint_stream_loop,
            name="galbot-joint-stream",
            daemon=True,
        )
        self._stream_thread.start()
        print(
            f"[Galbot] 关节流: 100Hz EMA+限速 {self.teleop_max_rad_s:.2f} rad/s，"
            "控制环不再等 set_joint_commands"
        )

    def _stop_joint_streamer(self) -> None:
        self._stream_stop.set()
        thread = self._stream_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=0.8)
        self._stream_thread = None
        self._stream_sides = set()

    def _joint_stream_loop(self) -> None:
        period = 0.01
        tau_s = 0.035
        max_rad_s = self.teleop_max_rad_s
        last_t = time.time()
        while not self._stream_stop.is_set():
            t0 = time.time()
            dt = min(0.05, max(1e-3, t0 - last_t))
            last_t = t0
            with self._ik_lock:
                sides = [s for s in ("left", "right") if s in self._stream_sides]
                targets = {
                    s: list(self._servo_q[s])
                    for s in sides
                    if self._servo_q.get(s) and len(self._servo_q[s]) >= 7
                }
                cmds = {s: list(self._cmd_q[s]) if self._cmd_q.get(s) else None for s in sides}
            qs: dict[str, list[float]] = {}
            vels: dict[str, list[float]] = {}
            alpha = 1.0 - float(np.exp(-dt / tau_s))
            max_dq = max_rad_s * dt
            for side, tgt in targets.items():
                cur = cmds.get(side) or list(tgt)
                if self.local_ik is not None:
                    tgt = self.local_ik.clamp_arm(side, tgt) or tgt
                    cur = self.local_ik.clamp_arm(side, cur) or cur
                out = []
                vel = []
                for a, b in zip(cur[:7], tgt[:7]):
                    x = float(a) + alpha * (float(b) - float(a))
                    step = float(np.clip(x - float(a), -max_dq, max_dq))
                    nxt = float(a) + step
                    out.append(nxt)
                    vel.append(0.0)
                qs[side] = out
                vels[side] = vel
            if qs:
                with self._ik_lock:
                    for side, q in qs.items():
                        self._cmd_q[side] = q
                        self._cmd_vel[side] = vels[side]
                self._send_streaming_joints(qs, velocities=vels)
            elapsed = time.time() - t0
            remain = period - elapsed
            if remain > 0.0 and self._stream_stop.wait(remain):
                break

    def _send_streaming_joints(
        self,
        qs: dict[str, list[float]],
        *,
        velocities: dict[str, list[float]] | None = None,
    ) -> bool:
        if self.robot is None or not qs:
            return True
        from galbot_sdk.g1 import JointCommand

        def _cmds_for(side: str, q: list[float]) -> list[Any]:
            q = (
                self.local_ik.clamp_arm(side, q)
                if self.local_ik is not None
                else q
            ) or q
            out = []
            for val in q[:7]:
                cmd = JointCommand()
                cmd.position = float(val)
                cmd.velocity = 0.0
                out.append(cmd)
            return out

        sides = [s for s in ("left", "right") if qs.get(s)]
        if not sides:
            return True
        if self._joint_batch_ok and len(sides) >= 1:
            cmds: list[Any] = []
            groups: list[str] = []
            for side in sides:
                groups.append(ARM_JOINT_GROUPS[side])
                cmds.extend(_cmds_for(side, qs[side]))
            try:
                status = self.robot.set_joint_commands(cmds, groups, [], 0.0)
            except Exception as exc:
                self._joint_batch_ok = False
                self._log_cmd_error(f"set_joint_commands(batch) 异常，改分臂发送: {exc}")
            else:
                if _status_ok(status, self._ControlStatus.SUCCESS if self._ControlStatus else None):
                    return True
                self._joint_batch_ok = False
                self._log_cmd_error(
                    f"set_joint_commands(batch) 失败: {_status_name(status)}，改分臂发送"
                )
        ok = True
        for side in sides:
            if not self._send_arm_joints(
                side, qs[side], blocking=False, speed_rad_s=0.8, timeout_s=0.5
            ):
                ok = False
        if not ok:
            self._recover_after_joint_fault(sides)
        return ok

    def _recover_after_joint_fault(self, sides: list[str]) -> None:
        """Stop slamming a bad target after FAULT; re-seed from measured joints."""
        try:
            self.refresh()
        except Exception:
            return
        self._seed_servo_from_joints()
        self.sync_local_ik()
        with self._ik_lock:
            for side in sides:
                q = self.arm_joints.get(side)
                if not q or len(q) < 7:
                    continue
                measured = [float(v) for v in q[:7]]
                self._cmd_q[side] = list(measured)
                self._cmd_vel[side] = [0.0] * 7
                self._servo_q[side] = list(measured)
        self._log_cmd_error("关节指令 FAULT，已用实测关节重新种子，停止继续顶限位")

    def _ik_arm(self, side: str, pose: EndEffectorPose) -> list[float] | None:
        if self.motion is None:
            return None
        with self._ik_lock:
            seed = self._servo_q.get(side) or self.arm_joints.get(side)
            seed = list(seed[:7]) if seed and len(seed) >= 7 else None
        if not seed:
            return None
        chain = ARM_CHAINS[side]
        try:
            with self._motion_lock:
                status, ik = self.motion.inverse_kinematics(
                    target_pose=pose.as_xyzw(),
                    chain_names=[chain],
                    target_frame=EE_FRAMES[side],
                    reference_frame="base_link",
                    initial_joint_positions={chain: seed},
                    enable_collision_check=False,
                )
        except Exception as exc:
            self._log_cmd_error(f"inverse_kinematics({side}) 异常: {exc}")
            return None
        if not _status_ok(status, self._MotionStatus.SUCCESS if self._MotionStatus else None):
            self._log_cmd_error(f"inverse_kinematics({side}) 失败: {_status_name(status)}")
            return None
        sol = None
        if isinstance(ik, dict):
            sol = ik.get(chain) or (next(iter(ik.values())) if ik else None)
        if not sol or len(sol) < 7:
            self._log_cmd_error(f"inverse_kinematics({side}) 解为空")
            return None
        return [float(v) for v in list(sol)[:7]]

    def _send_arm_joints(
        self,
        side: str,
        q: list[float],
        *,
        blocking: bool,
        speed_rad_s: float,
        timeout_s: float,
    ) -> bool:
        if self.robot is None:
            return False
        group = ARM_JOINT_GROUPS[side]
        target = [float(v) for v in q[:7]]
        if blocking:
            try:
                status = self.robot.set_joint_positions(
                    target,
                    joint_groups=[group],
                    is_blocking=True,
                    speed_rad_s=float(speed_rad_s),
                    timeout_s=float(timeout_s),
                )
            except TypeError:
                status = self.robot.set_joint_positions(
                    target, [group], [], True, float(speed_rad_s), float(timeout_s)
                )
            except Exception as exc:
                self._log_cmd_error(f"set_joint_positions({side}) 异常: {exc}")
                return False
        else:
            try:
                from galbot_sdk.g1 import JointCommand

                cmds = []
                for val in target:
                    cmd = JointCommand()
                    cmd.position = float(val)
                    cmds.append(cmd)
                status = self.robot.set_joint_commands(cmds, [group], [], 0.0)
            except Exception as exc:
                self._log_cmd_error(f"set_joint_commands({side}) 异常: {exc}")
                return False
        if not _status_ok(status, self._ControlStatus.SUCCESS if self._ControlStatus else None):
            self._log_cmd_error(
                f"{'set_joint_positions' if blocking else 'set_joint_commands'}({side}) "
                f"失败: {_status_name(status)}"
            )
            return False
        return True

    def move_arm_joints(
        self,
        *,
        left: list[float] | None,
        right: list[float] | None,
        speed_rad_s: float = 0.2,
        timeout_s: float = 20.0,
    ) -> bool:
        """Blocking joint-space move for arms only (not legs/head)."""
        if self.dry_run:
            print("[Galbot] dry-run：跳过关节回初始位")
            return True
        if self.robot is None:
            return False
        # 跟手关节流会抢 set_joint_commands，回位前先停一拍。
        with self._ik_lock:
            self._stream_sides.clear()
        time.sleep(0.03)
        ok = True
        for group, q in (("left_arm", left), ("right_arm", right)):
            if q is None:
                continue
            target = [float(v) for v in q[:7]]
            if len(target) < 7:
                self._log_cmd_error(f"{group} 初始关节不是 7 个数: {len(q)}")
                ok = False
                continue
            try:
                status = self.robot.set_joint_positions(
                    target,
                    joint_groups=[group],
                    is_blocking=True,
                    speed_rad_s=float(speed_rad_s),
                    timeout_s=float(timeout_s),
                )
            except TypeError:
                try:
                    status = self.robot.set_joint_positions(
                        target, [group], [], True, float(speed_rad_s), float(timeout_s)
                    )
                except Exception as exc:
                    self._log_cmd_error(f"set_joint_positions({group}) 异常: {exc}")
                    ok = False
                    continue
            except Exception as exc:
                self._log_cmd_error(f"set_joint_positions({group}) 异常: {exc}")
                ok = False
                continue
            if not _status_ok(status, self._ControlStatus.SUCCESS if self._ControlStatus else None):
                self._log_cmd_error(
                    f"set_joint_positions({group}) 失败: {_status_name(status)}"
                )
                ok = False
            else:
                print(f"[Galbot] {group} 已到初始关节 ({_status_name(status)})")
            self.refresh()
        self._seed_servo_from_joints()
        self.sync_local_ik()
        return ok

    def move_ee_poses(
        self,
        desired: dict[str, EndEffectorPose],
        *,
        timeout_s: float = 20.0,
    ) -> bool:
        """Blocking Cartesian move (1.7 planner or 1.8+ WBC)."""
        if self.dry_run:
            print("[Galbot] dry-run：跳过笛卡尔回初始位")
            return True
        if self.robot is None or not desired:
            return False
        if self._use_wbc:
            t0 = time.time()
            last = False
            while time.time() - t0 < float(timeout_s):
                last = self._send_wbc(desired)
                self.refresh()
                arrived = True
                for side, goal in desired.items():
                    now = self.latest_ee.get(side)
                    if now is None:
                        arrived = False
                        break
                    err = ((now.x - goal.x) ** 2 + (now.y - goal.y) ** 2 + (now.z - goal.z) ** 2) ** 0.5
                    if err > 0.03:
                        arrived = False
                        break
                if arrived:
                    return True
                time.sleep(0.05)
            return last
        if self.motion is None:
            return False
        ok = True
        self._refresh_joints()
        for side, pose in desired.items():
            q = self._ik_arm(side, pose)
            if q is None:
                ok = False
                continue
            if self._send_arm_joints(
                side,
                q,
                blocking=True,
                speed_rad_s=0.2,
                timeout_s=float(timeout_s),
            ):
                with self._ik_lock:
                    self._servo_q[side] = list(q[:7])
                print(f"[Galbot] {side} IK+关节回位 SUCCESS")
            else:
                ok = False
            self.refresh()
        self._seed_servo_from_joints()
        self.sync_local_ik()
        self._log_fk_compare()
        return ok

    def _empty_ee_error(self) -> str:
        if self._use_wbc:
            parts = [
                "GalbotRobot.init() 已成功，但 WBC 末端位姿仍为空。",
                "官方示例在 init 后还要等 ~2s 才读 get_wbc_end_effector_poses；",
                "空列表表示 SDK 还没收到机上 WBC 反馈（不是 import 失败）。",
                self._format_wbc_raw(self._last_wbc_raw),
                self._format_joint_diag(),
                self._format_controller_diag(),
                "排查：",
                "  1) 机上运控/WBC 是否已起来（仅 ping XCU/HPU 不够）",
                "  2) SDK 与 GBS 版本（1.7.x↔GBS 1.15；1.9.x↔GBS 1.17）",
                "  3) 加长等待: --ee-wait-s 10",
                "  4) 先跑官方 examples/g1/python/galbot_robot/set_end_effector_commands.py 对照",
            ]
            return "\n".join(parts)
        parts = [
            "GalbotRobot.init() 已成功，但关节状态和 Motion EE 都为空。",
            "能读到 active controller、读不到关节：不是再等几秒，是 XCU 臂反馈没接到 WBC。",
            "HPU robot_state_publish 订的 singorix/wbcs/sensor 会是 0 Hz。",
            self._format_joint_diag(),
            self._format_controller_diag(),
            "排查：",
            "  1) 先重启 XCU 运控（singorix_wbcs_main / 平板运控重启），不必先整机重启",
            "  2) 若 XCU 上 armL_report / armR_report 仍 unmatched，再重启整机",
            "  3) 本机必须是 SDK 1.7.x（GBS 1.15）；1.9 对 1.15 也会空状态",
            "  4) 对照官方 examples/g1/python/galbot_robot/get_joint_states.py",
        ]
        return "\n".join(parts)

    @staticmethod
    def _format_wbc_raw(raw: dict[str, Any]) -> str:
        if not raw:
            return "  WBC dict: <empty>"
        lines = ["  WBC dict:"]
        for key, val in raw.items():
            n = len(val) if isinstance(val, list) else -1
            preview = ""
            if isinstance(val, list) and n >= 7:
                preview = " " + ", ".join(f"{float(v):+.3f}" for v in val[:7])
            lines.append(f"    {key}: n={n}{preview}")
        return "\n".join(lines)

    def _format_joint_diag(self) -> str:
        lines = ["  关节:"]
        for side in ("left", "right"):
            joints = self.arm_joints.get(side)
            if joints:
                preview = ", ".join(f"{v:+.3f}" for v in joints[:4])
                lines.append(f"    {side}_arm: n={len(joints)} [{preview}, ...]")
            else:
                lines.append(f"    {side}_arm: <empty>")
        if all(not self.arm_joints.get(s) for s in ("left", "right")):
            lines.append("    → 关节也空：更像 Embosa 状态还没到，不只是 WBC EE。")
        else:
            lines.append("    → 关节有数、EE 为空：机器人在线，但 WBC EE 通道没出数。")
        return "\n".join(lines)

    def _format_controller_diag(self) -> str:
        if self.robot is None:
            return ""
        lines = ["  active controller:"]
        for group in ("left_arm", "right_arm"):
            try:
                name = self.robot.get_active_controller(group)
            except Exception as exc:
                name = f"<err {exc}>"
            lines.append(f"    {group}: {name}")
        return "\n".join(lines)

    def _log_cmd_error(self, msg: str) -> None:
        now = time.time()
        if msg == self._last_cmd_error and now - self._last_cmd_error_t < 1.0:
            return
        self._last_cmd_error = msg
        self._last_cmd_error_t = now
        print(f"\n[Galbot] {msg}")

    @staticmethod
    def _warn_embosa_config() -> None:
        path = os.environ.get("GALBOT_EMBOSA_CONFIG", DEFAULT_EMBOSA_CONFIG)
        if os.path.isfile(path):
            return
        print(
            f"[Galbot] 警告: 未找到 Embosa 配置 {path}。\n"
            "  SDK 默认从 /data/config/embosa_ip_config.json 读 PC/XCU/HPU。\n"
            "  在 SDK 目录执行: sudo ./configure_embosa_ip.sh\n"
            "  或: sudo mkdir -p /data/config && "
            "sudo cp -n third_party/GalbotSDK-V1.7.3/config/* /data/config/"
        )
