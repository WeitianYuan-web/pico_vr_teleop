#!/usr/bin/env python3
"""Robot-side UDP → /endposetarget_L|R bridge (run under Humble + XARM)."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
import time
from dataclasses import dataclass
from typing import Literal

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "../.."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from config import (  # noqa: E402
    DEFAULT_FROM_FRAME,
    DEFAULT_HOME_JOINT_DURATION_S,
    DEFAULT_TO_FRAME_LEFT,
    DEFAULT_TO_FRAME_RIGHT,
    DEFAULT_UDP_PORT,
    HOME_Q_LEFT,
    HOME_Q_RIGHT,
)
from ros_endpose import TianyiRosEndpose  # noqa: E402
from robot_status import RobotStatusMonitor  # noqa: E402
from udp_protocol import decode_pose_packet  # noqa: E402

Side = Literal["left", "right"]


@dataclass
class _BridgeSideState:
    active: bool = False
    last_active_rx: float = 0.0
    hold_pos: np.ndarray | None = None
    hold_quat: np.ndarray | None = None


class TeleopHoldGuard:
    """Turn release or command loss into a single frozen endpose target."""

    def __init__(self, ros: TianyiRosEndpose, *, watchdog_timeout_s: float = 0.40) -> None:
        self.ros = ros
        self.watchdog_timeout_s = max(0.05, float(watchdog_timeout_s))
        self.states = {"left": _BridgeSideState(), "right": _BridgeSideState()}
        self._watchdog_paused_until = 0.0
        self._last_hold_pub = {"left": 0.0, "right": 0.0}
        self._hold_pub_period_s = 1.0 / 15.0

    def pause_watchdog(self, seconds: float = 0.6) -> None:
        """Avoid false holds while bridge handles get_tf / prepare / joint home."""
        until = time.monotonic() + max(0.0, float(seconds))
        if until > self._watchdog_paused_until:
            self._watchdog_paused_until = until

    def begin_external_motion(self, seconds: float = 90.0) -> None:
        """Drop stale Cartesian holds before a joint-space motion takes control."""
        self.pause_watchdog(seconds)
        for state in self.states.values():
            state.active = False
            state.hold_pos = None
            state.hold_quat = None

    def hold_current_all(self, *, reason: str) -> None:
        """Re-seed both idle holds from measured TCP after external motion."""
        for side in ("left", "right"):
            self._hold_current(side, reason=reason)  # type: ignore[arg-type]

    def hold_targets_all(
        self,
        targets: dict[Side, tuple[np.ndarray, np.ndarray]],
        *,
        reason: str,
    ) -> None:
        """Seed idle holds from a frozen controller hand-off target."""
        for side, (pos, quat) in targets.items():
            self._hold_pose(side, pos, quat, reason=reason)

    def _publish_hold(self, side: Side, *, force: bool = False) -> None:
        state = self.states[side]
        if state.hold_pos is None or state.hold_quat is None:
            return
        now = time.monotonic()
        if not force and now - self._last_hold_pub[side] < self._hold_pub_period_s:
            return
        try:
            self.ros.publish_pose(side, state.hold_pos, state.hold_quat)
            self._last_hold_pub[side] = now
        except Exception:  # noqa: BLE001
            # Ignore transient rclpy invalid-context during service restart.
            pass

    def _hold_pose(
        self,
        side: Side,
        pos: np.ndarray,
        quat: np.ndarray,
        *,
        reason: str,
    ) -> None:
        state = self.states[side]
        state.active = False
        state.hold_pos = np.asarray(pos, dtype=float).copy()
        state.hold_quat = np.asarray(quat, dtype=float).copy()
        self._publish_hold(side, force=True)
        print(f"[Bridge] {side} hold ({reason}) xyz={state.hold_pos.round(4).tolist()}")

    def _hold_current(self, side: Side, *, reason: str) -> None:
        try:
            pos, quat = self.ros.lookup_tcp(side)
        except Exception as exc:  # noqa: BLE001
            state = self.states[side]
            state.active = False
            state.hold_pos = None
            state.hold_quat = None
            print(f"[Bridge] {side} hold-current failed ({reason}): {exc}")
            return
        self._hold_pose(side, pos, quat, reason=reason)

    def accept_pose(
        self,
        side: Side,
        *,
        active: bool,
        xyz: np.ndarray | None = None,
        quat: np.ndarray | None = None,
        now: float | None = None,
    ) -> None:
        state = self.states[side]
        if not active:
            # Explicit xyz/quat from PC always wins — including after a watchdog
            # hold_current that sampled lagged TF during B-home.  Ignoring those
            # packets leaves the pre-home pose locked and the arm crawls back.
            if xyz is not None and quat is not None:
                reason = "Grip release/cmd" if state.active else "freeze cmd"
                self._hold_pose(side, xyz, quat, reason=reason)
            elif state.active:
                self._hold_current(side, reason="Grip release/tf")
            return
        if xyz is None or quat is None:
            return
        state.active = True
        state.last_active_rx = time.monotonic() if now is None else float(now)
        state.hold_pos = None
        state.hold_quat = None
        self.ros.publish_pose(side, xyz, quat)

    def tick(self, *, now: float | None = None) -> None:
        current = time.monotonic() if now is None else float(now)
        watchdog_ok = current >= self._watchdog_paused_until
        for side in ("left", "right"):
            state = self.states[side]
            if (
                watchdog_ok
                and state.active
                and current - state.last_active_rx > self.watchdog_timeout_s
            ):
                self._hold_current(side, reason="UDP watchdog")
            if not state.active and state.hold_pos is not None and state.hold_quat is not None:
                # Keep a low-rate hold stream so endpose does not fall back to an
                # older internal target — but never compete with an active stream.
                self._publish_hold(side)


def _tf_reply(ros: TianyiRosEndpose) -> dict:
    reply: dict = {"t": time.time(), "left": None, "right": None}
    for side in ("left", "right"):
        side_reply: dict = {
            "joint_names": ros.arm_joint_names(side),  # type: ignore[arg-type]
            "joints": ros.arm_q_snapshot(side),  # type: ignore[arg-type]
            "joint_velocities": ros.arm_dq_snapshot(side),  # type: ignore[arg-type]
        }
        try:
            pos, quat = ros.lookup_tcp(side)  # type: ignore[arg-type]
            side_reply.update({"xyz": pos.tolist(), "quat_wxyz": quat.tolist()})
        except Exception as exc:  # noqa: BLE001
            side_reply["error"] = str(exc)
        reply[side] = side_reply
    return reply


def main() -> int:
    p = argparse.ArgumentParser(description="Tianyi UDP→ROS endpose bridge")
    p.add_argument("--udp-bind", default="0.0.0.0")
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    p.add_argument("--from-frame", default=DEFAULT_FROM_FRAME)
    p.add_argument("--to-frame-left", default=DEFAULT_TO_FRAME_LEFT)
    p.add_argument("--to-frame-right", default=DEFAULT_TO_FRAME_RIGHT)
    p.add_argument("--prepare", action="store_true", help="enable arm + mode3 + auto_switch")
    p.add_argument("--no-disable-on-exit", action="store_true")
    p.add_argument(
        "--watchdog-timeout-s",
        type=float,
        default=0.40,
        help="active 期间 UDP 无新包超时后锁住实际 TCP（秒）",
    )
    p.add_argument(
        "--status-file",
        default=os.environ.get(
            "TIANYEE_BRIDGE_STATUS_FILE",
            "/home/ubuntu/pico_vr_teleop_tianyee/status.json",
        ),
        help="周期性写入机器人健康状态 JSON；空字符串关闭",
    )
    p.add_argument(
        "--status-period-s",
        type=float,
        default=2.0,
        help="状态文件刷新周期（秒）",
    )
    args = p.parse_args()

    ros = TianyiRosEndpose(
        from_frame=args.from_frame,
        to_frame_left=args.to_frame_left,
        to_frame_right=args.to_frame_right,
        node_name="tianyee_udp_ros_bridge",
    )
    if args.prepare:
        print("[Bridge] prepare arm...")
        ros.prepare_for_teleop()

    status = RobotStatusMonitor(
        ros,
        status_file=str(args.status_file or ""),
        period_s=float(args.status_period_s),
    )
    status.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.udp_bind, int(args.udp_port)))
    sock.settimeout(0.05)
    hold_guard = TeleopHoldGuard(ros, watchdog_timeout_s=args.watchdog_timeout_s)
    print(f"[Bridge] listening udp://{args.udp_bind}:{args.udp_port} → /endposetarget_L|R")

    try:
        while True:
            ros.spin_once(0.0)
            status.tick()
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                hold_guard.tick()
                continue
            try:
                pkt = decode_pose_packet(data)
            except Exception as exc:  # noqa: BLE001
                print(f"[Bridge] bad packet: {exc}")
                continue

            cmd = pkt.get("cmd")
            if cmd == "get_tf":
                # Query must not starve the active stream into a watchdog hold.
                hold_guard.pause_watchdog(0.8)
                sock.sendto(json.dumps(_tf_reply(ros), separators=(",", ":")).encode("utf-8"), addr)
                hold_guard.tick()
                continue

            if cmd == "get_state":
                # Read-only collection path. Unlike get_tf, do not pause the
                # active-stream watchdog: state polling must never mask loss of
                # teleoperation commands.
                reply = _tf_reply(ros)
                reply["cmd"] = "get_state"
                sock.sendto(json.dumps(reply, separators=(",", ":")).encode("utf-8"), addr)
                hold_guard.tick()
                continue

            if cmd == "get_status":
                hold_guard.pause_watchdog(0.5)
                reply = status.snapshot()
                reply["cmd"] = "get_status"
                sock.sendto(json.dumps(reply, separators=(",", ":")).encode("utf-8"), addr)
                hold_guard.tick()
                continue

            if cmd == "prepare":
                hold_guard.pause_watchdog(60.0)
                print("[Bridge] prepare (enable/mode3/endpose)...")
                try:
                    t0 = time.time()
                    ros.prepare_for_teleop()
                    reply = _tf_reply(ros)
                    reply["ok"] = True
                    reply["cmd"] = "prepare"
                    reply["elapsed_s"] = round(time.time() - t0, 2)
                    print(f"[Bridge] prepare done in {reply['elapsed_s']}s")
                except Exception as exc:  # noqa: BLE001
                    print(f"[Bridge] prepare failed: {exc}")
                    reply = {"ok": False, "error": str(exc), "cmd": "prepare", "t": time.time()}
                sock.sendto(json.dumps(reply, separators=(",", ":")).encode("utf-8"), addr)
                hold_guard.pause_watchdog(0.5)
                continue

            if cmd == "go_home_joints":
                # Joint-space owns the arms during homing. Forget any Cartesian
                # target cached before the switch so it cannot be replayed later.
                hold_guard.begin_external_motion(90.0)
                duration = float(pkt.get("duration_s", DEFAULT_HOME_JOINT_DURATION_S))
                q_left = pkt.get("q_left") or list(HOME_Q_LEFT)
                q_right = pkt.get("q_right") or list(HOME_Q_RIGHT)
                print(f"[Bridge] go_home_joints duration={duration:.1f}s (elbow-down ready)")
                try:
                    t0 = time.time()
                    ros.move_joints_ready(
                        q_left=q_left,
                        q_right=q_right,
                        duration_s=duration,
                    )
                    # Freeze the reached TCP while jointspace still owns the
                    # hardware.  Reading TF repeatedly after activating
                    # endpose would chase any motion caused by its stale
                    # internal target and can pull both hands back upward.
                    ready_targets = ros.snapshot_tcp_targets()
                    if len(ready_targets) != 2:
                        raise RuntimeError("failed to snapshot both ready TCP targets")
                    # Pre-seed subscribers before the controller switch so
                    # endpose starts with the reached joint-ready pose.
                    ros.hold_endpose_targets(ready_targets, seconds=0.20)
                    # 关节控制器占用硬件时，末端话题不会动臂；必须切回 endpose
                    print("[Bridge] switch → endpose_single_arm_qp_*")
                    ros.activate_endpose_controllers()
                    # Keep publishing the exact pre-switch target.  Do not
                    # replace it with a moving post-switch TF sample.
                    ros.hold_endpose_targets(ready_targets, seconds=0.80)
                    hold_guard.hold_targets_all(
                        ready_targets,
                        reason="joint home frozen target",
                    )
                    reply = _tf_reply(ros)
                    reply["ok"] = True
                    reply["cmd"] = "go_home_joints"
                    reply["endpose_ready"] = True
                    reply["elapsed_s"] = round(time.time() - t0, 2)
                    print(f"[Bridge] go_home_joints done in {reply['elapsed_s']}s")
                except Exception as exc:  # noqa: BLE001
                    print(f"[Bridge] go_home_joints failed: {exc}")
                    reply = {"ok": False, "error": str(exc), "cmd": "go_home_joints", "t": time.time()}
                sock.sendto(json.dumps(reply, separators=(",", ":")).encode("utf-8"), addr)
                hold_guard.pause_watchdog(0.5)
                continue

            for side in ("left", "right"):
                side_data = pkt.get(side)
                if not isinstance(side_data, dict):
                    continue
                xyz = side_data.get("xyz")
                quat = side_data.get("quat_wxyz")
                xyz_arr = (
                    np.asarray(xyz, dtype=float)
                    if isinstance(xyz, (list, tuple)) and len(xyz) == 3
                    else None
                )
                quat_arr = (
                    np.asarray(quat, dtype=float)
                    if isinstance(quat, (list, tuple)) and len(quat) == 4
                    else None
                )
                if not side_data.get("active"):
                    hold_guard.accept_pose(  # type: ignore[arg-type]
                        side,
                        active=False,
                        xyz=xyz_arr,
                        quat=quat_arr,
                    )
                    continue
                if xyz_arr is None or quat_arr is None:
                    continue
                hold_guard.accept_pose(  # type: ignore[arg-type]
                    side,
                    active=True,
                    xyz=xyz_arr,
                    quat=quat_arr,
                )
            hold_guard.tick()
    except KeyboardInterrupt:
        print("\n[Bridge] stop")
    finally:
        sock.close()
        if args.prepare and not args.no_disable_on_exit:
            try:
                ros.call_enable(False)
            except Exception as exc:  # noqa: BLE001
                print(f"[Bridge] disable failed: {exc}")
        ros.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
