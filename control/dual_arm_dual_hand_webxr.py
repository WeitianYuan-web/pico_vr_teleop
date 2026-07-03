#!/usr/bin/env python3
from __future__ import annotations

import argparse
import asyncio
import os
import sys
import time
from types import SimpleNamespace

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
WEBXR_SCRIPT_DIR = os.path.join(PROJECT_ROOT, "webxr_test", "scripts")
PUBLISHER_DIR = os.path.join(PROJECT_ROOT, "publisher")
INSPIRE_BUILD_PYTHON_DIR = os.path.join(PROJECT_ROOT, "InspireHandSDK_Y", "build", "python")

sys.path.insert(0, WEBXR_SCRIPT_DIR)
sys.path.insert(0, PUBLISHER_DIR)

from teleop_piper_webxr import (  # noqa: E402
    HANDS,
    WebXRPiperPlacoTeleop,
    resolve_arm_can_ports,
)
from teleop_state_bridge import hand_registers_to_radians  # noqa: E402


HAND_MIN_POSITION = 0.2          # 扳机松开时手的归一化位置下限（0=全张, 1=全握）
HAND_MAX_POSITION = 0.95         # 扳机按满时手的归一化位置上限
HAND_CMD_DEADBAND = 0.02         # 归一化位置变化小于该值则不下发
HAND_CMD_MIN_INTERVAL_S = 0.05   # 手命令最小间隔


class DualArmDualHandWebXRTeleop(WebXRPiperPlacoTeleop):
    """双臂 + 双灵巧手 WebXR 遥操作（Trigger 控制半张/张开）。"""

    def __init__(
        self,
        left_hand_port: str,
        right_hand_port: str,
        left_hand_id: int,
        right_hand_id: int,
        hand_baudrate: int,
        hand_force: int,
        hand_speed: int,
        hand_min_position: float,
        hand_max_position: float,
        hand_control_hz: int,
        hand_io_hz: int,
        hand_connect_retry: int,
        strict_hand_connect: bool = False,
        disable_hands: bool = False,
        publish_state: bool = True,
        state_udp_host: str = "127.0.0.1",
        state_udp_port: int = 17981,
        state_publish_hz: float = 50.0,
        **kwargs,
    ):
        super().__init__(
            publish_state=publish_state,
            state_udp_host=state_udp_host,
            state_udp_port=state_udp_port,
            state_publish_hz=state_publish_hz,
            **kwargs,
        )
        self.disable_hands = disable_hands
        self.strict_hand_connect = strict_hand_connect
        self.hand_baudrate = hand_baudrate
        self.hand_force = hand_force
        self.hand_speed = hand_speed
        self.hand_min_position = max(0.0, min(float(hand_min_position), 1.0))
        self.hand_max_position = max(self.hand_min_position, min(float(hand_max_position), 1.0))
        self.hand_control_hz = hand_control_hz
        self.hand_io_hz = hand_io_hz
        self.hand_connect_retry = max(1, hand_connect_retry)
        self._ih = None
        self._hand_open_pose: list[int] | None = None
        self._hand_close_pose: list[int] | None = None
        self.dex_hands = {
            "left": SimpleNamespace(
                side="left",
                port=left_hand_port,
                hand_id=left_hand_id,
                dev=None,
                connected=False,
                last_cmd_alpha=None,
                last_cmd_time=0.0,
            ),
            "right": SimpleNamespace(
                side="right",
                port=right_hand_port,
                hand_id=right_hand_id,
                dev=None,
                connected=False,
                last_cmd_alpha=None,
                last_cmd_time=0.0,
            ),
        }

    def _load_inspire_binding(self):
        if self.disable_hands:
            return None
        if self._ih is not None:
            return self._ih
        if os.path.isdir(INSPIRE_BUILD_PYTHON_DIR):
            sys.path.insert(0, INSPIRE_BUILD_PYTHON_DIR)
        try:
            import inspire_hand_py as ih
        except ImportError as exc:
            raise RuntimeError(
                "未找到 inspire_hand_py，请先编译 InspireHandSDK_Y Python 绑定："
                "cd InspireHandSDK_Y && "
                "cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON && "
                "cmake --build build --target inspire_hand_py"
            ) from exc
        self._ih = ih
        return ih

    def _trigger_to_hand_alpha(self, trigger: float) -> float:
        """将扳机 [0,1] 线性映射到 [hand_min_position, hand_max_position]。"""
        t = max(0.0, min(float(trigger), 1.0))
        lo = self.hand_min_position
        hi = self.hand_max_position
        return lo + t * (hi - lo)

    def _lerp_hand_pose(self, alpha: float) -> list[int]:
        if self._hand_open_pose is None or self._hand_close_pose is None:
            raise RuntimeError("灵巧手姿态未初始化")
        a = max(0.0, min(float(alpha), 1.0))
        return [
            int(self._hand_open_pose[i] + a * (self._hand_close_pose[i] - self._hand_open_pose[i]))
            for i in range(len(self._hand_open_pose))
        ]

    def _submit_hand_alpha(self, info: SimpleNamespace, alpha: float) -> bool:
        if info.dev is None:
            return False
        pose = self._lerp_hand_pose(alpha)
        return bool(
            info.dev.submit_angles(pose, self.hand_force, self.hand_speed, True)
        )

    def connect_dex_hands(self):
        if self.disable_hands:
            print("[Hand] 已禁用灵巧手控制（--disable-hands）")
            return
        ih = self._load_inspire_binding()
        self._hand_open_pose = list(ih.Hand.default_open_pose())
        self._hand_close_pose = list(ih.Hand.half_close_pose())
        for side in self.active_hands:
            info = self.dex_hands[side]
            dev = None
            for attempt in range(1, self.hand_connect_retry + 1):
                dev = ih.Hand(info.port)
                print(
                    f"[Hand-{side}] 连接尝试 {attempt}/{self.hand_connect_retry}: "
                    f"port={info.port}, hand_id={info.hand_id}, baud={self.hand_baudrate}"
                )
                ok = dev.connect(
                    hand_id=info.hand_id,
                    baudrate=self.hand_baudrate,
                    control_hz=self.hand_control_hz,
                    io_hz=self.hand_io_hz,
                    force=self.hand_force,
                    speed=self.hand_speed,
                )
                if not ok:
                    try:
                        dev.disconnect()
                    except Exception:
                        pass
                    time.sleep(0.2)
                    continue
                if not dev.start():
                    try:
                        dev.disconnect()
                    except Exception:
                        pass
                    time.sleep(0.2)
                    continue
                info.dev = dev
                info.connected = True
                info.last_cmd_alpha = None
                info.last_cmd_time = 0.0
                init_alpha = self.hand_min_position
                if self._submit_hand_alpha(info, init_alpha):
                    info.last_cmd_alpha = init_alpha
                    info.last_cmd_time = time.time()
                print(
                    f"[Hand-{side}] 已连接，线性控制 "
                    f"[{self.hand_min_position:.2f}, {self.hand_max_position:.2f}]（扳机映射）"
                )
                break
            if info.connected:
                continue
            err_msg = (
                f"[Hand-{side}] 连接失败: port={info.port}, hand_id={info.hand_id}, "
                f"baud={self.hand_baudrate}。建议先执行："
                f"python {PROJECT_ROOT}/InspireHandSDK_Y/python/diagnose_hand.py {info.port} "
                f"--hand-id {info.hand_id} --scan-id"
            )
            if self.strict_hand_connect:
                raise RuntimeError(err_msg)
            print(f"{err_msg}；已自动跳过该侧灵巧手，仅保留机械臂控制。")

    def _collect_hand_state(self, side: str) -> dict | None:
        if side not in self.active_hands or self.disable_hands:
            return None
        info = self.dex_hands[side]
        if not info.connected or info.dev is None:
            return None
        try:
            state = info.dev.get_state()
        except Exception:
            return None
        angles = state.get("angles", [])
        if not angles:
            return None
        return {
            "hand_valid": True,
            "hand_joints": hand_registers_to_radians(angles),
        }

    def disconnect_dex_hands(self):
        for side in self.active_hands:
            info = self.dex_hands[side]
            if not info.connected or info.dev is None:
                continue
            try:
                self._submit_hand_alpha(info, self.hand_min_position)
                time.sleep(0.05)
            except Exception:
                pass
            try:
                info.dev.stop()
            except Exception:
                pass
            try:
                info.dev.disconnect()
            except Exception:
                pass
            info.connected = False
            info.dev = None
            print(f"[Hand-{side}] 已断开")

    def _process_trigger_for_dex_hand(self, data: dict, side: str):
        info = self.dex_hands[side]
        if self.disable_hands or not info.connected or info.dev is None:
            return
        ctrl = next((c for c in data.get("controllers", []) if c.get("handedness") == side), None)
        if ctrl is None:
            return
        trigger = max(0.0, min(float(ctrl.get("trigger", 0.0)), 1.0))
        target_alpha = self._trigger_to_hand_alpha(trigger)
        now = time.time()
        if info.last_cmd_alpha is not None:
            if abs(target_alpha - info.last_cmd_alpha) < HAND_CMD_DEADBAND:
                return
        if now - info.last_cmd_time < HAND_CMD_MIN_INTERVAL_S:
            return
        ok = self._submit_hand_alpha(info, target_alpha)
        if ok:
            info.last_cmd_alpha = target_alpha
            info.last_cmd_time = now
        elif info.last_cmd_alpha is None or abs(target_alpha - info.last_cmd_alpha) >= 0.1:
            print(f"\n[Hand-{side}] 指令失败: Trigger={trigger:.2f}, alpha={target_alpha:.2f}")

    def process_vr_data(self, data: dict, hand: str):
        super().process_vr_data(data, hand)
        self._process_trigger_for_dex_hand(data, hand)

    async def ws_loop(self):
        print(
            "[Hand] 控制规则：Trigger 线性映射 -> "
            f"手位置 [{self.hand_min_position:.2f}, {self.hand_max_position:.2f}]（0=全张, 1=半握）"
        )
        await super().ws_loop()

    def run(self):
        try:
            self.connect_robots()
            self.connect_dex_hands()
            asyncio.run(self.ws_loop())
        except KeyboardInterrupt:
            print("\n[System] 收到退出信号")
        finally:
            self.disconnect_dex_hands()
            for hand in self.active_hands:
                arm = self.arms[hand]
                if arm.robot is None:
                    continue
                if self.disable_on_exit:
                    arm.robot.disable()
                arm.robot.disconnect()
                if self.disable_on_exit:
                    print(f"[Robot-{hand}] 已断开（已失能）")
                else:
                    print(f"[Robot-{hand}] 已断开（未失能）")


def parse_args():
    parser = argparse.ArgumentParser(description="WebXR 双臂双手遥操作（Piper + Inspire）")
    parser.add_argument(
        "--hands",
        choices=("both", "left", "right"),
        default="both",
        help="控制模式：both=双臂双手；left/right=单侧调试",
    )
    parser.add_argument("--left-can-port", default=None, help="左臂 CAN 端口，双臂默认 can0")
    parser.add_argument("--right-can-port", default=None, help="右臂 CAN 端口，双臂默认 can1")
    parser.add_argument("--left-hand-port", default="/dev/ttyUSB0", help="左手串口")
    parser.add_argument("--right-hand-port", default="/dev/ttyUSB1", help="右手串口")
    parser.add_argument("--left-hand-id", type=int, default=1, help="左手 hand_id")
    parser.add_argument("--right-hand-id", type=int, default=1, help="右手 hand_id")
    parser.add_argument("--hand-baudrate", type=int, default=115200, help="灵巧手串口波特率")
    parser.add_argument("--hand-force", type=int, default=300, help="灵巧手力参数")
    parser.add_argument("--hand-speed", type=int, default=900, help="灵巧手速度参数")
    parser.add_argument("--hand-connect-retry", type=int, default=3, help="灵巧手连接重试次数")
    parser.add_argument(
        "--strict-hand-connect",
        action="store_true",
        help="任一灵巧手连接失败即退出（默认跳过失败侧并继续双臂）",
    )
    parser.add_argument(
        "--hand-min-position",
        type=float,
        default=HAND_MIN_POSITION,
        help="扳机松开时手的归一化位置下限（默认 0.2，范围 0~1）",
    )
    parser.add_argument(
        "--hand-max-position",
        type=float,
        default=HAND_MAX_POSITION,
        help="扳机按满时手的归一化位置上限（默认 0.95，范围 0~1）",
    )
    parser.add_argument("--hand-control-hz", type=int, default=200, help="灵巧手控制线程频率")
    parser.add_argument("--hand-io-hz", type=int, default=30, help="灵巧手 IO 线程频率")
    parser.add_argument("--disable-hands", action="store_true", help="仅控制双臂，禁用灵巧手")
    parser.add_argument(
        "--publish-state",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="向 publisher 上报臂/手状态（UDP，默认开启）",
    )
    parser.add_argument("--state-udp-host", default="127.0.0.1", help="状态上报目标主机")
    parser.add_argument("--state-udp-port", type=int, default=17981, help="状态上报目标端口")
    parser.add_argument("--state-publish-hz", type=float, default=50.0, help="状态上报频率 Hz")
    parser.add_argument("--robot-model", default="piper_h", help="机械臂型号")
    parser.add_argument("--disable-on-exit", action="store_true", help="退出时执行 disable")
    parser.add_argument(
        "--rotation-mode",
        choices=("always", "hold-a", "off"),
        default="always",
        help="旋转控制：always=Grip 时平移+旋转；hold-a=Grip+A 才旋转；off=仅平移",
    )
    parser.add_argument("--rotation-scale", type=float, default=1.0, help="旋转增量缩放系数")
    parser.add_argument(
        "--tcp-offset",
        type=str,
        default="0,0,0,0,0,0",
        help="TCP 偏移 x,y,z,roll,pitch,yaw；单位 m/rad",
    )
    return parser.parse_args()


def parse_pose6(text: str):
    vals = [float(x.strip()) for x in text.split(",") if x.strip()]
    if len(vals) != 6:
        raise ValueError("--tcp-offset 必须是 6 个逗号分隔浮点数，例如: 0,0,0.10,0,0,0")
    return vals


def main():
    args = parse_args()
    active_hands = HANDS if args.hands == "both" else (args.hands,)
    left_can_port, right_can_port = resolve_arm_can_ports(
        active_hands, args.left_can_port, args.right_can_port
    )
    if len(active_hands) == 2:
        print(f"[System] 双臂 CAN 映射: left={left_can_port}, right={right_can_port}")
        print(
            "[System] 双手串口映射: "
            f"left={args.left_hand_port}(id={args.left_hand_id}), "
            f"right={args.right_hand_port}(id={args.right_hand_id})"
        )
    teleop = DualArmDualHandWebXRTeleop(
        hands=active_hands,
        left_can_port=left_can_port,
        right_can_port=right_can_port,
        robot_model=args.robot_model,
        disable_on_exit=args.disable_on_exit,
        rotation_mode=args.rotation_mode,
        rotation_scale=args.rotation_scale,
        tcp_offset_pose=parse_pose6(args.tcp_offset),
        left_hand_port=args.left_hand_port,
        right_hand_port=args.right_hand_port,
        left_hand_id=args.left_hand_id,
        right_hand_id=args.right_hand_id,
        hand_baudrate=args.hand_baudrate,
        hand_force=args.hand_force,
        hand_speed=args.hand_speed,
        hand_min_position=args.hand_min_position,
        hand_max_position=args.hand_max_position,
        hand_control_hz=args.hand_control_hz,
        hand_io_hz=args.hand_io_hz,
        hand_connect_retry=args.hand_connect_retry,
        strict_hand_connect=args.strict_hand_connect,
        disable_hands=args.disable_hands,
        publish_state=args.publish_state,
        state_udp_host=args.state_udp_host,
        state_udp_port=args.state_udp_port,
        state_publish_hz=args.state_publish_hz,
    )
    teleop.run()


if __name__ == "__main__":
    main()
