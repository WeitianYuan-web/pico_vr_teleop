#!/usr/bin/env python3
"""JAKA Python SDK 薄封装。

SDK 关节角与速度单位为弧度；TCP 位置 mm、姿态 rx/ry/rz 为弧度。
"""

from __future__ import annotations

import math
import os
import sys
import time
import ctypes
from pathlib import Path
from typing import Any, Sequence

from config import ROBOT_IP, SDK_LIB_DIR


ERR_NAMES: dict[int, str] = {
    2: "interface_error",
    -4: "kine_inverse_err",
    -5: "emergency_stop",
    -6: "not_powered",
    -7: "not_enabled",
    -10: "program_running",
    -12: "motion_abnormal",
    -16: "kine_forward_err",
    -20: "protective_stop",
}


class JakaSdkError(RuntimeError):
    """SDK 返回非零错误码。"""


def _setup_sdk_path() -> None:
    """将 SDK 库目录加入 Python 与动态库搜索路径。"""
    sdk_dir = str(Path(SDK_LIB_DIR).resolve())
    if sdk_dir not in sys.path:
        sys.path.insert(0, sdk_dir)
    lib_path = os.environ.get("LD_LIBRARY_PATH", "")
    if sdk_dir not in lib_path.split(":"):
        os.environ["LD_LIBRARY_PATH"] = f"{sdk_dir}:{lib_path}" if lib_path else sdk_dir

    # 运行时预加载基础动态库，避免 import jkrc 时出现 libjakaAPI.so 找不到。
    # 某些 Linux 环境里仅修改 LD_LIBRARY_PATH 不能被当前进程即时感知。
    lib_jaka = Path(sdk_dir) / "libjakaAPI.so"
    if lib_jaka.exists():
        try:
            ctypes.CDLL(str(lib_jaka), mode=ctypes.RTLD_GLOBAL)
        except OSError:
            # 若预加载失败，仍让后续 import jkrc 抛出原始错误，便于定位缺失依赖。
            pass


def _import_jkrc():
    _setup_sdk_path()
    import jkrc  # noqa: WPS433

    return jkrc


def _check(ret: tuple[Any, ...], cmd: str) -> tuple[Any, ...]:
    code = int(ret[0])
    if code != 0:
        name = ERR_NAMES.get(code, "unknown")
        raise JakaSdkError(f"{cmd} 失败: errno={code} ({name}), ret={ret}")
    return ret


class JakaSdkRobot:
    """
     * @brief JAKA Python SDK 机器人客户端。
     *
     * 注意：SDK 与 TCP 10001 不宜同时连接；使用前请关闭其他控制程序。
     """

    ABS = 0
    INCR = 1

    def __init__(self, ip: str = ROBOT_IP) -> None:
        jkrc = _import_jkrc()
        self._jkrc = jkrc
        self._rc = jkrc.RC(ip)
        self._logged_in = False

    def __enter__(self) -> JakaSdkRobot:
        self.login()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        if self._logged_in:
            try:
                self.logout()
            except JakaSdkError:
                pass

    def login(self) -> None:
        _check(self._rc.login(), "login")
        self._logged_in = True

    def logout(self) -> None:
        _check(self._rc.logout(), "logout")
        self._logged_in = False

    def power_on(self) -> None:
        _check(self._rc.power_on(), "power_on")

    def power_off(self) -> None:
        _check(self._rc.power_off(), "power_off")

    def enable_robot(self) -> None:
        _check(self._rc.enable_robot(), "enable_robot")

    def disable_robot(self) -> None:
        _check(self._rc.disable_robot(), "disable_robot")

    def wait_power_on(self, timeout: float = 25.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            time.sleep(0.5)
            # SDK 无直接 power 查询，短等后由 enable 验证
            return

    def ensure_ready(self, *, skip_power: bool = False) -> None:
        if not skip_power:
            self.power_on()
            time.sleep(8.0)
        self.enable_robot()

    def get_joint_pos_rad(self) -> list[float]:
        ret = _check(self._rc.get_joint_position(), "get_joint_position")
        return [float(v) for v in ret[1]]

    def get_joint_pos_deg(self) -> list[float]:
        return [math.degrees(v) for v in self.get_joint_pos_rad()]

    def get_tcp_pos(self) -> list[float]:
        """返回 [x,y,z,rx,ry,rz]，位置 mm，姿态弧度。"""
        ret = _check(self._rc.get_tcp_position(), "get_tcp_position")
        return [float(v) for v in ret[1]]

    def get_rapid_rate(self) -> float:
        """查询速度倍率 [0, 1]。"""
        ret = _check(self._rc.get_rapidrate(), "get_rapidrate")
        return float(ret[1])

    def set_rapid_rate(self, rate: float) -> None:
        """设置速度倍率 [0, 1]，例如 0.5 表示 50%。"""
        rate = max(0.0, min(1.0, rate))
        _check(self._rc.set_rapidrate(rate), "set_rapidrate")

    def joint_move_rad(
        self,
        joints: Sequence[float],
        *,
        block: bool = True,
        speed_rad_s: float = 0.26,
    ) -> None:
        if len(joints) != 6:
            raise ValueError("需要 6 个关节角（弧度）")
        _check(
            self._rc.joint_move(tuple(joints), self.ABS, block, speed_rad_s),
            "joint_move",
        )

    def joint_move_deg(
        self,
        joints_deg: Sequence[float],
        *,
        block: bool = True,
        speed_deg_s: float = 15.0,
    ) -> None:
        joints = [math.radians(v) for v in joints_deg]
        self.joint_move_rad(joints, block=block, speed_rad_s=math.radians(speed_deg_s))

    def linear_move(
        self,
        pose: Sequence[float],
        *,
        block: bool = True,
        speed_mm_s: float = 20.0,
        timeout: float | None = 60.0,
    ) -> None:
        """
         * @brief 直线运动；可选超时（秒），避免奇异点处永久阻塞。
         """
        import threading

        err: list[Exception] = []

        def _run() -> None:
            try:
                _check(
                    self._rc.linear_move(tuple(pose), self.ABS, block, speed_mm_s),
                    "linear_move",
                )
            except JakaSdkError as exc:
                err.append(exc)

        if timeout is None:
            _run()
            return

        th = threading.Thread(target=_run, daemon=True)
        th.start()
        th.join(timeout)
        if th.is_alive():
            raise JakaSdkError(
                f"linear_move 超时 ({timeout}s)，可能处于奇异点附近。"
                "请示教器手动调整姿态，或使用 joint 回零。"
            )
        if err:
            raise err[0]

    def kine_inverse(
        self,
        ref_joint: Sequence[float],
        cart_pose: Sequence[float],
        *,
        raise_on_error: bool = True,
    ) -> list[float] | None:
        """逆运动学，返回关节角（弧度）。"""
        ret = self._rc.kine_inverse(tuple(ref_joint), tuple(cart_pose))
        code = int(ret[0])
        if code != 0:
            if raise_on_error:
                name = ERR_NAMES.get(code, "unknown")
                raise JakaSdkError(f"kine_inverse 失败: errno={code} ({name})")
            return None
        return [float(v) for v in ret[1]]

    def move_to_pose(
        self,
        pose: Sequence[float],
        *,
        speed_mm_s: float = 20.0,
        timeout: float = 60.0,
    ) -> None:
        """
         * @brief 先尝试 linear_move，失败则逆解 + joint_move。
         """
        try:
            self.linear_move(pose, block=True, speed_mm_s=speed_mm_s, timeout=timeout)
            return
        except JakaSdkError:
            pass
        ref = self.get_joint_pos_rad()
        joints = self.kine_inverse(ref, pose)
        if joints is None:
            raise JakaSdkError("无法到达目标位姿（linear_move 与 kine_inverse 均失败）")
        self.joint_move_rad(joints, block=True, speed_rad_s=math.radians(15.0))

    def prepare_servo(self, *, filter: str = "lpf") -> None:
        """进入伺服模式并设置滤波器。"""
        if filter == "none":
            _check(self._rc.servo_move_use_none_filter(), "servo_move_use_none_filter")
        elif filter == "lpf":
            _check(self._rc.servo_move_use_joint_LPF(0.5), "servo_move_use_joint_LPF")
        elif filter == "carte":
            _check(
                self._rc.servo_move_use_carte_NLF(50, 200, 800, 2, 2, 4),
                "servo_move_use_carte_NLF",
            )
        else:
            raise ValueError(f"未知 filter: {filter}")
        _check(self._rc.servo_move_enable(True, True), "servo_move_enable")

    def exit_servo(self) -> None:
        _check(self._rc.servo_move_enable(False, True), "servo_move_enable(False)")

    def servo_j_rad(self, joints: Sequence[float], *, step_num: int = 1) -> None:
        _check(self._rc.servo_j(tuple(joints), self.ABS, step_num), "servo_j")

    def servo_p(self, pose: Sequence[float], *, step_num: int = 1) -> int:
        """发送 servo_p，返回 errno（0 为成功）。"""
        ret = self._rc.servo_p(tuple(pose), self.ABS, step_num)
        return int(ret[0])

    def jog_stop(self) -> None:
        _check(self._rc.jog_stop(-1), "jog_stop")
