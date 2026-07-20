#!/usr/bin/env python3
"""JAKA 机械臂 TCP/IP 协议客户端（端口 10001 控制 / 10000 状态）。"""

from __future__ import annotations

import json
import socket
import time
from typing import Any


class JakaTcpError(RuntimeError):
    """TCP/IP 指令返回非零 errorCode 时抛出。"""


class JakaTcpClient:
    """
     * @brief 通过 JSON over TCP 与 JAKA 控制器通信。
     *
     * 控制指令走 10001 端口；10000 端口用于周期性状态推送（可选连接）。
     """

    CMD_PORT = 10001
    STATUS_PORT = 10000

    def __init__(self, ip: str, timeout: float = 5.0) -> None:
        self.ip = ip
        self.timeout = timeout
        self._cmd_sock: socket.socket | None = None
        self._status_sock: socket.socket | None = None
        self._cmd_buffer = b""

    def connect(self, enable_status_port: bool = False) -> None:
        """
         * @brief 连接控制器 10001 端口；可选连接 10000 状态端口。
         * @param enable_status_port 是否同时连接 10000 端口接收状态流
         """
        self._cmd_buffer = b""
        self._cmd_sock = self._open_socket(self.CMD_PORT)
        if enable_status_port:
            self._status_sock = self._open_socket(self.STATUS_PORT)

    def reconnect(self, enable_status_port: bool = False) -> None:
        """关闭后重新连接 10001 端口。"""
        self.close()
        self.connect(enable_status_port=enable_status_port)

    def probe(self, retries: int = 3, retry_delay: float = 1.0) -> dict[str, Any]:
        """
         * @brief 探测控制器通信是否正常，失败时自动重连重试。
         * @return get_version 的响应
         """
        last_error: Exception | None = None
        for attempt in range(1, retries + 1):
            try:
                return self.get_version()
            except (TimeoutError, ConnectionError, OSError, ValueError) as exc:
                last_error = exc
                if attempt >= retries:
                    break
                time.sleep(retry_delay)
                self.reconnect()
        assert last_error is not None
        raise last_error

    def close(self) -> None:
        """关闭所有已建立的 socket。"""
        for sock in (self._cmd_sock, self._status_sock):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        self._cmd_sock = None
        self._status_sock = None
        self._cmd_buffer = b""

    def __enter__(self) -> JakaTcpClient:
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def send_command(
        self,
        cmd: dict[str, Any],
        *,
        wait_response: bool = True,
        raise_on_error: bool = True,
    ) -> dict[str, Any]:
        """
         * @brief 向 10001 端口发送 JSON 指令。
         * @param wait_response 是否等待并读取控制器响应
         * @param raise_on_error errorCode 非 "0" 时是否抛异常
         """
        if self._cmd_sock is None:
            raise RuntimeError("未连接控制器，请先调用 connect()")

        cmd_name = str(cmd.get("cmdName", "unknown"))
        payload = json.dumps(cmd, separators=(",", ":"))
        self._cmd_sock.sendall(payload.encode("utf-8"))

        if not wait_response:
            return {}

        response = self._wait_response_for(cmd_name)
        if raise_on_error:
            self._check_response(response, cmd_name)
        return response

    def _wait_response_for(self, cmd_name: str) -> dict[str, Any]:
        """
         * @brief 读取响应直到 cmdName 与请求匹配。
         *
         * 高频非阻塞发送后，socket 中可能积压多条旧响应，需跳过直至收到本条指令的回复。
         """
        if self._cmd_sock is None:
            raise RuntimeError("未连接控制器，请先调用 connect()")

        deadline = time.monotonic() + self.timeout
        stale: list[str] = []
        while time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            old_timeout = self._cmd_sock.gettimeout()
            try:
                self._cmd_sock.settimeout(max(remaining, 0.01))
                resp = self._recv_json(
                    self._cmd_sock,
                    buffer_attr="_cmd_buffer",
                    cmd_name=cmd_name,
                )
            finally:
                self._cmd_sock.settimeout(old_timeout)

            resp_cmd = str(resp.get("cmdName", "")).strip()
            if resp_cmd == cmd_name:
                return resp
            stale.append(resp_cmd or "?")

        detail = ""
        if stale:
            preview = ", ".join(stale[:8])
            if len(stale) > 8:
                preview += f", ... (+{len(stale) - 8})"
            detail = f"（期间跳过 {len(stale)} 条旧响应: {preview}）"
        raise TimeoutError(
            f"指令 {cmd_name} 等待响应超时{detail}。"
            "若使用非阻塞伺服发送，请确认 drain_responses 已及时调用。"
        )

    def poll_response(self, timeout: float = 0.002) -> dict[str, Any] | None:
        """非阻塞读取一条已到达的响应（用于伺服高频发送场景）。"""
        if self._cmd_sock is None:
            return None

        old_timeout = self._cmd_sock.gettimeout()
        try:
            self._cmd_sock.settimeout(timeout)
            return self._recv_json(
                self._cmd_sock,
                buffer_attr="_cmd_buffer",
                cmd_name="",
            )
        except (TimeoutError, BlockingIOError):
            return None
        finally:
            self._cmd_sock.settimeout(old_timeout)

    def drain_responses(self, limit: int = 512) -> list[dict[str, Any]]:
        """尽量读完缓冲区中的响应，避免高频发送时积压。"""
        responses: list[dict[str, Any]] = []
        idle_rounds = 0
        for _ in range(limit):
            resp = self.poll_response(timeout=0.001)
            if resp is None:
                idle_rounds += 1
                if idle_rounds >= 3:
                    break
                continue
            idle_rounds = 0
            responses.append(resp)
        return responses

    def power_on(self) -> dict[str, Any]:
        """上电。"""
        return self.send_command({"cmdName": "power_on"})

    def power_off(self) -> dict[str, Any]:
        """断电。"""
        return self.send_command({"cmdName": "power_off"})

    def enable_robot(self) -> dict[str, Any]:
        """上使能。"""
        return self.send_command({"cmdName": "enable_robot"})

    def disable_robot(self) -> dict[str, Any]:
        """下使能。"""
        return self.send_command({"cmdName": "disable_robot"})

    def get_robot_state(self) -> dict[str, Any]:
        """查询上电/使能状态。"""
        return self.send_command({"cmdName": "get_robot_state"})

    def get_version(self) -> dict[str, Any]:
        """查询控制器版本信息。"""
        return self.send_command({"cmdName": "get_version"})

    def get_joint_pos(self) -> list[float]:
        """获取当前关节角（度）。"""
        resp = self.send_command({"cmdName": "get_joint_pos"})
        return list(resp.get("joint_pos", []))

    def get_motion_state(self) -> dict[str, Any]:
        """查询运动队列与到位状态。"""
        return self.send_command({"cmdName": "get_motion_state"})

    def get_drag_status(self) -> dict[str, Any]:
        """查询是否处于拖拽示教模式。"""
        return self.send_command({"cmdName": "get_drag_status"})

    def get_rapid_rate(self) -> dict[str, Any]:
        """查询速度倍率。"""
        return self.send_command({"cmdName": "get_rapid_rate"})

    def set_rapid_rate(self, rate: float) -> dict[str, Any]:
        """设置速度倍率 [0, 1]。"""
        return self.send_command({"cmdName": "rapid_rate", "rate_value": rate})

    def get_curr_user_id(self) -> dict[str, Any]:
        """查询当前用户坐标系 ID。"""
        return self.send_command({"cmdName": "get_curr_user_id"})

    def get_curr_tool_id(self) -> dict[str, Any]:
        """查询当前工具坐标系 ID。"""
        return self.send_command({"cmdName": "get_curr_tool_id"})

    def stop_program(self) -> dict[str, Any]:
        """停止当前程序/运动。"""
        return self.send_command({"cmdName": "stop_program"})

    def ensure_motion_ready(self) -> dict[str, Any]:
        """
         * @brief 运动前清理：退出伺服、停止残留运动，并返回关键状态。
         """
        if self.is_in_servomove():
            self.servo_move_enable(False)
        try:
            self.stop_program()
        except JakaTcpError:
            pass
        time.sleep(0.1)
        return {
            "robot": self.get_robot_state(),
            "motion": self.get_motion_state(),
            "drag": self.get_drag_status(),
            "rapid": self.get_rapid_rate(),
            "servo": self.is_in_servomove(),
            "user_id": self.get_curr_user_id().get("id"),
            "tool_id": self.get_curr_tool_id().get("id"),
        }

    def get_tcp_pos(self) -> list[float]:
        """获取当前 TCP 位姿 [x,y,z,a,b,c]，单位 mm/度。"""
        resp = self.send_command({"cmdName": "get_tcp_pos"})
        return list(resp.get("tcp_pos", []))

    def get_actual_tcp_pos(self) -> list[float]:
        """获取当前实际 TCP 位姿 [x,y,z,a,b,c]，单位 mm/度。"""
        resp = self.send_command({"cmdName": "get_actual_tcp_pos"})
        return list(resp.get("position", []))

    def is_in_servomove(self) -> bool:
        """查询是否处于伺服运动模式。"""
        resp = self.send_command({"cmdName": "is_in_servomove"})
        return bool(resp.get("in_servomove"))

    def servo_move_enable(self, enable: bool) -> dict[str, Any]:
        """进入/退出笛卡尔伺服模式。enable=True 进入，False 退出。"""
        return self.send_command({"cmdName": "servo_move", "relFlag": 1 if enable else 0})

    def prepare_servo_mode(
        self,
        *,
        filter_type: int | None = None,
        filter_preset: str = "lpf",
        **filter_kwargs: Any,
    ) -> None:
        """
         * @brief 按官方流程准备伺服模式：先退出 → 设滤波器 → 再进入。
         *
         * 文档要求 set_servo_move_filter 必须在非伺服状态下调用。
         * 遥操作推荐 filter_preset=\"lpf\"，无滤波仅适合预规划轨迹。
         """
        if self.is_in_servomove():
            self.servo_move_enable(False)

        if filter_type is None:
            if filter_preset == "none":
                filter_type = 0
                filter_kwargs = {}
            elif filter_preset == "lpf":
                filter_type = 1
                filter_kwargs.setdefault("lpf_cf", 0.5)
            elif filter_preset == "carte":
                filter_type = 4
                filter_kwargs.setdefault("nlf_max_vr", 2.0)
                filter_kwargs.setdefault("nlf_max_ar", 2.0)
                filter_kwargs.setdefault("nlf_max_jr", 4.0)
                filter_kwargs.setdefault("nlf_max_vp", 50.0)
                filter_kwargs.setdefault("nlf_max_ap", 200.0)
                filter_kwargs.setdefault("nlf_max_jp", 800.0)
            else:
                raise ValueError(f"未知 filter_preset: {filter_preset}")

        try:
            self.set_servo_move_filter(filter_type, **filter_kwargs)
        except JakaTcpError:
            pass
        self.servo_move_enable(True)
        if not self.is_in_servomove():
            raise JakaTcpError("未能进入伺服模式，请确认机械臂已使能且无报警")

    def move_l(
        self,
        cart_position: list[float],
        *,
        rel_flag: int = 1,
        speed: float = 20.0,
        accel: float = 50.0,
        wait: bool = True,
        poll_interval: float = 0.05,
        timeout: float = 60.0,
    ) -> dict[str, Any]:
        """
         * @brief 笛卡尔直线运动 MoveL，单位 mm/度。
         *
         * TCP 返回 errorCode=0 仅表示指令已接受（与 C++ 非阻塞 linear_move 类似），
         * 需配合 wait_motion_done 等待实际到位。
         """
        resp = self.send_command(
            {
                "cmdName": "moveL",
                "relFlag": rel_flag,
                "cartPosition": cart_position,
                "speed": speed,
                "accel": accel,
            }
        )
        if wait:
            self.wait_motion_done(timeout=timeout, poll_interval=poll_interval)
        return resp

    def kine_inverse(
        self,
        ref_joint: list[float],
        cart_position: list[float],
        *,
        raise_on_error: bool = True,
    ) -> list[float]:
        """
         * @brief 逆运动学求解，返回关节角（度）。
         * @param ref_joint 参考关节角（建议用当前关节角）
         * @param cart_position [x,y,z,a,b,c]，mm/度
         """
        resp = self.send_command(
            {
                "cmdName": "kine_inverse",
                "jointPosition": ref_joint,
                "cartPosition": cart_position,
            },
            raise_on_error=raise_on_error,
        )
        joints = resp.get("jointPosition")
        if joints is None:
            raise JakaTcpError(f"kine_inverse 失败: {resp}")
        return [float(v) for v in joints]

    def servo_p(
        self,
        cart_position: list[float],
        *,
        rel_flag: int = 0,
        step_num: int = 1,
        raise_on_error: bool = True,
        wait_response: bool = True,
    ) -> dict[str, Any]:
        """
         * @brief 笛卡尔空间伺服运动。
         * @param cart_position [x,y,z,a,b,c]，位置 mm，姿态度（欧拉 XYZ，度）
         * @param rel_flag 0=绝对（官方 SDK 示例），1=相对小增量
         * @param step_num 周期分频，执行周期 = step_num * 8ms
         * @param wait_response 是否等待响应；伺服环建议 False
         """
        # TCP 文档字段为 catPosition；moveL/kine_inverse 使用 cartPosition，勿混用。
        return self.send_command(
            {
                "cmdName": "servo_p",
                "catPosition": cart_position,
                "relFlag": rel_flag,
                "stepNum": step_num,
            },
            raise_on_error=raise_on_error,
            wait_response=wait_response,
        )

    def servo_j(
        self,
        joint_position: list[float],
        *,
        rel_flag: int = 0,
        step_num: int = 1,
        raise_on_error: bool = True,
        wait_response: bool = True,
    ) -> dict[str, Any]:
        """关节空间伺服运动，单位：度。"""
        return self.send_command(
            {
                "cmdName": "servo_j",
                "jointPosition": joint_position,
                "relFlag": rel_flag,
                "stepNum": step_num,
            },
            raise_on_error=raise_on_error,
            wait_response=wait_response,
        )

    def set_servo_move_filter(self, filter_type: int = 0, **kwargs: Any) -> dict[str, Any]:
        """设置伺服滤波器（建议在进入伺服模式前设置）。"""
        cmd: dict[str, Any] = {"cmdName": "set_servo_move_filter", "filter_type": filter_type}
        cmd.update(kwargs)
        return self.send_command(cmd)

    def joint_move(
        self,
        joint_position: list[float],
        *,
        rel_flag: int = 0,
        speed: float = 20.0,
        accel: float = 20.0,
        wait: bool = True,
        poll_interval: float = 0.1,
        timeout: float = 60.0,
    ) -> dict[str, Any]:
        """
         * @brief 关节空间运动 MoveJ。
         * @param joint_position 六个关节角，单位：度
         * @param rel_flag 0=绝对运动，1=相对运动
         * @param speed 关节速度 (°/s)
         * @param accel 关节加速度 (°/s²)
         * @param wait 是否阻塞等待到位
         """
        resp = self.send_command(
            {
                "cmdName": "joint_move",
                "relFlag": rel_flag,
                "jointPosition": joint_position,
                "speed": speed,
                "accel": accel,
            }
        )
        if wait:
            self.wait_motion_done(timeout=timeout, poll_interval=poll_interval)
        return resp

    def wait_motion_done(
        self,
        *,
        poll_interval: float = 0.05,
        timeout: float = 60.0,
        start_timeout: float = 3.0,
    ) -> None:
        """
         * @brief 轮询 get_motion_state，直到运动完成。
         *
         * 先等待 inpos 变 false（运动开始），再等待 inpos 变 true（到位）。
         """
        deadline = time.monotonic() + timeout
        start_deadline = time.monotonic() + start_timeout

        while time.monotonic() < start_deadline:
            state = self.get_motion_state()
            if state.get("inpos") is False:
                break
            queue = int(state.get("queue") or 0)
            active = int(state.get("active_queue") or 0)
            if queue > 0 or active > 0:
                break
            time.sleep(poll_interval)

        while time.monotonic() < deadline:
            state = self.get_motion_state()
            if state.get("inpos") is True and not state.get("paused"):
                return
            time.sleep(poll_interval)
        raise TimeoutError("等待运动完成超时")

    def read_status_once(self, bufsize: int = 65536) -> dict[str, Any] | None:
        """
         * @brief 从 10000 端口读取一帧状态 JSON（需已连接状态端口）。
         * @return 解析后的状态字典；无数据时返回 None
         """
        if self._status_sock is None:
            raise RuntimeError("未连接 10000 状态端口")

        self._status_sock.settimeout(2.0)
        try:
            chunk = self._status_sock.recv(bufsize)
        except socket.timeout:
            return None
        if not chunk:
            return None

        text = chunk.decode("utf-8", errors="replace").strip()
        if not text:
            return None
        return json.loads(text)

    def _open_socket(self, port: int) -> socket.socket:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        sock.connect((self.ip, port))
        return sock

    def _recv_json(
        self,
        sock: socket.socket,
        *,
        buffer_attr: str,
        cmd_name: str = "unknown",
    ) -> dict[str, Any]:
        """从 socket 缓冲区解析一条 JSON 响应，并保留后续未消费的数据。"""
        buffer: bytes = getattr(self, buffer_attr)
        decoder = json.JSONDecoder()
        deadline = time.monotonic() + self.timeout

        while time.monotonic() < deadline:
            if buffer:
                text = buffer.decode("utf-8", errors="replace").lstrip()
                if text:
                    try:
                        obj, end = decoder.raw_decode(text)
                        remainder = text[end:].lstrip().encode("utf-8")
                        setattr(self, buffer_attr, remainder)
                        return obj
                    except json.JSONDecodeError:
                        pass

            try:
                chunk = sock.recv(65536)
            except (socket.timeout, BlockingIOError) as exc:
                if buffer:
                    break
                raise TimeoutError(self._timeout_help(cmd_name)) from exc

            if not chunk:
                if not buffer:
                    raise ConnectionError(self._peer_closed_help(cmd_name))
                break
            buffer += chunk
            setattr(self, buffer_attr, buffer)

        buffer = getattr(self, buffer_attr)
        if buffer:
            text = buffer.decode("utf-8", errors="replace").lstrip()
            try:
                obj, end = decoder.raw_decode(text)
                remainder = text[end:].lstrip().encode("utf-8")
                setattr(self, buffer_attr, remainder)
                return obj
            except json.JSONDecodeError as exc:
                raise ValueError(f"无法解析控制器响应({cmd_name}): {buffer!r}") from exc
        raise TimeoutError(self._timeout_help(cmd_name))

    @staticmethod
    def _timeout_help(cmd_name: str) -> str:
        if not cmd_name:
            return "读取控制器响应超时（缓冲区暂无完整 JSON）。"
        return (
            f"指令 {cmd_name} 等待响应超时。"
            "请检查：1) 关闭 JAKA App 或其他占用 10001 端口的程序；"
            "2) 确认机械臂控制器已启动；3) 检查网线/交换机连接。"
        )

    @staticmethod
    def _peer_closed_help(cmd_name: str) -> str:
        return (
            f"控制器立即关闭了连接（指令 {cmd_name}）。"
            "10001 端口同时只能有一个控制客户端——请关闭 JAKA App、"
            "示教器上的二次开发连接或其他 TCP 程序后重试。"
        )

    @staticmethod
    def is_powered(state: dict[str, Any]) -> bool:
        """判断 get_robot_state 返回的是否已上电。"""
        power = state.get("power")
        return power in (1, True, "1")

    @staticmethod
    def is_enabled(state: dict[str, Any]) -> bool:
        """判断 get_robot_state 返回的是否已使能。"""
        enable = state.get("enable")
        return enable in (1, True, "1")

    @staticmethod
    def _check_response(response: dict[str, Any], cmd_name: str) -> None:
        error_code = str(response.get("errorCode", ""))
        if error_code != "0":
            error_msg = response.get("errorMsg", "")
            raise JakaTcpError(
                f"指令 {cmd_name} 失败: errorCode={error_code}, errorMsg={error_msg}"
            )
