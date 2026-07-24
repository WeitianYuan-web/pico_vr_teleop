#!/usr/bin/env python3
"""
示例：仅订阅 io_gateway WebSocket 上指定的 stream_id，统计客户端实际收到的频率。

用于对比 ROS2 话题频率与 WS 到达频率，例如：
  ros2 topic hz /io_teleop/AGIBOT_OmniHand_Pro/joint_cmd_finger_left   # 约 120 Hz
  python3 scripts/ws_stream_hz.py io_teleop.joint_cmd_left.AGIBOT_OmniHand_Pro  # 若约 60 Hz

若配置了 gateway.yaml websocket.max_fps > 0，WsHub 会按流节流（见 ws/ws_hub.py）；
max_fps: 0 表示不节流，WS 频率应接近 ROS 订阅回调频率。

前置：./scripts/run_gateway.sh 已启动，且对应 ROS 节点在发布。

用法：
  python3 scripts/ws_stream_hz.py --list
  python3 scripts/ws_stream_hz.py io_teleop.joint_cmd_left.AGIBOT_OmniHand_Pro
  python3 scripts/ws_stream_hz.py io_esk.joint_data --window 1.0 --interval 1.0 --duration 30

环境变量：
  GATEWAY_URL  默认 http://127.0.0.1:8080
  GATEWAY_WS   默认 ws://127.0.0.1:8080/ws
  IO_EXOTRANS2HAND_ROOT  用于读取 topics.yaml 中的 max_fps（可选）
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import re
import signal
import sys
import time
import urllib.error
import urllib.request
from collections import deque
from pathlib import Path
from typing import Any

DEFAULT_API = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080").rstrip("/")
DEFAULT_WS = os.environ.get("GATEWAY_WS", "ws://127.0.0.1:8080/ws")


def project_root() -> Path:
    env = os.environ.get("IO_EXOTRANS2HAND_ROOT")
    if env:
        return Path(env).resolve()
    here = Path(__file__).resolve()
    for parent in here.parents:
        if (parent / "configs" / "config" / "topics.yaml").is_file():
            return parent
    return here.parents[2]


def http_json(path: str, timeout: float = 5.0) -> Any:
    url = f"{DEFAULT_API}{path}"
    with urllib.request.urlopen(url, timeout=timeout) as resp:
        return json.loads(resp.read().decode())


def load_gateway_max_fps_hint() -> int | None:
    """从 configs/config/gateway.yaml 读取 websocket.max_fps（无需 PyYAML）。"""
    for name in ("gateway.yaml", "topics.yaml"):
        yaml_path = project_root() / "configs" / "config" / name
        if not yaml_path.is_file():
            continue
        try:
            text = yaml_path.read_text(encoding="utf-8")
        except OSError:
            continue
        in_websocket = False
        for line in text.splitlines():
            if re.match(r"^\s*websocket:\s*$", line):
                in_websocket = True
                continue
            if in_websocket and re.match(r"^\S", line) and not line.startswith(" "):
                in_websocket = False
            m = re.match(r"^\s*max_fps:\s*(\d+)\s*(?:#.*)?$", line)
            if m and (in_websocket or name == "gateway.yaml"):
                return int(m.group(1))
    return None


def list_subscribe_streams() -> list[dict[str, Any]]:
    items = http_json("/api/v1/streams")
    if not isinstance(items, list):
        raise RuntimeError("/api/v1/streams 返回不是列表")
    return [x for x in items if x.get("direction") == "subscribe"]


def print_stream_catalog() -> None:
    try:
        items = list_subscribe_streams()
    except (urllib.error.URLError, TimeoutError, OSError) as e:
        print(f"无法访问 {DEFAULT_API}/api/v1/streams: {e}", file=sys.stderr)
        sys.exit(1)
    if not items:
        print("(无 subscribe 流)")
        return
    print(f"可订阅 stream id（GET /api/v1/streams，共 {len(items)} 个）：")
    for x in sorted(items, key=lambda i: str(i.get("id", ""))):
        print(f"  {x.get('id', '')}\ttopic={x.get('topic', '')}")


def resolve_stream_topic(stream_id: str) -> str | None:
    try:
        for x in list_subscribe_streams():
            if x.get("id") == stream_id:
                return x.get("topic")
    except (urllib.error.URLError, TimeoutError, OSError):
        return None
    return None


class SlidingWindowHz:
    """与 viz.js computeStreamHz 相同：滑动窗口内 (n-1)/span。"""

    def __init__(self, window_sec: float) -> None:
        self.window_sec = max(window_sec, 0.05)
        self._ticks: deque[float] = deque()

    def tick(self, t: float | None = None) -> None:
        now = time.perf_counter() if t is None else t
        self._ticks.append(now)
        cutoff = now - self.window_sec
        while self._ticks and self._ticks[0] < cutoff:
            self._ticks.popleft()

    def hz(self) -> float:
        n = len(self._ticks)
        if n == 0:
            return 0.0
        if n == 1:
            return 1.0
        span = self._ticks[-1] - self._ticks[0]
        if span <= 0:
            return float(n)
        return (n - 1) / span

    def count(self) -> int:
        return len(self._ticks)


class WsStreamHzRunner:
    def __init__(
        self,
        stream_id: str,
        *,
        window_sec: float,
        report_interval: float,
        duration_sec: float | None,
    ) -> None:
        self.stream_id = stream_id
        self.window_sec = window_sec
        self.report_interval = max(report_interval, 0.2)
        self.duration_sec = duration_sec
        self._meter = SlidingWindowHz(window_sec)
        self._total = 0
        self._started = time.perf_counter()
        self._last_report = self._started
        self._stop = asyncio.Event()

    def request_stop(self) -> None:
        self._stop.set()

    def _print_banner(self) -> None:
        topic = resolve_stream_topic(self.stream_id)
        max_fps = load_gateway_max_fps_hint()
        print(f"WebSocket: {DEFAULT_WS}")
        print(f"订阅 stream: {self.stream_id}")
        if topic:
            print(f"对应 ROS 话题: {topic}")
            print(f"对比命令: ros2 topic hz {topic}")
        if max_fps is not None:
            print(
                f"网关节流提示: topics.yaml io_gateway.websocket.max_fps = {max_fps} "
                f"(WsHub 每条流最高约 {max_fps} 条/秒推到 WS；高于 ROS 频率时会被丢弃)"
            )
        print(f"统计: 滑动窗口 {self.window_sec}s，每 {self.report_interval}s 打印一次")
        print("Ctrl+C 结束\n")

    def _maybe_report(self, force: bool = False) -> None:
        now = time.perf_counter()
        if not force and now - self._last_report < self.report_interval:
            return
        self._last_report = now
        elapsed = now - self._started
        avg = self._total / elapsed if elapsed > 0 else 0.0
        win_hz = self._meter.hz()
        ts = time.strftime("%H:%M:%S")
        print(
            f"[{ts}] stream={self.stream_id}  "
            f"window({self.window_sec:.1f}s)={win_hz:6.1f} Hz  "
            f"avg({elapsed:.1f}s)={avg:6.1f} Hz  "
            f"total={self._total}  win_n={self._meter.count()}",
            flush=True,
        )

    def _final_report(self) -> None:
        elapsed = time.perf_counter() - self._started
        avg = self._total / elapsed if elapsed > 0 else 0.0
        print("\n--- 汇总 ---")
        print(f"stream_id: {self.stream_id}")
        print(f"运行: {elapsed:.2f}s")
        print(f"收到消息: {self._total}")
        print(f"全程平均: {avg:.2f} Hz")
        print(f"末段滑动窗口({self.window_sec}s): {self._meter.hz():.2f} Hz")
        max_fps = load_gateway_max_fps_hint()
        if max_fps is not None and avg > 0:
            ratio = avg / max_fps if max_fps else 0
            if avg < max_fps * 0.85 and max_fps <= 120:
                print(
                    f"\n若 ROS2 topic hz 明显高于 {avg:.0f} Hz，而 max_fps={max_fps}，"
                    "请检查是否被网关节流；可将 topics.yaml 中 max_fps 调高或设为 ≥ ROS 频率。"
                )
            elif max_fps < 120 and avg <= max_fps + 2:
                print(
                    f"\nWS 频率接近 max_fps={max_fps}，说明瓶颈很可能在网关 WsHub 节流。"
                )

    async def run(self) -> int:
        try:
            import websockets
        except ImportError:
            print("需要: pip install websockets", file=sys.stderr)
            return 1

        self._print_banner()
        deadline = (
            self._started + self.duration_sec
            if self.duration_sec and self.duration_sec > 0
            else None
        )

        try:
            async with websockets.connect(DEFAULT_WS) as ws:
                await ws.send(
                    json.dumps(
                        {"op": "subscribe", "streams": [self.stream_id]},
                        ensure_ascii=False,
                    )
                )
                while not self._stop.is_set():
                    if deadline and time.perf_counter() >= deadline:
                        break
                    try:
                        raw = await asyncio.wait_for(ws.recv(), timeout=0.5)
                    except asyncio.TimeoutError:
                        self._maybe_report()
                        continue

                    try:
                        obj = json.loads(raw)
                    except json.JSONDecodeError:
                        continue

                    if obj.get("op") in ("published", "error"):
                        continue

                    stream = obj.get("stream")
                    if stream != self.stream_id:
                        continue
                    if not isinstance(obj.get("data"), dict):
                        continue

                    self._total += 1
                    self._meter.tick()
                    self._maybe_report()
        except OSError as e:
            print(f"WebSocket 连接失败: {e}", file=sys.stderr)
            return 1
        finally:
            self._maybe_report(force=True)
            self._final_report()
        return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="订阅单个 WS stream_id 并统计到达频率（对比 ros2 topic hz）"
    )
    p.add_argument(
        "stream_id",
        nargs="?",
        help="如 io_teleop.joint_cmd_left.AGIBOT_OmniHand_Pro、io_esk.joint_data、io_esk.tf",
    )
    p.add_argument("--list", action="store_true", help="列出可订阅 stream id")
    p.add_argument(
        "--window",
        type=float,
        default=1.0,
        metavar="SEC",
        help="滑动窗口长度（秒），用于瞬时 Hz，默认 1.0",
    )
    p.add_argument(
        "--interval",
        type=float,
        default=1.0,
        metavar="SEC",
        help="打印间隔（秒），默认 1.0",
    )
    p.add_argument(
        "--duration",
        type=float,
        default=None,
        metavar="SEC",
        help="运行 SEC 秒后自动退出",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.list:
        print_stream_catalog()
        return 0
    if not args.stream_id:
        print("请指定 stream_id，或 --list 查看列表。", file=sys.stderr)
        print(
            "示例: python3 scripts/ws_stream_hz.py io_teleop.joint_cmd_left.AGIBOT_OmniHand_Pro",
            file=sys.stderr,
        )
        return 2

    runner = WsStreamHzRunner(
        args.stream_id.strip(),
        window_sec=args.window,
        report_interval=args.interval,
        duration_sec=args.duration,
    )

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            loop.add_signal_handler(sig, runner.request_stop)
        except NotImplementedError:
            pass

    try:
        return loop.run_until_complete(runner.run())
    except KeyboardInterrupt:
        runner.request_stop()
        return 0
    finally:
        loop.close()


if __name__ == "__main__":
    sys.exit(main())
