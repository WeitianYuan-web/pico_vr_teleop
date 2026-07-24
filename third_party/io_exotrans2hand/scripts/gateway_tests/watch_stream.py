#!/usr/bin/env python3
"""
订阅或发布 io_gateway WebSocket 数据流。

前置：./scripts/run_gateway.sh 已启动。

用法：
  python3 scripts/gateway_tests/watch_stream.py io_esk.joint_data
  python3 scripts/gateway_tests/watch_stream.py io_esk.joystick_data
  python3 scripts/gateway_tests/watch_stream.py io_esk.tf io_teleop.joint_cmd_left.BrainCo_Revo2
  python3 scripts/gateway_tests/watch_stream.py io_esk.vibration_feedback \
  --data '{"data":[10,0,0,0,0,0,0,0,0,10]}'

环境变量：GATEWAY_WS（默认 ws://127.0.0.1:8080/ws）
"""

from __future__ import annotations

import argparse
import asyncio
import json
import os
import sys
import time
from typing import Any

DEFAULT_WS = os.environ.get("GATEWAY_WS", "ws://127.0.0.1:8080/ws")


async def subscribe_streams(streams: list[str], *, ws_url: str = DEFAULT_WS) -> int:
    """订阅流函数模板。"""
    try:
        import websockets
    except ImportError:
        print("需要 websockets: pip install websockets", file=sys.stderr)
        return 1

    wanted = set(streams)
    print(f"连接 {ws_url}，订阅: {', '.join(streams)}", flush=True)
    print("Ctrl+C 结束\n", flush=True)

    try:
        async with websockets.connect(ws_url) as ws:
            await ws.send(json.dumps({"op": "subscribe", "streams": streams}))
            idx = 0
            while True:
                raw = await ws.recv()
                obj = json.loads(raw)
                if obj.get("op") in ("published", "error"):
                    print(f"# {obj}", flush=True)
                    continue
                stream = obj.get("stream")
                data = obj.get("data")
                if not stream or stream not in wanted or not isinstance(data, dict):
                    continue
                idx += 1
                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] #{idx} {stream} {json.dumps(data, ensure_ascii=False)}", flush=True)
    except KeyboardInterrupt:
        print("\n已停止", flush=True)
        return 0
    except OSError as e:
        print(f"WebSocket 失败: {e}", file=sys.stderr)
        return 1


async def publish_stream(
    stream_id: str,
    data: dict[str, Any],
    *,
    ws_url: str = DEFAULT_WS,
) -> int:
    """发布流函数模板。"""
    try:
        import websockets
    except ImportError:
        print("需要 websockets: pip install websockets", file=sys.stderr)
        return 1

    print(f"连接 {ws_url}，发布: {stream_id}", flush=True)
    print(f"data: {json.dumps(data, ensure_ascii=False)}\n", flush=True)

    try:
        async with websockets.connect(ws_url) as ws:
            await ws.send(
                json.dumps({"op": "publish", "stream": stream_id, "data": data})
            )
            raw = await ws.recv()
            print(json.dumps(json.loads(raw), ensure_ascii=False, indent=2), flush=True)
            return 0
    except OSError as e:
        print(f"WebSocket 失败: {e}", file=sys.stderr)
        return 1


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="订阅或发布 io_gateway WebSocket 流")
    p.add_argument(
        "streams",
        nargs="+",
        metavar="stream_id",
        help="stream id；订阅时可写多个，发布时取第一个",
    )
    p.add_argument(
        "--data",
        metavar="JSON",
        help="发布载荷；提供时进入发布模式",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    """主进入函数。"""
    args = parse_args(argv)

    if args.data:
        payload = json.loads(args.data)
        if not isinstance(payload, dict):
            print("--data 须为 JSON 对象", file=sys.stderr)
            return 2
        return asyncio.run(publish_stream(args.streams[0], payload))

    return asyncio.run(subscribe_streams(args.streams))


if __name__ == "__main__":
    sys.exit(main())
