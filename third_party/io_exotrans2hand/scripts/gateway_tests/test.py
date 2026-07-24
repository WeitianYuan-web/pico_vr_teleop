#!/usr/bin/env python3
"""
io_gateway 简易测试：REST 示例 + WebSocket 订阅

前置：./scripts/run_gateway.sh 已启动

REST 示例：
  python3 scripts/gateway_tests/test.py --api status
  python3 scripts/gateway_tests/test.py --api hands_select DexcelRobotics_Apex
  python3 scripts/gateway_tests/test.py --api frequency 50
  python3 scripts/gateway_tests/test.py --api-help

WebSocket：
  python3 scripts/gateway_tests/test.py
  python3 scripts/gateway_tests/test.py io_esk.joint_data
  python3 scripts/gateway_tests/test.py --list

环境变量：GATEWAY_URL、GATEWAY_WS
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from typing import Any

from websockets.sync.client import connect

DEFAULT_API = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080").rstrip("/")
DEFAULT_WS = os.environ.get("GATEWAY_WS", "ws://127.0.0.1:8080/ws")

API_HELP = """
REST 示例（--api <命令>）：
  GET   status | bootstrap | probe | hands_configs | hands_dirs | streams | viz_config
  POST  hands_select [手型 ...]   # 无手型参数时清空
        frequency <hz>
        sync_start [手型]
        sync_stop [手型]

上传手型包请用：python3 scripts/gateway_tests/gateway_api_test_runner.py upload <文件>
""".strip()


def _print_json(data: Any) -> None:
    print(json.dumps(data, ensure_ascii=False, indent=2))


def http_json(path: str, *, method: str = "GET", body: dict | None = None, timeout: float = 15.0) -> Any:
    url = f"{DEFAULT_API}{path}"
    data = None
    headers = {}
    if body is not None:
        data = json.dumps(body).encode()
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, data=data, method=method, headers=headers)
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            raw = resp.read().decode()
            return json.loads(raw) if raw else {}
    except urllib.error.HTTPError as e:
        err_body = e.read().decode()
        try:
            detail = json.loads(err_body)
        except json.JSONDecodeError:
            detail = err_body
        print(f"HTTP {e.code}", file=sys.stderr)
        _print_json(detail)
        raise SystemExit(1) from e
    except urllib.error.URLError as e:
        print(f"请求失败（请先启动 gateway）: {e.reason}", file=sys.stderr)
        raise SystemExit(1) from e


def http_get(path: str) -> Any:
    return http_json(path, method="GET")


def http_post(path: str, body: dict | None = None) -> Any:
    return http_json(path, method="POST", body=body or {})


def run_api(cmd: str, rest: list[str]) -> int:
    """按命令名调用 GET / POST 并打印 JSON。"""
    get_cmds = {
        "status": "/api/v1/status",
        "bootstrap": "/api/v1/bootstrap",
        "probe": "/api/v1/probe",
        "hands_configs": "/api/v1/hands/configs",
        "hands_dirs": "/api/v1/hands/dirs",
        "streams": "/api/v1/streams",
        "viz_config": "/api/v1/visualization/config",
    }

    if cmd in get_cmds:
        _print_json(http_get(get_cmds[cmd]))
        return 0

    if cmd == "hands_select":
        _print_json(http_post("/api/v1/hands/select", {"hands": rest}))
        return 0

    if cmd == "frequency":
        if len(rest) != 1:
            print("用法: --api frequency <hz>", file=sys.stderr)
            return 1
        _print_json(http_post("/api/v1/runtime/frequency", {"hz": int(rest[0])}))
        return 0

    if cmd == "sync_start":
        body = {"hand": rest[0]} if rest else {}
        _print_json(http_post("/api/v1/control/sync/start", body))
        return 0

    if cmd == "sync_stop":
        body = {"hand": rest[0]} if rest else {}
        _print_json(http_post("/api/v1/control/sync/stop", body))
        return 0

    print(f"未知 API 命令: {cmd}\n\n{API_HELP}", file=sys.stderr)
    return 1


def print_stream_catalog() -> None:
    items = http_get("/api/v1/streams")
    subs = [x for x in items if x.get("direction") == "subscribe"]
    print(f"可订阅流（共 {len(subs)} 个）：")
    for x in sorted(subs, key=lambda i: str(i.get("id", ""))):
        print(f"  {x.get('id')}")


def run_client(ws_url: str, streams: list[str]) -> int:
    print(f"连接 {ws_url}")
    print(f"订阅 {len(streams)} 条: {', '.join(streams)}")
    print("Ctrl+C 结束\n")

    try:
        with connect(ws_url, open_timeout=5) as ws:
            ws.send(json.dumps({"op": "subscribe", "streams": streams}))

            raw = ws.recv()
            try:
                obj = json.loads(raw)
            except json.JSONDecodeError:
                print(raw[:200])
                return 0

            if obj.get("op") in ("published", "error"):
                print(f"# {obj}")

            stream = obj.get("stream")
            data = obj.get("data")
            print(f"stream: {stream} data: {data}")

    except KeyboardInterrupt:
        print("\n已停止")
        return 0
    except Exception as e:
        print(f"连接失败: {e}", file=sys.stderr)
        return 1


def main() -> int:
    p = argparse.ArgumentParser(description="io_gateway REST 示例 + WS 订阅")
    p.add_argument("streams", nargs="*", help="WS stream id，默认 io_esk.joint_data")
    p.add_argument("--list", action="store_true", help="列出 GET /api/v1/streams")
    p.add_argument("--api", metavar="CMD", help="REST 示例命令，见 --api-help")
    p.add_argument("--api-help", action="store_true", help="打印 REST 示例命令列表")
    p.add_argument("--ws", default=DEFAULT_WS, help=f"WebSocket 地址（默认 {DEFAULT_WS}）")
    args, extra = p.parse_known_args()

    if args.api_help:
        print(API_HELP)
        return 0

    if args.api:
        return run_api(args.api, extra)

    if args.list:
        print_stream_catalog()
        return 0

    streams = args.streams or ["io_esk.joint_data"]
    return run_client(args.ws, streams)


if __name__ == "__main__":
    sys.exit(main())
