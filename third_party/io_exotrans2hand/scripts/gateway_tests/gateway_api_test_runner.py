#!/usr/bin/env python3
"""
io_gateway REST API 简单调用示例。

用法：
  python3 gateway_api_test_runner.py status
  python3 gateway_api_test_runner.py hands_configs
  python3 gateway_api_test_runner.py hands_select DexcelRobotics_Apex
  python3 gateway_api_test_runner.py frequency 50
  python3 gateway_api_test_runner.py upload /path/to/hand.zip [--overwrite]

完整说明见 docs/io_gateway_api.md

环境变量：GATEWAY_URL（默认 http://127.0.0.1:8080）
"""

from __future__ import annotations

import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

BASE = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080").rstrip("/")

COMMANDS = """
可用命令：
  status | bootstrap | probe | hands_configs | streams
  hands_select [hand ...]   # 无参数时 {"hands":[]} 清空手型
  frequency <hz>
  sync_start [hand] | sync_stop [hand]
  upload <zip_or_tar> [--overwrite]
  wifi_config | wifi_provision <ssid> <return_ip> [password]
  viz_urdf_exo | viz_urdf_hand <hand> [left|right]
""".strip()


def _print_response(status: int, raw: str) -> None:
    print(f"HTTP {status}")
    try:
        print(json.dumps(json.loads(raw), ensure_ascii=False, indent=2))
    except json.JSONDecodeError:
        print(raw[:4000] if len(raw) > 4000 else raw)


def _request(method: str, path: str, body: dict | None = None, raw_body: bytes | None = None, headers: dict | None = None) -> None:
    url = BASE + path
    hdrs = dict(headers or {})
    data = raw_body
    if body is not None:
        data = json.dumps(body).encode()
        hdrs.setdefault("Content-Type", "application/json")
    req = urllib.request.Request(url, data=data, method=method, headers=hdrs)
    try:
        with urllib.request.urlopen(req, timeout=15) as resp:
            _print_response(resp.status, resp.read().decode())
    except urllib.error.HTTPError as e:
        _print_response(e.code, e.read().decode())
    except urllib.error.URLError as e:
        sys.exit(f"请求失败（请先启动 gateway）: {e.reason}")


def _upload(file_path: Path, overwrite: bool) -> None:
    boundary = f"----{os.urandom(8).hex()}"
    parts: list[bytes] = []

    def field(name: str, value: str) -> None:
        parts.append(
            f'--{boundary}\r\nContent-Disposition: form-data; name="{name}"\r\n\r\n{value}\r\n'.encode()
        )

    content = file_path.read_bytes()
    parts.append(
        (
            f'--{boundary}\r\nContent-Disposition: form-data; name="files"; filename="{file_path.name}"\r\n'
            f"Content-Type: application/octet-stream\r\n\r\n"
        ).encode()
        + content
        + b"\r\n"
    )
    if overwrite:
        field("overwrite", "true")
    parts.append(f"--{boundary}--\r\n".encode())

    _request(
        "POST",
        "/api/v1/hands/configs/upload",
        raw_body=b"".join(parts),
        headers={"Content-Type": f"multipart/form-data; boundary={boundary}"},
    )


def main() -> None:
    args = sys.argv[1:]
    if not args or args[0] in ("-h", "--help", "help"):
        print(COMMANDS)
        return

    cmd = args[0]

    if cmd == "status":
        _request("GET", "/api/v1/status")
    elif cmd == "bootstrap":
        _request("GET", "/api/v1/bootstrap")
    elif cmd == "probe":
        _request("GET", "/api/v1/probe")
    elif cmd == "hands_configs":
        _request("GET", "/api/v1/hands/configs")
    elif cmd == "streams":
        _request("GET", "/api/v1/streams")
    elif cmd == "hands_select":
        _request("POST", "/api/v1/hands/select", {"hands": args[1:]})
    elif cmd == "frequency":
        if len(args) < 2:
            sys.exit("用法: frequency <hz>")
        _request("POST", "/api/v1/runtime/frequency", {"hz": int(args[1])})
    elif cmd == "sync_start":
        body = {"hand": args[1]} if len(args) > 1 else {}
        _request("POST", "/api/v1/control/sync/start", body)
    elif cmd == "sync_stop":
        body = {"hand": args[1]} if len(args) > 1 else {}
        _request("POST", "/api/v1/control/sync/stop", body)
    elif cmd == "wifi_config":
        _request("GET", "/api/v1/wifi/config")
    elif cmd == "wifi_provision":
        if len(args) < 3:
            sys.exit("用法: wifi_provision <ssid> <return_ip> [password]")
        body = {
            "ssid": args[1],
            "return_ip": args[2],
            "password": args[3] if len(args) > 3 else "",
        }
        _request("POST", "/api/v1/wifi/provision", body)
    elif cmd == "upload":
        if len(args) < 2:
            sys.exit("用法: upload <file> [--overwrite]")
        path = Path(args[1]).expanduser().resolve()
        if not path.is_file():
            sys.exit(f"文件不存在: {path}")
        _upload(path, overwrite="--overwrite" in args[2:])

    else:
        print(f"未知命令: {cmd}\n\n{COMMANDS}")
        sys.exit(1)


if __name__ == "__main__":
    main()
