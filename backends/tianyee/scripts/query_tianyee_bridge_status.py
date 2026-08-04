#!/usr/bin/env python3
"""Query on-robot bridge health via UDP get_status (or SSH cat status.json)."""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys

import pexpect

DEFAULT_HOST = os.environ.get("TIANYEE_HOST", "192.168.41.1")
DEFAULT_USER = os.environ.get("TIANYEE_USER", "ubuntu")
DEFAULT_PASS = os.environ.get("TIANYEE_SSH_PASS", "123")
DEFAULT_PORT = int(os.environ.get("TIANYEE_UDP_PORT", "19011"))


def via_udp(host: str, port: int, timeout: float) -> dict:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(timeout)
    try:
        sock.sendto(json.dumps({"cmd": "get_status"}).encode("utf-8"), (host, port))
        data, _ = sock.recvfrom(65535)
        return json.loads(data.decode("utf-8"))
    except TimeoutError as exc:
        raise TimeoutError(
            f"UDP :{port} no reply — bridge may be restarting/stopped; "
            "try --ssh or: ssh robot 'systemctl status tianyee_udp_bridge'"
        ) from exc
    finally:
        sock.close()


def via_ssh(host: str, user: str, password: str) -> dict:
    target = f"{user}@{host}"
    child = pexpect.spawn(
        "ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
        f"-o PubkeyAuthentication=no {target} "
        "cat /home/ubuntu/pico_vr_teleop_tianyee/status.json",
        encoding="utf-8",
        timeout=30,
    )
    child.expect(["password:", "Password:"])
    child.sendline(password)
    child.expect(pexpect.EOF, timeout=30)
    text = (child.before or "").strip()
    # drop ssh banner noise: find first '{'
    i = text.find("{")
    if i < 0:
        raise RuntimeError(f"no status.json on robot:\n{text}")
    return json.loads(text[i:])


def main() -> int:
    p = argparse.ArgumentParser(description="Query Tianyi bridge robot status")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--port", type=int, default=DEFAULT_PORT)
    p.add_argument("--ssh", action="store_true", help="read status.json over SSH instead of UDP")
    p.add_argument("--user", default=DEFAULT_USER)
    p.add_argument("--password", default=DEFAULT_PASS)
    p.add_argument("--timeout", type=float, default=3.0)
    args = p.parse_args()
    try:
        if args.ssh:
            snap = via_ssh(args.host, args.user, args.password)
        else:
            snap = via_udp(args.host, args.port, args.timeout)
    except Exception as exc:  # noqa: BLE001
        print(f"[status] FAIL: {exc}", file=sys.stderr)
        return 1
    print(json.dumps(snap, ensure_ascii=False, indent=2))
    return 0 if snap.get("ok") else 2


if __name__ == "__main__":
    raise SystemExit(main())
