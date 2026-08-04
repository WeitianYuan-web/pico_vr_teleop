#!/usr/bin/env python3
"""Install Tianyi UDP bridge onto the robot permanently + enable systemd autostart."""

from __future__ import annotations

import argparse
import os
import shlex
import sys
import tempfile
import textwrap

import pexpect

DEFAULT_HOST = os.environ.get("TIANYEE_HOST", "192.168.41.1")
DEFAULT_USER = os.environ.get("TIANYEE_USER", "ubuntu")
DEFAULT_PASS = os.environ.get("TIANYEE_SSH_PASS", "123")
REMOTE_DIR = "/home/ubuntu/pico_vr_teleop_tianyee"
SERVICE_NAME = "tianyee_udp_bridge.service"
XARM_SERVICE_NAME = "tianyee_xarm.service"
SERVICES = (XARM_SERVICE_NAME, SERVICE_NAME)


def _ssh(host: str, password: str, cmd: str, timeout: int = 120) -> str:
    # Quote whole remote command so pipes/&& survive pexpect/shlex splitting.
    remote = shlex.quote(cmd)
    child = pexpect.spawn(
        "ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
        f"-o PubkeyAuthentication=no {host} {remote}",
        encoding="utf-8",
        timeout=timeout,
        maxread=400000,
    )
    child.expect(["password:", "Password:"])
    child.sendline(password)
    child.expect(pexpect.EOF, timeout=timeout)
    return (child.before or "").strip()


def _scp(host: str, password: str, src: str, dst: str, timeout: int = 180) -> None:
    child = pexpect.spawn(
        "scp -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
        f"-o PubkeyAuthentication=no -r {src} {host}:{dst}",
        encoding="utf-8",
        timeout=timeout,
    )
    child.expect(["password:", "Password:"])
    child.sendline(password)
    child.expect(pexpect.EOF, timeout=timeout)
    if child.exitstatus not in (0, None):
        raise RuntimeError(f"scp failed: {src} -> {dst}\n{child.before}")


def _ssh_sudo(host: str, password: str, cmd: str, timeout: int = 120) -> str:
    wrapped = f"echo {shlex.quote(password)} | sudo -S -p '' bash -lc {shlex.quote(cmd)}"
    return _ssh(host, password, wrapped, timeout=timeout)


def main() -> int:
    p = argparse.ArgumentParser(description="Install on-robot Tianyi UDP bridge + systemd")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--user", default=DEFAULT_USER)
    p.add_argument("--password", default=DEFAULT_PASS)
    p.add_argument("--no-start", action="store_true", help="install/enable but do not start now")
    p.add_argument("--uninstall", action="store_true", help="disable/remove systemd unit only")
    args = p.parse_args()

    project = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
    target = f"{args.user}@{args.host}"

    if args.uninstall:
        print(f"[install] uninstall services on {target}")
        cmds = " && ".join(
            [
                f"systemctl disable --now {name} 2>/dev/null || true"
                for name in SERVICES
            ]
            + [
                f"rm -f /etc/systemd/system/{name}"
                for name in SERVICES
            ]
            + ["systemctl daemon-reload", "echo UNINSTALLED"]
        )
        out = _ssh_sudo(target, args.password, cmds, timeout=60)
        print(out)
        return 0

    print(f"[install] sync → {target}:{REMOTE_DIR}")
    _ssh(
        target,
        args.password,
        f"mkdir -p {REMOTE_DIR}/backends {REMOTE_DIR}/common {REMOTE_DIR}/logs "
        f"{REMOTE_DIR}/robot_service",
        timeout=30,
    )
    for rel in ("backends/tianyee", "common"):
        src = os.path.join(project, rel)
        parent = "/".join(rel.split("/")[:-1])
        _scp(target, args.password, src, f"{REMOTE_DIR}/{parent}/")
        print(f"  synced {rel}")

    svc_src = os.path.join(project, "backends/tianyee/robot_service")
    _scp(target, args.password, svc_src, f"{REMOTE_DIR}/")
    print("  synced robot_service/")

    # Ensure launcher executable
    _ssh(
        target,
        args.password,
        f"chmod +x {REMOTE_DIR}/robot_service/run_bridge.sh "
        f"{REMOTE_DIR}/robot_service/wait_proc_manager_running.sh",
        timeout=20,
    )

    print(f"[install] install systemd {XARM_SERVICE_NAME} + {SERVICE_NAME}")
    # Also try revive official body stack if it was OOM-killed earlier.
    revive = (
        "systemctl reset-failed proc_manager.service 2>/dev/null || true; "
        "systemctl start proc_manager.service 2>/dev/null || true; "
    )
    copy_units = " && ".join(
        f"cp {REMOTE_DIR}/robot_service/{name} /etc/systemd/system/{name}"
        for name in SERVICES
    )
    enable = " && ".join(f"systemctl enable {name}" for name in SERVICES)
    if args.no_start:
        start = "true"
    else:
        # A code update only needs a bridge restart.  Starting an already-active
        # XARM unit is a no-op; restarting it interrupts ROS contexts and can
        # leave the bridge stopped when the chained command returns non-zero.
        # Force-kill only a stuck bridge (ros2 context shutdown can block stop).
        start = (
            f"systemctl stop {SERVICE_NAME} 2>/dev/null || true; "
            f"systemctl kill -s SIGKILL {SERVICE_NAME} 2>/dev/null || true; "
            f"pkill -9 -f 'pico_vr_teleop_tianyee/backends/tianyee/udp_ros_bridge' 2>/dev/null || true; "
            f"systemctl reset-failed {SERVICE_NAME} 2>/dev/null || true; "
            f"systemctl start {XARM_SERVICE_NAME} && "
            f"systemctl start {SERVICE_NAME}"
        )
    status = (
        f"systemctl --no-pager --full status {XARM_SERVICE_NAME} | head -25; "
        f"systemctl --no-pager --full status {SERVICE_NAME} | head -25"
    )
    out = _ssh_sudo(
        target,
        args.password,
        f"{revive}{copy_units} && systemctl daemon-reload && {enable} && "
        f"{start} && {status} && echo INSTALL_OK",
        timeout=180,
    )
    print(out)

    # Quick status probe after a short wait
    probe = textwrap.dedent(
        f"""\
        sleep 3
        echo '=== status.json ==='
        if [[ -f {REMOTE_DIR}/status.json ]]; then
          cat {REMOTE_DIR}/status.json
          echo
        else
          echo '(not written yet — bridge may still be waiting for XARM/arm)'
          tail -n 30 {REMOTE_DIR}/logs/bridge.service.log 2>/dev/null || true
        fi
        """
    )
    with tempfile.NamedTemporaryFile("w", delete=False, suffix="_probe.sh") as fh:
        fh.write(probe)
        local = fh.name
    try:
        _scp(target, args.password, local, f"{REMOTE_DIR}/_probe_status.sh")
        print(_ssh(target, args.password, f"bash {REMOTE_DIR}/_probe_status.sh", timeout=60))
    finally:
        os.unlink(local)

    if "INSTALL_OK" not in out and "Active:" not in out:
        print("[install] warning: systemd status unclear; check robot logs", file=sys.stderr)
        return 1
    print(
        f"[install] done. On robot: systemctl status {SERVICE_NAME}; "
        f"cat {REMOTE_DIR}/status.json"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
