#!/usr/bin/env python3
"""Sync tianyee UDP→ROS bridge to robot and start it (Humble + XARM)."""

from __future__ import annotations

import argparse
import os
import sys
import tempfile
import textwrap

import pexpect

DEFAULT_HOST = os.environ.get("TIANYEE_HOST", "192.168.41.1")
DEFAULT_USER = os.environ.get("TIANYEE_USER", "ubuntu")
DEFAULT_PASS = os.environ.get("TIANYEE_SSH_PASS", "123")
REMOTE_DIR = "/tmp/pico_vr_teleop_tianyee"


def _ssh(host: str, password: str, cmd: str, timeout: int = 60) -> str:
    child = pexpect.spawn(
        "ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
        f"-o PubkeyAuthentication=no {host} {cmd}",
        encoding="utf-8",
        timeout=timeout,
        maxread=200000,
    )
    child.expect(["password:", "Password:"])
    child.sendline(password)
    child.expect(pexpect.EOF, timeout=timeout)
    return (child.before or "").strip()


def _scp(host: str, password: str, src: str, dst: str, timeout: int = 120) -> None:
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


def main() -> int:
    p = argparse.ArgumentParser(description="Start Tianyi UDP ROS bridge on robot")
    p.add_argument("--prepare", action="store_true", help="enable/mode3/auto_switch")
    p.add_argument("--host", default=DEFAULT_HOST)
    p.add_argument("--user", default=DEFAULT_USER)
    p.add_argument("--password", default=DEFAULT_PASS)
    args = p.parse_args()

    project = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    target = f"{args.user}@{args.host}"
    prepare_flag = "--prepare" if args.prepare else ""

    print(f"[tianyee-bridge] sync → {target}:{REMOTE_DIR}")
    _ssh(target, args.password, f"mkdir -p {REMOTE_DIR}/backends {REMOTE_DIR}/common", timeout=30)
    for rel in ("backends/tianyee", "common"):
        src = os.path.join(project, rel)
        parent = "/".join(rel.split("/")[:-1])
        _scp(target, args.password, src, f"{REMOTE_DIR}/{parent}/")
        print(f"synced {rel}")

    start_script = textwrap.dedent(
        f"""\
        #!/usr/bin/env bash
        set +u
        export ROS_HOME=/tmp/xarm_run/ros_home
        mkdir -p "$ROS_HOME" /tmp/xarm_run
        source /opt/ros/humble/setup.bash
        source /home/ubuntu/ros2ws/install/setup.bash
        source /home/ubuntu/XARM/install/setup.bash
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/data/param/dds_profile.xml
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export PYTHONPATH={REMOTE_DIR}/backends/tianyee:{REMOTE_DIR}:{REMOTE_DIR}/common:${{PYTHONPATH:-}}
        pkill -f udp_ros_bridge.py 2>/dev/null || true
        sleep 0.5
        export PYTHONUNBUFFERED=1
        nohup python3 {REMOTE_DIR}/backends/tianyee/udp_ros_bridge.py {prepare_flag} \\
          > /tmp/xarm_run/udp_bridge.log 2>&1 &
        echo BRIDGE_PID=$!
        # prepare(enable/mode) may take several seconds before UDP bind
        for i in $(seq 1 30); do
          if grep -q "listening udp" /tmp/xarm_run/udp_bridge.log 2>/dev/null; then
            break
          fi
          sleep 0.5
        done
        tail -n 40 /tmp/xarm_run/udp_bridge.log || true
        """
    )
    with tempfile.NamedTemporaryFile("w", delete=False, suffix="_start_bridge.sh") as fh:
        fh.write(start_script)
        local_start = fh.name
    try:
        _scp(target, args.password, local_start, f"{REMOTE_DIR}/start_bridge.sh")
        print("synced start_bridge.sh")
        print("[tianyee-bridge] start on robot")
        out = _ssh(target, args.password, f"bash {REMOTE_DIR}/start_bridge.sh", timeout=120)
        print(out)
    finally:
        os.unlink(local_start)

    if "BRIDGE_PID=" not in out:
        print("[tianyee-bridge] warning: no BRIDGE_PID in output", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
