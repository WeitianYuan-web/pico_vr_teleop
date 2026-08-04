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
# Persistent install path (prefer). Temporary path kept for one-shot debug.
REMOTE_DIR = os.environ.get("TIANYEE_BRIDGE_HOME", "/home/ubuntu/pico_vr_teleop_tianyee")
FALLBACK_TMP_DIR = "/tmp/pico_vr_teleop_tianyee"


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

    project = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
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
        # Prefer persistent systemd service if installed
        if systemctl list-unit-files 2>/dev/null | grep -q '^tianyee_udp_bridge.service'; then
          echo "[tianyee-bridge] restart systemd tianyee_udp_bridge.service"
          echo {args.password} | sudo -S systemctl restart tianyee_udp_bridge.service
          for i in $(seq 1 40); do
            if [[ -f {REMOTE_DIR}/status.json ]] || \\
               grep -q "listening udp" {REMOTE_DIR}/logs/bridge.service.log 2>/dev/null; then
              systemctl --no-pager --full status tianyee_udp_bridge.service | head -25 || true
              echo BRIDGE_READY=1
              exit 0
            fi
            sleep 1
          done
          systemctl --no-pager --full status tianyee_udp_bridge.service | head -40 || true
          tail -n 40 {REMOTE_DIR}/logs/bridge.service.log 2>/dev/null || true
          echo BRIDGE_READY=0
          exit 21
        fi

        export ROS_HOME=/tmp/xarm_run/ros_home
        mkdir -p "$ROS_HOME" /tmp/xarm_run {REMOTE_DIR}/logs
        source /opt/ros/humble/setup.bash
        source /home/ubuntu/ros2ws/install/setup.bash
        source /home/ubuntu/XARM/install/setup.bash
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/data/param/dds_profile.xml
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        export PYTHONPATH={REMOTE_DIR}/backends/tianyee:{REMOTE_DIR}:{REMOTE_DIR}/common:${{PYTHONPATH:-}}
        if [[ -n "{prepare_flag}" ]]; then
          if ! timeout 5 ros2 service list 2>/dev/null | grep -qx '/EAIHardware/set_arm_enable'; then
            echo "[tianyee-bridge] ERROR: /EAIHardware/set_arm_enable unavailable"
            echo "[tianyee-bridge] Run ./scripts/run_tianyee_robot_stack.sh first"
            echo "[tianyee-bridge] Or install autostart: ./scripts/run_tianyee_bridge_install.sh"
            exit 20
          fi
        fi
        pkill -f udp_ros_bridge.py 2>/dev/null || true
        sleep 0.5
        export PYTHONUNBUFFERED=1
        nohup python3 {REMOTE_DIR}/backends/tianyee/udp_ros_bridge.py {prepare_flag} \\
          --status-file {REMOTE_DIR}/status.json \\
          > {REMOTE_DIR}/logs/udp_bridge.log 2>&1 &
        bridge_pid=$!
        echo BRIDGE_PID=$bridge_pid
        bridge_ready=0
        for i in $(seq 1 30); do
          if grep -q "listening udp" {REMOTE_DIR}/logs/udp_bridge.log 2>/dev/null; then
            bridge_ready=1
            break
          fi
          if ! kill -0 "$bridge_pid" 2>/dev/null; then
            break
          fi
          sleep 0.5
        done
        tail -n 40 {REMOTE_DIR}/logs/udp_bridge.log || true
        if [[ "$bridge_ready" -eq 1 ]] && kill -0 "$bridge_pid" 2>/dev/null; then
          echo BRIDGE_READY=1
          exit 0
        fi
        echo BRIDGE_READY=0
        exit 21
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

    if "BRIDGE_READY=1" not in out:
        print("[tianyee-bridge] failed: bridge did not become ready", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
