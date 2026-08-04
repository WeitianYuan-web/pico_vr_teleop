#!/usr/bin/env python3
"""Start body_control + tianyi2_bringup on the robot (Humble)."""

from __future__ import annotations

import os
import sys
import tempfile
import textwrap

import pexpect

DEFAULT_HOST = os.environ.get("TIANYEE_HOST", "192.168.41.1")
DEFAULT_USER = os.environ.get("TIANYEE_USER", "ubuntu")
DEFAULT_PASS = os.environ.get("TIANYEE_SSH_PASS", "123")


def main() -> int:
    host = f"{DEFAULT_USER}@{DEFAULT_HOST}"
    password = DEFAULT_PASS
    script = textwrap.dedent(
        """\
        #!/usr/bin/env bash
        set +u
        mkdir -p /tmp/xarm_run /tmp/xarm_run/ros_home
        export ROS_HOME=/tmp/xarm_run/ros_home
        source /opt/ros/humble/setup.bash
        source /home/ubuntu/ros2ws/install/setup.bash
        source /home/ubuntu/XARM/install/setup.bash
        export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/data/param/dds_profile.xml
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

        if ! pgrep -af body_control | grep -q body.launch; then
          echo "[stack] start body_control"
          nohup ros2 launch body_control body.launch.py > /tmp/xarm_run/body.log 2>&1 &
          echo BODY_PID=$!
          sleep 5
        else
          echo "[stack] body_control already running"
        fi

        if ! pgrep -af tianyi2.launch >/dev/null; then
          echo "[stack] start tianyi2 hardware:=real"
          nohup ros2 launch tianyi2_bringup tianyi2.launch.py hardware:=real \\
            > /tmp/xarm_run/xarm.log 2>&1 &
          echo XARM_PID=$!
        else
          echo "[stack] tianyi2 already running"
        fi

        for i in $(seq 1 36); do
          if timeout 3 ros2 service list 2>/dev/null | grep -q set_arm_enable; then
            echo "[stack] SERVICE_OK /EAIHardware/set_arm_enable"
            exit 0
          fi
          echo "[stack] wait set_arm_enable ($i) ..."
          sleep 5
        done
        echo "[stack] SERVICE_TIMEOUT"
        tail -n 30 /tmp/xarm_run/body.log 2>/dev/null || true
        tail -n 30 /tmp/xarm_run/xarm.log 2>/dev/null || true
        exit 1
        """
    )
    with tempfile.NamedTemporaryFile("w", delete=False, suffix="_stack.sh") as fh:
        fh.write(script)
        local = fh.name
    try:
        child = pexpect.spawn(
            "scp -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
            f"-o PubkeyAuthentication=no {local} {host}:/tmp/start_tianyee_stack.sh",
            encoding="utf-8",
            timeout=60,
        )
        child.expect(["password:", "Password:"])
        child.sendline(password)
        child.expect(pexpect.EOF, timeout=60)

        child = pexpect.spawn(
            "ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
            f"-o PubkeyAuthentication=no {host} bash /tmp/start_tianyee_stack.sh",
            encoding="utf-8",
            timeout=240,
            maxread=200000,
        )
        child.expect(["password:", "Password:"])
        child.sendline(password)
        child.expect(pexpect.EOF, timeout=240)
        print(child.before or "")
        return 0 if child.exitstatus in (0, None) else int(child.exitstatus or 1)
    finally:
        os.unlink(local)


if __name__ == "__main__":
    raise SystemExit(main())
