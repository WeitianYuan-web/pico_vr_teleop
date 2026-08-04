"""Lightweight robot health monitor for the on-robot UDP bridge."""

from __future__ import annotations

import json
import os
import subprocess
import time
from dataclasses import dataclass, field
from typing import Any


OFFLINE_ERROR = 33072


def _mem_available_mb() -> float | None:
    try:
        with open("/proc/meminfo", encoding="utf-8") as fh:
            for line in fh:
                if line.startswith("MemAvailable:"):
                    return float(line.split()[1]) / 1024.0
    except OSError:
        return None
    return None


def _systemctl_active(unit: str) -> str:
    try:
        r = subprocess.run(
            ["systemctl", "is-active", unit],
            check=False,
            capture_output=True,
            text=True,
            timeout=2,
        )
        return (r.stdout or r.stderr or "unknown").strip() or "unknown"
    except Exception:  # noqa: BLE001
        return "unknown"


@dataclass
class RobotStatusMonitor:
    """Subscribe /arm/status and periodically flush status.json."""

    ros: Any
    status_file: str
    period_s: float = 2.0
    _last_write: float = 0.0
    _last_arm_msg_t: float = 0.0
    _motor_errors: dict[int, int] = field(default_factory=dict)
    _prev_offline: int | None = None
    _arm_sub: Any = None

    def start(self) -> None:
        if not self.status_file:
            return
        try:
            from bodyctrl_msgs.msg import MotorStatusMsg
        except Exception as exc:  # noqa: BLE001
            print(f"[Status] bodyctrl_msgs unavailable, arm monitor disabled: {exc}")
            return

        def _on_arm(msg: Any) -> None:
            self._last_arm_msg_t = time.time()
            errs: dict[int, int] = {}
            for st in getattr(msg, "status", []) or []:
                try:
                    mid = int(getattr(st, "name", 0))
                    err = int(getattr(st, "error", 0))
                except Exception:  # noqa: BLE001
                    continue
                errs[mid] = err
            self._motor_errors = errs

        self._arm_sub = self.ros.node.create_subscription(
            MotorStatusMsg, "/arm/status", _on_arm, 10
        )
        print(f"[Status] monitoring /arm/status → {self.status_file}")

    def snapshot(self) -> dict[str, Any]:
        errs = dict(self._motor_errors)
        offline = [mid for mid, e in errs.items() if e == OFFLINE_ERROR]
        other = sorted({e for e in errs.values() if e not in (0, OFFLINE_ERROR)})
        age = None
        if self._last_arm_msg_t > 0:
            age = round(time.time() - self._last_arm_msg_t, 3)
        ok = bool(errs) and not offline and age is not None and age < 1.0
        return {
            "t": time.time(),
            "ok": ok,
            "arm": {
                "motors": len(errs),
                "offline_33072": len(offline),
                "offline_ids": offline,
                "other_errors": other,
                "status_age_s": age,
            },
            "services": {
                "set_arm_enable": bool(self.ros._cli_en.service_is_ready()),  # noqa: SLF001
            },
            "systemd": {
                "proc_manager": _systemctl_active("proc_manager.service"),
                "tianyee_xarm": _systemctl_active("tianyee_xarm.service"),
                "tianyee_udp_bridge": _systemctl_active("tianyee_udp_bridge.service"),
            },
            "mem_available_mb": _mem_available_mb(),
            "hint": (
                "ok"
                if ok
                else (
                    "arm motors offline (error=33072) — check body_control/CAN; "
                    "avoid PC Jazzy DDS storms"
                    if offline
                    else "waiting for /arm/status"
                )
            ),
        }

    def tick(self) -> None:
        if not self.status_file:
            return
        now = time.monotonic()
        if now - self._last_write < max(0.5, float(self.period_s)):
            return
        self._last_write = now
        snap = self.snapshot()
        offline_n = int(snap["arm"]["offline_33072"])
        if self._prev_offline is None or offline_n != self._prev_offline:
            print(
                f"[Status] arm motors={snap['arm']['motors']} "
                f"offline_33072={offline_n} mem_avail_mb={snap['mem_available_mb']} "
                f"proc_manager={snap['systemd']['proc_manager']}"
            )
            self._prev_offline = offline_n
        try:
            parent = os.path.dirname(self.status_file) or "."
            os.makedirs(parent, exist_ok=True)
            tmp = self.status_file + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                json.dump(snap, fh, ensure_ascii=False, separators=(",", ":"))
            os.replace(tmp, self.status_file)
        except OSError as exc:
            print(f"[Status] write failed: {exc}")
