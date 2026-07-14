#!/usr/bin/env python3
"""统一 VR 入口：Piper 双臂双手 WebXR 遥操作。"""

from __future__ import annotations

import os
import sys


def _bootstrap() -> None:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
    teleop_dir = os.path.join(project_root, "backends", "piper", "teleop")
    if teleop_dir not in sys.path:
        sys.path.insert(0, teleop_dir)


def main() -> int:
    _bootstrap()
    from dual_arm_dual_hand_webxr import main as control_main

    control_main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
