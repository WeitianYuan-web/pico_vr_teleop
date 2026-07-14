#!/usr/bin/env python3
"""统一 VR 入口：Unitree G1 双臂 WebXR 遥操作。"""

from __future__ import annotations

import os
import sys


def _bootstrap() -> None:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    g1_dir = os.path.join(project_root, "g1_control")
    if g1_dir not in sys.path:
        sys.path.insert(0, g1_dir)


def main() -> int:
    _bootstrap()
    from vr_teleop_dual import main as g1_main

    return int(g1_main())


if __name__ == "__main__":
    raise SystemExit(main())
