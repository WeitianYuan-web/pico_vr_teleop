#!/usr/bin/env python3
"""统一 VR 入口：JAKA 双臂 WebXR servo_p 遥操作。"""

from __future__ import annotations

import os
import sys


def _bootstrap() -> None:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
    sdk_dir = os.path.join(project_root, "backends", "jaka", "sdk")
    if sdk_dir not in sys.path:
        sys.path.insert(0, sdk_dir)


def main() -> int:
    _bootstrap()
    from vr_teleop_dual import main as sdk_main

    return int(sdk_main())


if __name__ == "__main__":
    raise SystemExit(main())
