#!/usr/bin/env python3
"""统一 VR 入口：Noetix M1 双臂 WebXR 遥操作。"""

from __future__ import annotations

import os
import sys


def _bootstrap() -> None:
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)
    noetix_dir = os.path.join(project_root, "backends", "noetix")
    if noetix_dir not in sys.path:
        sys.path.insert(0, noetix_dir)


def main() -> int:
    _bootstrap()
    from vr_teleop_dual import main as noetix_main

    return int(noetix_main())


if __name__ == "__main__":
    raise SystemExit(main())
