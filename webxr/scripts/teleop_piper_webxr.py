#!/usr/bin/env python3
"""兼容入口：已迁移至 backends/piper/teleop/teleop_piper_webxr.py。"""

from __future__ import annotations

import os
import runpy
import sys

_TARGET = os.path.abspath(
    os.path.join(
        os.path.dirname(__file__),
        "..",
        "..",
        "backends",
        "piper",
        "teleop",
        "teleop_piper_webxr.py",
    )
)
print(f"[compat] webxr/scripts/ 已迁移，转发到 {_TARGET}", file=sys.stderr)
runpy.run_path(_TARGET, run_name="__main__")
