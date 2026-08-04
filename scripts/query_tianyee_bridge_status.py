#!/usr/bin/env python3
"""Compatibility wrapper → backends/tianyee/scripts/query_tianyee_bridge_status.py."""

from __future__ import annotations

import os
import runpy
import sys


def main() -> int:
    project = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    target = os.path.join(
        project, "backends", "tianyee", "scripts", "query_tianyee_bridge_status.py"
    )
    sys.argv[0] = target
    runpy.run_path(target, run_name="__main__")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
