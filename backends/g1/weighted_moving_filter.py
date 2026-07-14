"""兼容 shim：WeightedMovingFilter 已迁至 common.filters。"""

from __future__ import annotations

import os
import sys

_PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
if _PROJECT_ROOT not in sys.path:
    sys.path.insert(0, _PROJECT_ROOT)

from common.filters import WeightedMovingFilter

__all__ = ["WeightedMovingFilter"]
