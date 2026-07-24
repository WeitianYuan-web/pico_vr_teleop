"""InspireHandSDK_Y 驱动封装，默认 RH56F2。"""

from __future__ import annotations

import os
import sys
from typing import Any, List, Optional, Sequence

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
INSPIRE_BUILD_PYTHON_DIR = os.path.join(
    PROJECT_ROOT, "third_party", "InspireHandSDK_Y", "build", "python"
)

DEFAULT_MODEL = "rh56f2"
DEFAULT_FORCE = 6000
DEFAULT_SPEED = 4000


def load_inspire_binding():
    if os.path.isdir(INSPIRE_BUILD_PYTHON_DIR):
        sys.path.insert(0, INSPIRE_BUILD_PYTHON_DIR)
    try:
        import inspire_hand_py as ih
    except ImportError as exc:
        raise RuntimeError(
            "未找到 inspire_hand_py，请先编译 InspireHandSDK_Y Python 绑定：\n"
            "  cd third_party/InspireHandSDK_Y && "
            "cmake -B build -DINSPIRE_HAND_BUILD_PYTHON=ON && "
            "cmake --build build --target inspire_hand_py"
        ) from exc
    return ih


class InspireSdkHand:
    """单手：connect → start → submit_angles。"""

    def __init__(
        self,
        port: str,
        *,
        hand_id: int = 1,
        model: str = DEFAULT_MODEL,
        baudrate: int = 115200,
        control_hz: int = 100,
        io_hz: int = 30,
        force: int = DEFAULT_FORCE,
        speed: int = DEFAULT_SPEED,
    ) -> None:
        self.port = port
        self.hand_id = hand_id
        self.model = model
        self.baudrate = baudrate
        self.control_hz = control_hz
        self.io_hz = io_hz
        self.force = force
        self.speed = speed
        self._ih = load_inspire_binding()
        self.dev: Any = None

    @property
    def connected(self) -> bool:
        return self.dev is not None

    def connect(self) -> bool:
        if self.dev is not None:
            return True
        dev = self._ih.Hand(self.port, self.model)
        ok = dev.connect(
            hand_id=self.hand_id,
            baudrate=self.baudrate,
            control_hz=self.control_hz,
            io_hz=self.io_hz,
            force=self.force,
            speed=self.speed,
        )
        if not ok:
            try:
                dev.disconnect()
            except Exception:
                pass
            return False
        if not dev.start():
            try:
                dev.disconnect()
            except Exception:
                pass
            return False
        self.dev = dev
        return True

    def submit_angles(self, angles: Sequence[int]) -> bool:
        if self.dev is None:
            return False
        if len(angles) != 6:
            return False
        return bool(
            self.dev.submit_angles(
                list(angles), self.force, self.speed, True
            )
        )

    def get_angles(self) -> Optional[List[int]]:
        if self.dev is None:
            return None
        try:
            state = self.dev.get_state()
        except Exception:
            return None
        angles = state.get("angles")
        return list(angles) if angles else None

    def close(self) -> None:
        if self.dev is None:
            return
        try:
            self.dev.stop()
        except Exception:
            pass
        try:
            self.dev.disconnect()
        except Exception:
            pass
        self.dev = None
