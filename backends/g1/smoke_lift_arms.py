#!/usr/bin/env python3
"""
G1_29_ArmController 冒烟：缓慢抬起双臂再放下。

用法示例:
  # 运控 Regular + arm_sdk（推荐）
  python smoke_lift_arms.py --motion --network-interface enp12s0

  # Debug 模式（rt/lowcmd，会锁非臂关节）
  python smoke_lift_arms.py --network-interface enp12s0

  # 空跑
  python smoke_lift_arms.py --dry-run
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np

from g1_arm_controller import create_arm_controller


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="G1 双臂抬臂冒烟测试")
    p.add_argument("--network-interface", default=None, help="DDS 网卡，如 enp12s0 / enp2s0")
    p.add_argument("--motion", action="store_true", help="使用 rt/arm_sdk + weight")
    p.add_argument("--sim", action="store_true", help="仿真 DDS domain=1")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--hold-s", type=float, default=2.0, help="抬起后保持秒数")
    p.add_argument("--period-s", type=float, default=0.02)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    ctrl = create_arm_controller(
        dry_run=args.dry_run,
        motion_mode=args.motion,
        simulation_mode=args.sim,
        network_interface=args.network_interface,
        arm_velocity_limit=8.0,
    )

    # 目标：肩 roll / 肘 约 90°，其余 0（与官方 g1_arm7 示例类似）
    home = np.zeros(14, dtype=float)
    target = np.zeros(14, dtype=float)
    target[1] = math.pi / 2.0   # left shoulder roll
    target[3] = math.pi / 2.0   # left elbow
    target[8] = -math.pi / 2.0  # right shoulder roll
    target[10] = math.pi / 2.0  # right elbow

    try:
        print("[Smoke] 当前双臂 q:", np.round(ctrl.get_current_dual_arm_q(), 3))
        input("[Smoke] 确认周围安全后按 Enter 开始抬臂 ...")

        steps = int(3.0 / args.period_s)
        for i in range(steps + 1):
            alpha = i / steps
            q = (1.0 - alpha) * home + alpha * target
            ctrl.ctrl_dual_arm(q, np.zeros(14))
            time.sleep(args.period_s)
        print("[Smoke] 已抬起，保持", args.hold_s, "s")
        time.sleep(args.hold_s)

        print("[Smoke] 放下 ...")
        for i in range(steps + 1):
            alpha = i / steps
            q = (1.0 - alpha) * target + alpha * home
            ctrl.ctrl_dual_arm(q, np.zeros(14))
            time.sleep(args.period_s)

        print("[Smoke] 完成。当前 q:", np.round(ctrl.get_current_dual_arm_q(), 3))
    except KeyboardInterrupt:
        print("\n[Smoke] 用户中断")
    finally:
        ctrl.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
