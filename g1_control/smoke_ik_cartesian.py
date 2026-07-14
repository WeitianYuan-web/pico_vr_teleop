#!/usr/bin/env python3
"""
G1 双臂 IK 末端笛卡尔位姿控制测试。

流程：读当前关节 → FK 得末端位姿 → 在笛卡尔空间插值目标 → CLIK → rt/arm_sdk。

用法:
  python smoke_ik_cartesian.py --motion --network-interface enp12s0
  python smoke_ik_cartesian.py --dry-run   # 仅验证 IK，不连真机
"""

from __future__ import annotations

import argparse
import time

import numpy as np
import pinocchio as pin

from g1_arm_controller import create_arm_controller
from g1_arm_ik import G1DualArmIK


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="G1 双臂 IK 笛卡尔末端测试")
    p.add_argument("--network-interface", default="enp12s0")
    p.add_argument("--motion", action="store_true", default=True)
    p.add_argument("--no-motion", action="store_true", help="不用 arm_sdk，改走 rt/lowcmd")
    p.add_argument("--sim", action="store_true")
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--dz", type=float, default=0.06, help="上抬高度 (m)")
    p.add_argument("--dy", type=float, default=0.04, help="左右外展位移 (m)")
    p.add_argument("--seg-s", type=float, default=3.0, help="每段运动时长 (s)")
    p.add_argument("--hold-s", type=float, default=1.0, help="段末保持 (s)")
    p.add_argument("--hz", type=float, default=50.0, help="控制频率")
    p.add_argument("--yes", action="store_true", help="跳过 Enter 确认")
    return p.parse_args()


def _fmt_xyz(p: np.ndarray) -> str:
    return f"({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})"


def _lerp_se3(a: pin.SE3, b: pin.SE3, t: float) -> pin.SE3:
    t = float(np.clip(t, 0.0, 1.0))
    # 位置线性；姿态保持起点（本测试只动平移，更安全）
    out = a.copy()
    out.translation = (1.0 - t) * a.translation + t * b.translation
    out.rotation = a.rotation.copy()
    return out


def _offset_pose(pose: pin.SE3, dxyz: np.ndarray) -> pin.SE3:
    out = pose.copy()
    out.translation = pose.translation + np.asarray(dxyz, dtype=float)
    return out


def _run_segment(
    *,
    name: str,
    ctrl,
    ik: G1DualArmIK,
    left0: pin.SE3,
    right0: pin.SE3,
    left1: pin.SE3,
    right1: pin.SE3,
    duration_s: float,
    hz: float,
) -> tuple[float, float]:
    """
    /**
     * @brief 笛卡尔插值一段，返回左右末端最大位置误差 (m)
     */
    """
    period = 1.0 / max(1.0, hz)
    steps = max(1, int(duration_s / period))
    max_err_l = 0.0
    max_err_r = 0.0
    print(f"\n[IK] === {name} ===  {steps} steps @ {hz:.0f}Hz")
    print(f"  L {_fmt_xyz(left0.translation)} -> {_fmt_xyz(left1.translation)}")
    print(f"  R {_fmt_xyz(right0.translation)} -> {_fmt_xyz(right1.translation)}")

    for i in range(steps + 1):
        alpha = i / steps
        tgt_l = _lerp_se3(left0, left1, alpha)
        tgt_r = _lerp_se3(right0, right1, alpha)

        q_now = ctrl.get_current_dual_arm_q()
        dq_now = ctrl.get_current_dual_arm_dq()
        sol_q, sol_tau = ik.solve_ik(tgt_l, tgt_r, q_now, dq_now)
        ctrl.ctrl_dual_arm(sol_q, sol_tau)

        # 用实测关节做 FK，评估跟踪误差
        q_meas = ctrl.get_current_dual_arm_q()
        meas_l, meas_r = ik.forward_kinematics(q_meas)
        err_l = float(np.linalg.norm(meas_l.translation - tgt_l.translation))
        err_r = float(np.linalg.norm(meas_r.translation - tgt_r.translation))
        max_err_l = max(max_err_l, err_l)
        max_err_r = max(max_err_r, err_r)

        if i % max(1, steps // 5) == 0 or i == steps:
            print(
                f"  [{alpha*100:3.0f}%] "
                f"L_meas={_fmt_xyz(meas_l.translation)} err={err_l*1000:.1f}mm | "
                f"R_meas={_fmt_xyz(meas_r.translation)} err={err_r*1000:.1f}mm"
            )
        time.sleep(period)

    return max_err_l, max_err_r


def main() -> int:
    args = parse_args()
    motion = bool(args.motion) and not bool(args.no_motion)

    ik = G1DualArmIK()
    ctrl = create_arm_controller(
        dry_run=args.dry_run,
        motion_mode=motion,
        simulation_mode=args.sim,
        network_interface=None if args.dry_run else args.network_interface,
        arm_velocity_limit=12.0,
    )

    try:
        q0 = ctrl.get_current_dual_arm_q()
        left0, right0 = ik.forward_kinematics(q0)
        print("[IK] 当前双臂 q:", np.round(q0, 3))
        print("[IK] 当前左末端:", _fmt_xyz(left0.translation))
        print("[IK] 当前右末端:", _fmt_xyz(right0.translation))
        print(
            f"[IK] 测试位移: dz={args.dz:.3f}m, dy=±{args.dy:.3f}m, "
            f"seg={args.seg_s:.1f}s, motion={motion}"
        )

        if not args.yes and not args.dry_run:
            input("[IK] 确认周围安全、运控 Regular 站立后按 Enter 开始 ...")

        # 段1: 双臂上抬
        left_up = _offset_pose(left0, np.array([0.0, 0.0, args.dz]))
        right_up = _offset_pose(right0, np.array([0.0, 0.0, args.dz]))
        e1 = _run_segment(
            name="上抬 +Z",
            ctrl=ctrl,
            ik=ik,
            left0=left0,
            right0=right0,
            left1=left_up,
            right1=right_up,
            duration_s=args.seg_s,
            hz=args.hz,
        )
        time.sleep(args.hold_s)

        # 段2: 左右外展（左 +Y，右 -Y，躯干系 Y 向左）
        left_out = _offset_pose(left_up, np.array([0.0, args.dy, 0.0]))
        right_out = _offset_pose(right_up, np.array([0.0, -args.dy, 0.0]))
        e2 = _run_segment(
            name="外展 ±Y",
            ctrl=ctrl,
            ik=ik,
            left0=left_up,
            right0=right_up,
            left1=left_out,
            right1=right_out,
            duration_s=args.seg_s,
            hz=args.hz,
        )
        time.sleep(args.hold_s)

        # 段3: 回到初始末端位姿
        q_now = ctrl.get_current_dual_arm_q()
        left_cur, right_cur = ik.forward_kinematics(q_now)
        e3 = _run_segment(
            name="回初始位姿",
            ctrl=ctrl,
            ik=ik,
            left0=left_cur,
            right0=right_cur,
            left1=left0,
            right1=right0,
            duration_s=args.seg_s,
            hz=args.hz,
        )

        q_f = ctrl.get_current_dual_arm_q()
        left_f, right_f = ik.forward_kinematics(q_f)
        print("\n[IK] 完成")
        print("[IK] 最终左末端:", _fmt_xyz(left_f.translation), "相对起点 Δ", _fmt_xyz(left_f.translation - left0.translation))
        print("[IK] 最终右末端:", _fmt_xyz(right_f.translation), "相对起点 Δ", _fmt_xyz(right_f.translation - right0.translation))
        print(
            f"[IK] 段内最大跟踪误差 L/R (mm): "
            f"{e1[0]*1000:.1f}/{e1[1]*1000:.1f}, "
            f"{e2[0]*1000:.1f}/{e2[1]*1000:.1f}, "
            f"{e3[0]*1000:.1f}/{e3[1]*1000:.1f}"
        )
    except KeyboardInterrupt:
        print("\n[IK] 用户中断")
    finally:
        ctrl.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
