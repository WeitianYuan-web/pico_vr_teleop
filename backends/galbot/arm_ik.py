"""Local Galbot dual-arm IK (Pinocchio DLS). Used for 1.7 teleop servo.

HPU Motion.inverse_kinematics is a planning RPC (~50–150 ms). Streaming that
solution at 50 Hz still only updates the target at IK rate, which feels like
stutter. This solver runs in-process from the calibrated golf URDF.
"""

from __future__ import annotations

import os
from typing import Iterable

import numpy as np
import pinocchio as pin

from config import (
    ARM_JOINT_NAMES,
    EE_FRAMES,
    LEFT_ARM_JOINT_NAMES,
    RIGHT_ARM_JOINT_NAMES,
    SUPPORT_JOINT_NAMES,
)

# XCU PVT 比 URDF 更紧。超这个发 set_joint_commands 会 FAULT。
_CONTROLLER_LIMITS: dict[str, tuple[float, float]] = {
    "right_arm_joint4": (-1.869862177, 2.50),
}
# 只挡住再往硬限位顶。0.10 比这组预备位两肩剩下的外展余量还宽，
# 会把已经合法的 j2 往回拽。
_HARD_KEEPOUT_RAD = 0.03


def _pose_to_se3(xyz: np.ndarray, quat_wxyz: np.ndarray) -> pin.SE3:
    qw, qx, qy, qz = (float(v) for v in quat_wxyz)
    return pin.SE3(
        pin.Quaternion(qw, qx, qy, qz).toRotationMatrix(),
        np.asarray(xyz, dtype=float).reshape(3),
    )


class GalbotLocalArmIK:
    def __init__(self, urdf_path: str) -> None:
        path = os.path.abspath(urdf_path)
        if not os.path.isfile(path):
            raise FileNotFoundError(path)
        self.urdf_path = path
        self.model = pin.buildModelFromUrdf(path)
        self.data = self.model.createData()
        self.q = pin.neutral(self.model)
        self._ee_id = {
            side: self.model.getFrameId(frame) for side, frame in EE_FRAMES.items()
        }
        missing = [s for s, i in self._ee_id.items() if i >= self.model.nframes]
        if missing:
            raise RuntimeError(f"URDF 缺少末端 frame: {missing}")
        self._idx_q: dict[str, int] = {}
        self._idx_v: dict[str, int] = {}
        self._lo: dict[str, float] = {}
        self._hi: dict[str, float] = {}
        for name in ARM_JOINT_NAMES + SUPPORT_JOINT_NAMES:
            jid = self.model.getJointId(name)
            if jid >= self.model.njoints:
                raise RuntimeError(f"URDF 缺少关节 {name}")
            joint = self.model.joints[jid]
            self._idx_q[name] = int(joint.idx_q)
            self._idx_v[name] = int(joint.idx_v)
            lo = float(self.model.lowerPositionLimit[joint.idx_q])
            hi = float(self.model.upperPositionLimit[joint.idx_q])
            if name in _CONTROLLER_LIMITS:
                clo, chi = _CONTROLLER_LIMITS[name]
                lo = max(lo, float(clo))
                hi = min(hi, float(chi))
            self._lo[name] = lo
            self._hi[name] = hi
        self._cmd_lo = {n: self._lo[n] + _HARD_KEEPOUT_RAD for n in self._lo}
        self._cmd_hi = {n: self._hi[n] - _HARD_KEEPOUT_RAD for n in self._hi}
        # 兼容旧字段名：streamer / 测试若仍读 soft_*，等同于硬限位 keepout。
        self._soft_lo = self._cmd_lo
        self._soft_hi = self._cmd_hi
        self._arm_v = {
            "left": np.array([self._idx_v[n] for n in LEFT_ARM_JOINT_NAMES], dtype=int),
            "right": np.array([self._idx_v[n] for n in RIGHT_ARM_JOINT_NAMES], dtype=int),
        }
        self._arm_q = {
            "left": np.array([self._idx_q[n] for n in LEFT_ARM_JOINT_NAMES], dtype=int),
            "right": np.array([self._idx_q[n] for n in RIGHT_ARM_JOINT_NAMES], dtype=int),
        }
        self.last_pos_err_m = 0.0
        print(
            f"[Galbot] 本地 pinocchio IK: {path} "
            f"(nq={self.model.nq}, L_ee={EE_FRAMES['left']}, R_ee={EE_FRAMES['right']})"
        )

    def set_named_positions(self, names: Iterable[str], values: Iterable[float]) -> None:
        for name, val in zip(names, values):
            idx = self._idx_q.get(name)
            if idx is None:
                continue
            lo = self._lo[name]
            hi = self._hi[name]
            self.q[idx] = float(np.clip(float(val), lo, hi))

    def set_arm(self, side: str, q7: list[float] | None) -> None:
        names = LEFT_ARM_JOINT_NAMES if side == "left" else RIGHT_ARM_JOINT_NAMES
        if not q7 or len(q7) < 7:
            return
        self.set_named_positions(names, q7[:7])

    def arm_q(self, side: str) -> list[float]:
        idxs = self._arm_q[side]
        return [float(self.q[i]) for i in idxs]

    def clamp_arm(self, side: str, q7: list[float] | None) -> list[float] | None:
        if not q7 or len(q7) < 7:
            return None
        names = LEFT_ARM_JOINT_NAMES if side == "left" else RIGHT_ARM_JOINT_NAMES
        out = []
        for name, val in zip(names, q7[:7]):
            out.append(float(np.clip(float(val), self._cmd_lo[name], self._cmd_hi[name])))
        return out

    def fk_xyz_wxyz(self, side: str) -> tuple[np.ndarray, np.ndarray]:
        pin.forwardKinematics(self.model, self.data, self.q)
        pin.updateFramePlacements(self.model, self.data)
        T = self.data.oMf[self._ee_id[side]]
        quat = pin.Quaternion(T.rotation)
        return (
            np.array(T.translation, dtype=float),
            np.array([quat.w, quat.x, quat.y, quat.z], dtype=float),
        )

    def solve(
        self,
        side: str,
        xyz: np.ndarray,
        quat_wxyz: np.ndarray,
        *,
        n_iter: int = 4,
        ori_weight: float = 0.35,
        step_gain: float = 0.7,
    ) -> list[float]:
        """Incremental DLS for teleop: a few damped steps, not a full planner solve."""
        target = _pose_to_se3(xyz, quat_wxyz)
        frame_id = self._ee_id[side]
        v_idx = self._arm_v[side]
        q_idx = self._arm_q[side]
        names = LEFT_ARM_JOINT_NAMES if side == "left" else RIGHT_ARM_JOINT_NAMES
        last_pos_err = 0.0
        for _ in range(max(1, n_iter)):
            pin.forwardKinematics(self.model, self.data, self.q)
            pin.updateFramePlacements(self.model, self.data)
            pin.computeJointJacobians(self.model, self.data, self.q)
            cur = self.data.oMf[frame_id]
            err = np.zeros(6, dtype=float)
            err[:3] = np.asarray(target.translation, dtype=float) - np.asarray(
                cur.translation, dtype=float
            )
            err[3:] = pin.log3(target.rotation @ cur.rotation.T)
            last_pos_err = float(np.linalg.norm(err[:3]))
            if last_pos_err < 8e-4 and float(np.linalg.norm(err[3:])) < 2e-2:
                break
            # 位置跟不上时先别拧腕去保 6D；近限位构型上那会走出另一套冗余解。
            ori_w = float(
                ori_weight * np.exp(-max(0.0, last_pos_err - 0.012) / 0.025)
            )
            w = np.array([1.0, 1.0, 1.0, ori_w, ori_w, ori_w], dtype=float)
            J = pin.getFrameJacobian(
                self.model, self.data, frame_id, pin.LOCAL_WORLD_ALIGNED
            )[:, v_idx]
            qnow = np.array([float(self.q[int(q_idx[i])]) for i in range(7)], dtype=float)
            wj = np.ones(7, dtype=float)
            for i, name in enumerate(names):
                room = min(qnow[i] - self._lo[name], self._hi[name] - qnow[i])
                if room < 0.10:
                    wj[i] = max(0.30, room / 0.10)
            W = np.diag(wj)
            jw = (J @ W) * w[:, np.newaxis]
            errw = err * w
            smin = float(np.linalg.svd(jw, compute_uv=False)[-1])
            lam = 4e-2 if smin < 0.08 else 5e-3
            dq = W @ (jw.T @ np.linalg.solve(jw @ jw.T + lam * np.eye(6), errw))
            dq *= float(step_gain)
            dq = np.clip(dq, -0.06, 0.06)
            for i, name in enumerate(names):
                qi = int(q_idx[i])
                qv = float(self.q[qi])
                dqi = float(dq[i])
                lo = self._cmd_lo[name]
                hi = self._cmd_hi[name]
                if dqi > 0.0:
                    room = hi - qv
                    if room < 0.08:
                        dqi *= max(0.0, room / 0.08)
                elif dqi < 0.0:
                    room = qv - lo
                    if room < 0.08:
                        dqi *= max(0.0, room / 0.08)
                self.q[qi] = float(np.clip(qv + dqi, lo, hi))
        self.last_pos_err_m = last_pos_err
        return self.arm_q(side)
