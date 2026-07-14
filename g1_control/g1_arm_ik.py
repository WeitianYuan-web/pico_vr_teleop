"""G1 双臂 Pinocchio CLIK 逆解（无需 mesh / casadi）。"""

from __future__ import annotations

import os

import numpy as np
import pinocchio as pin

from config import ARM_JOINT_NAMES, DEFAULT_URDF_PATH, LEFT_EE_FRAME, RIGHT_EE_FRAME
from weighted_moving_filter import WeightedMovingFilter


class G1DualArmIK:
    """
    /**
     * @brief 基于 Pinocchio 的双臂阻尼最小二乘 IK
     *
     * 使用 g1_dual_arm.urdf（仅上肢 14 DoF），末端默认 left/right_rubber_hand。
     * 输出关节顺序与 G1_29_JointArmIndex / DDS 电机索引一致。
     */
    """

    def __init__(
        self,
        urdf_path: str | None = None,
        left_ee_frame: str = LEFT_EE_FRAME,
        right_ee_frame: str = RIGHT_EE_FRAME,
        damp: float = 1e-4,
        pos_weight: float = 1.0,
        rot_weight: float = 0.35,
        max_iter: int = 40,
        step_size: float = 0.5,
    ):
        self.urdf_path = os.path.abspath(urdf_path or DEFAULT_URDF_PATH)
        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(
                f"未找到 G1 URDF: {self.urdf_path}\n"
                "请将 g1_dual_arm.urdf 放到 g1_control/assets/ 下。"
            )

        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        if self.model.nq != 14:
            raise RuntimeError(f"期望 14 DoF 双臂模型，实际 nq={self.model.nq}")

        joint_names = [self.model.names[i] for i in range(1, self.model.njoints)]
        if tuple(joint_names) != ARM_JOINT_NAMES:
            print("[G1-IK] 警告: URDF 关节顺序与默认 ARM_JOINT_NAMES 不一致")
            print(f"  URDF: {joint_names}")

        self.left_ee_id = self.model.getFrameId(left_ee_frame)
        self.right_ee_id = self.model.getFrameId(right_ee_frame)
        self.damp = float(damp)
        self.pos_weight = float(pos_weight)
        self.rot_weight = float(rot_weight)
        self.max_iter = int(max_iter)
        self.step_size = float(step_size)
        self.q = pin.neutral(self.model)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)
        print(f"[G1-IK] 已加载 URDF: {self.urdf_path}")

    def forward_kinematics(self, q: np.ndarray) -> tuple[pin.SE3, pin.SE3]:
        """
        /**
         * @brief 计算左右末端位姿
         * @return (left_SE3, right_SE3)
         */
        """
        q = np.asarray(q, dtype=float).reshape(14)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.left_ee_id].copy(), self.data.oMf[self.right_ee_id].copy()

    def _frame_error(self, current: pin.SE3, target: pin.SE3) -> np.ndarray:
        # LOCAL_WORLD_ALIGNED: 位置误差在世界系，姿态用 log3
        pos_err = target.translation - current.translation
        rot_err = pin.log3(current.rotation.T @ target.rotation)
        return np.concatenate([self.pos_weight * pos_err, self.rot_weight * rot_err])

    def solve_ik(
        self,
        left_pose: np.ndarray | pin.SE3,
        right_pose: np.ndarray | pin.SE3,
        current_q: np.ndarray | None = None,
        current_dq: np.ndarray | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        /**
         * @brief 求解双臂关节目标
         * @param left_pose 左末端 4x4 或 pin.SE3（机器人躯干系）
         * @param right_pose 右末端 4x4 或 pin.SE3
         * @param current_q 当前双臂关节角 (14,)
         * @return (q_des, tau_ff)
         */
        """
        if current_q is not None:
            self.q = np.asarray(current_q, dtype=float).reshape(14).copy()

        left_tgt = left_pose if isinstance(left_pose, pin.SE3) else pin.SE3(left_pose)
        right_tgt = right_pose if isinstance(right_pose, pin.SE3) else pin.SE3(right_pose)

        q = self.q.copy()
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            pin.computeJointJacobians(self.model, self.data, q)

            err_l = self._frame_error(self.data.oMf[self.left_ee_id], left_tgt)
            err_r = self._frame_error(self.data.oMf[self.right_ee_id], right_tgt)
            err = np.concatenate([err_l, err_r])
            if float(np.linalg.norm(err)) < 1e-4:
                break

            J_l = pin.getFrameJacobian(
                self.model, self.data, self.left_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            J_r = pin.getFrameJacobian(
                self.model, self.data, self.right_ee_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
            )
            w = np.array(
                [self.pos_weight] * 3 + [self.rot_weight] * 3,
                dtype=float,
            )
            J = np.vstack([(w[:, None]) * J_l, (w[:, None]) * J_r])
            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + (self.damp**2) * np.eye(J.shape[0]), err)
            q = pin.integrate(self.model, q, self.step_size * dq)
            q = np.clip(q, self.model.lowerPositionLimit, self.model.upperPositionLimit)

        self.smooth_filter.add_data(q)
        sol_q = self.smooth_filter.filtered_data.copy()
        self.q = sol_q

        v = np.zeros(self.model.nv) if current_dq is None else np.asarray(current_dq, dtype=float)
        if v.shape != (self.model.nv,):
            v = np.zeros(self.model.nv)
        tau = pin.rnea(self.model, self.data, sol_q, v, np.zeros(self.model.nv))
        return sol_q, tau
