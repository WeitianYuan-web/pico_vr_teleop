"""G1 双臂 Placo QP IK（末端位姿 + 肘部外展正则 + 关节限位）。"""

from __future__ import annotations

import os
import re
import tempfile
from pathlib import Path

import numpy as np
import pinocchio as pin

from config import ARM_JOINT_NAMES, DEFAULT_URDF_PATH, LEFT_EE_FRAME, RIGHT_EE_FRAME
from weighted_moving_filter import WeightedMovingFilter

try:
    import placo
except ImportError as exc:  # pragma: no cover
    raise ImportError("需要安装 placo 才能使用 G1 Placo IK") from exc


# /**
#  * @brief 偏好关节姿态：双手略抬、置于身体前方，肘部外展
#  *
#  * shoulder_pitch < 0 → 手臂前伸并抬起（躯干系 +X / +Z）
#  * 左肩 roll > 0、右肩 roll < 0 → 肘向量偏外侧
#  * 肘关节约 0.8 rad 保持弯曲
#  */
PREFERRED_JOINTS: dict[str, float] = {
    "left_shoulder_pitch_joint": -0.20,
    "left_shoulder_roll_joint": 0.28,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_joint": 0.5,
    "left_wrist_roll_joint": 0.0,
    "left_wrist_pitch_joint": 0.0,
    "left_wrist_yaw_joint": 0.0,
    "right_shoulder_pitch_joint": -0.20,
    "right_shoulder_roll_joint": -0.28,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_joint": 0.5,
    "right_wrist_roll_joint": 0.0,
    "right_wrist_pitch_joint": 0.0,
    "right_wrist_yaw_joint": 0.0,
}

LEFT_ELBOW_FRAME = "left_elbow_link"
RIGHT_ELBOW_FRAME = "right_elbow_link"


def _strip_mesh_blocks(urdf_text: str) -> str:
    """去掉 visual/collision，避免依赖 STL mesh。"""
    text = re.sub(r"<visual\b[^>]*>.*?</visual>", "", urdf_text, flags=re.S | re.I)
    text = re.sub(r"<collision\b[^>]*>.*?</collision>", "", text, flags=re.S | re.I)
    return text


def _prepare_urdf_for_placo(urdf_path: str) -> str:
    src = Path(urdf_path).resolve()
    text = _strip_mesh_blocks(src.read_text(encoding="utf-8"))
    tmp = tempfile.NamedTemporaryFile(
        prefix="g1_placo_",
        suffix=".urdf",
        delete=False,
        dir=str(src.parent),
    )
    tmp_path = Path(tmp.name)
    tmp.close()
    tmp_path.write_text(text, encoding="utf-8")
    return str(tmp_path)


class G1DualArmIK:
    """
    /**
     * @brief G1 双臂 Placo QP 逆解
     *
     * - 双末端 frame task（位姿）
     * - joints regularization：偏好肘部外展姿态，抑制肘部翻转
     * - 可选肘部位置软约束：左右肘分别偏向 +Y / -Y
     * - enable_joint_limits / velocity_limits：减轻限位附近抖动
     * - 单臂模式：mask 未激活侧 DoF，关闭其对侧软任务，避免双臂耦合抖动
     *
     * 对外 API 与旧版 Pinocchio CLIK 兼容：solve_ik / forward_kinematics。
     */
    """

    def __init__(
        self,
        urdf_path: str | None = None,
        left_ee_frame: str = LEFT_EE_FRAME,
        right_ee_frame: str = RIGHT_EE_FRAME,
        dt: float = 0.02,
        position_weight: float = 5.0,
        orientation_weight: float = 0.8,
        joints_regularization_weight: float = 5e-3,
        elbow_out_weight: float = 0.15,
        elbow_out_offset_m: float = 0.08,
        manipulability_weight: float = 1e-3,
        max_joint_step_rad: float = 0.08,
        prefer_joints: dict[str, float] | None = None,
    ):
        self.urdf_path = os.path.abspath(urdf_path or DEFAULT_URDF_PATH)
        if not os.path.isfile(self.urdf_path):
            raise FileNotFoundError(f"未找到 G1 URDF: {self.urdf_path}")

        self._placo_urdf = _prepare_urdf_for_placo(self.urdf_path)
        self.left_ee_frame = left_ee_frame
        self.right_ee_frame = right_ee_frame
        self.dt = float(dt)
        self.prefer_joints = dict(prefer_joints or PREFERRED_JOINTS)
        self.elbow_out_offset_m = float(elbow_out_offset_m)
        self.max_joint_step_rad = float(max_joint_step_rad)

        self._w_pos = float(position_weight)
        self._w_ori = float(orientation_weight)
        self._w_elbow = float(elbow_out_weight)
        self._w_manip = float(manipulability_weight)
        self._w_joints = float(joints_regularization_weight)

        # FK 用 pinocchio 纯模型（无 floating base），与电机 14 维一致
        self.model = pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        if self.model.nq != 14:
            raise RuntimeError(f"期望 14 DoF 双臂模型，实际 nq={self.model.nq}")
        self.left_ee_id = self.model.getFrameId(left_ee_frame)
        self.right_ee_id = self.model.getFrameId(right_ee_frame)

        self.robot = placo.RobotWrapper(self._placo_urdf)
        self.solver = placo.KinematicsSolver(self.robot)
        self.solver.dt = self.dt
        self.solver.mask_fbase(True)
        self.solver.enable_joint_limits(True)
        self.solver.enable_velocity_limits(True)

        self.robot.update_kinematics()
        left_T = np.array(self.robot.get_T_world_frame(left_ee_frame), dtype=float)
        right_T = np.array(self.robot.get_T_world_frame(right_ee_frame), dtype=float)

        # 位置任务（更高权重）+ 姿态任务
        self.left_pos_task = self.solver.add_position_task(left_ee_frame, left_T[:3, 3])
        self.left_pos_task.configure("left_ee_pos", "soft", self._w_pos)
        self.left_frame_task = self.solver.add_frame_task(left_ee_frame, left_T)
        self.left_frame_task.configure("left_ee_pose", "soft", self._w_ori)

        self.right_pos_task = self.solver.add_position_task(right_ee_frame, right_T[:3, 3])
        self.right_pos_task.configure("right_ee_pos", "soft", self._w_pos)
        self.right_frame_task = self.solver.add_frame_task(right_ee_frame, right_T)
        self.right_frame_task.configure("right_ee_pose", "soft", self._w_ori)

        # 肘部外展：软位置任务，目标 = 当前肘位 + 外侧偏移（每步相对当前刷新）
        self.elbow_out_weight = self._w_elbow
        self.left_elbow_task = None
        self.right_elbow_task = None
        if self._w_elbow > 0.0:
            left_elb = np.array(self.robot.get_T_world_frame(LEFT_ELBOW_FRAME)[:3, 3], dtype=float)
            right_elb = np.array(self.robot.get_T_world_frame(RIGHT_ELBOW_FRAME)[:3, 3], dtype=float)
            self.left_elbow_task = self.solver.add_position_task(LEFT_ELBOW_FRAME, left_elb)
            self.left_elbow_task.configure("left_elbow_out", "soft", self._w_elbow)
            self.right_elbow_task = self.solver.add_position_task(RIGHT_ELBOW_FRAME, right_elb)
            self.right_elbow_task.configure("right_elbow_out", "soft", self._w_elbow)

        self.left_manip = None
        self.right_manip = None
        if self._w_manip > 0.0:
            self.left_manip = self.solver.add_manipulability_task(left_ee_frame, "both", 1.0)
            self.left_manip.configure("left_manip", "soft", self._w_manip)
            self.right_manip = self.solver.add_manipulability_task(right_ee_frame, "both", 1.0)
            self.right_manip.configure("right_manip", "soft", self._w_manip)

        self.joints_task = self.solver.add_joints_task()
        self.joints_task.configure("joints_regularization", "soft", self._w_joints)
        # Placo joint_names 可能含 root；只正则化臂关节
        self._joint_names = [n for n in self.robot.joint_names() if n in ARM_JOINT_NAMES]
        if len(self._joint_names) != 14:
            # fallback：按 ARM_JOINT_NAMES 顺序
            self._joint_names = list(ARM_JOINT_NAMES)
        self._left_joint_names = self._joint_names[0:7]
        self._right_joint_names = self._joint_names[7:14]
        self.joints_task.set_joints(
            {name: float(self.prefer_joints.get(name, 0.0)) for name in self._joint_names}
        )

        # floating base 占 7，臂关节从 q[7:]
        n = len(self._joint_names)
        self._q_slice = slice(7, 7 + n)
        self.joint_lower = np.array(self.robot.model.lowerPositionLimit[self._q_slice], dtype=float)
        self.joint_upper = np.array(self.robot.model.upperPositionLimit[self._q_slice], dtype=float)
        # 限位内缩，避免贴边抖动
        margin = 0.05
        self.joint_lower = self.joint_lower + margin
        self.joint_upper = self.joint_upper - margin

        self.q = np.array(
            [self.prefer_joints.get(name, 0.0) for name in ARM_JOINT_NAMES], dtype=float
        )
        # 稍长窗口，减轻单帧尖峰（单臂/双臂均受益）
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), 14)
        self._lock_left = False
        self._lock_right = False
        self._sync_robot_state(self.q)
        print(
            f"[G1-IK] Placo QP 已加载 URDF: {self.urdf_path} "
            f"(joints_reg={joints_regularization_weight}, elbow_out={elbow_out_weight})"
        )

    def _sync_robot_state(self, q14: np.ndarray) -> None:
        q14 = np.asarray(q14, dtype=float).reshape(14)
        self.robot.state.q[self._q_slice] = q14
        self.robot.update_kinematics()

    def _configure_side_tasks(self, side: str, active: bool) -> None:
        """按侧开关末端/肘/可操作度软任务权重。"""
        w_pos = self._w_pos if active else 0.0
        w_ori = self._w_ori if active else 0.0
        w_elb = self._w_elbow if active else 0.0
        w_man = self._w_manip if active else 0.0
        if side == "left":
            self.left_pos_task.configure("left_ee_pos", "soft", w_pos)
            self.left_frame_task.configure("left_ee_pose", "soft", w_ori)
            if self.left_elbow_task is not None:
                self.left_elbow_task.configure("left_elbow_out", "soft", w_elb)
            if self.left_manip is not None:
                self.left_manip.configure("left_manip", "soft", w_man)
        else:
            self.right_pos_task.configure("right_ee_pos", "soft", w_pos)
            self.right_frame_task.configure("right_ee_pose", "soft", w_ori)
            if self.right_elbow_task is not None:
                self.right_elbow_task.configure("right_elbow_out", "soft", w_elb)
            if self.right_manip is not None:
                self.right_manip.configure("right_manip", "soft", w_man)

    def _apply_arm_locks(self, lock_left: bool, lock_right: bool) -> None:
        """
        /**
         * @brief 单臂遥操时 mask 对侧 DoF，并关闭其对侧软任务，切断双臂 QP 耦合
         */
        """
        if lock_left == self._lock_left and lock_right == self._lock_right:
            return

        if lock_left and not self._lock_left:
            for name in self._left_joint_names:
                self.solver.mask_dof(name)
        elif (not lock_left) and self._lock_left:
            for name in self._left_joint_names:
                self.solver.unmask_dof(name)

        if lock_right and not self._lock_right:
            for name in self._right_joint_names:
                self.solver.mask_dof(name)
        elif (not lock_right) and self._lock_right:
            for name in self._right_joint_names:
                self.solver.unmask_dof(name)

        self._configure_side_tasks("left", active=not lock_left)
        self._configure_side_tasks("right", active=not lock_right)
        self._lock_left = bool(lock_left)
        self._lock_right = bool(lock_right)

    def forward_kinematics(self, q: np.ndarray) -> tuple[pin.SE3, pin.SE3]:
        """
        /**
         * @brief 计算左右末端位姿（Pinocchio FK，躯干系）
         */
        """
        q = np.asarray(q, dtype=float).reshape(14)
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.left_ee_id].copy(), self.data.oMf[self.right_ee_id].copy()

    def _se3_to_T(self, pose: np.ndarray | pin.SE3) -> np.ndarray:
        if isinstance(pose, pin.SE3):
            T = np.eye(4, dtype=float)
            T[:3, :3] = pose.rotation
            T[:3, 3] = pose.translation
            return T
        return np.asarray(pose, dtype=float).reshape(4, 4)

    def _update_elbow_out_targets(self) -> None:
        if self.left_elbow_task is None or self.right_elbow_task is None:
            return
        # 仅刷新仍激活侧，避免对锁臂持续注入外侧误差
        if not self._lock_left:
            left_elb = np.array(self.robot.get_T_world_frame(LEFT_ELBOW_FRAME)[:3, 3], dtype=float)
            left_tgt = left_elb.copy()
            left_tgt[1] = left_elb[1] + self.elbow_out_offset_m
            self.left_elbow_task.target_world = left_tgt
        if not self._lock_right:
            right_elb = np.array(self.robot.get_T_world_frame(RIGHT_ELBOW_FRAME)[:3, 3], dtype=float)
            right_tgt = right_elb.copy()
            right_tgt[1] = right_elb[1] - self.elbow_out_offset_m
            self.right_elbow_task.target_world = right_tgt

    def solve_ik(
        self,
        left_pose: np.ndarray | pin.SE3,
        right_pose: np.ndarray | pin.SE3,
        current_q: np.ndarray | None = None,
        current_dq: np.ndarray | None = None,
        *,
        lock_left: bool = False,
        lock_right: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        """
        /**
         * @brief 求解双臂关节目标
         * @param lock_left 锁定左臂 DoF（单臂右遥操时用）
         * @param lock_right 锁定右臂 DoF（单臂左遥操时用）
         * @return (q_des[14], tau_ff[14])
         */
        """
        q_ref = self.q.copy() if current_q is None else np.asarray(current_q, dtype=float).reshape(14).copy()
        self.q = q_ref
        self._apply_arm_locks(lock_left, lock_right)
        self._sync_robot_state(self.q)

        left_T = self._se3_to_T(left_pose)
        right_T = self._se3_to_T(right_pose)
        if not lock_left:
            self.left_frame_task.T_world_frame = left_T
            self.left_pos_task.target_world = left_T[:3, 3]
        if not lock_right:
            self.right_frame_task.T_world_frame = right_T
            self.right_pos_task.target_world = right_T[:3, 3]

        # 正则目标：偏好姿态与当前状态混合；锁侧钉在当前关节，避免被 prefer 拉走
        # 激活侧降低 blend，减轻与末端任务对抗造成的卡顿
        blend_active = 0.12
        reg = {}
        for i, name in enumerate(self._joint_names):
            locked = (i < 7 and lock_left) or (i >= 7 and lock_right)
            if locked:
                reg[name] = float(self.q[i])
            else:
                prefer = float(self.prefer_joints.get(name, 0.0))
                reg[name] = (1.0 - blend_active) * float(self.q[i]) + blend_active * prefer
        self.joints_task.set_joints(reg)
        self._update_elbow_out_targets()

        try:
            self.solver.solve(True)
            sol_q = np.array(self.robot.state.q[self._q_slice], dtype=float)
        except RuntimeError as exc:
            print(f"[G1-IK] Placo solve 失败，保持上一解: {exc}")
            sol_q = self.q.copy()

        # 锁侧硬保持（mask 已保证，这里再保险 + 同步滤波器）
        if lock_left:
            sol_q[0:7] = q_ref[0:7]
        if lock_right:
            sol_q[7:14] = q_ref[7:14]

        sol_q = np.clip(sol_q, self.joint_lower, self.joint_upper)

        # 激活侧逐步限幅，抑制偶发大步长导致的卡顿/抖动
        if self.max_joint_step_rad > 0.0:
            delta = sol_q - q_ref
            max_step = self.max_joint_step_rad
            if lock_left:
                delta[0:7] = 0.0
            if lock_right:
                delta[7:14] = 0.0
            abs_d = np.abs(delta)
            over = abs_d > max_step
            if np.any(over):
                delta[over] = np.sign(delta[over]) * max_step
            sol_q = q_ref + delta
            if lock_left:
                sol_q[0:7] = q_ref[0:7]
            if lock_right:
                sol_q[7:14] = q_ref[7:14]

        self.smooth_filter.add_data(sol_q)
        sol_q = self.smooth_filter.filtered_data.copy()
        if lock_left:
            self.smooth_filter.pin_indices(slice(0, 7), q_ref[0:7])
            sol_q[0:7] = q_ref[0:7]
        if lock_right:
            self.smooth_filter.pin_indices(slice(7, 14), q_ref[7:14])
            sol_q[7:14] = q_ref[7:14]

        self.q = sol_q
        self._sync_robot_state(sol_q)

        v = np.zeros(self.model.nv) if current_dq is None else np.asarray(current_dq, dtype=float)
        if v.shape != (self.model.nv,):
            v = np.zeros(self.model.nv)
        tau = pin.rnea(self.model, self.data, sol_q, v, np.zeros(self.model.nv))
        return sol_q, tau

    def preferred_q(self) -> np.ndarray:
        """返回偏好关节向量（用于回 home / 初始对齐）。"""
        return np.array([self.prefer_joints.get(n, 0.0) for n in ARM_JOINT_NAMES], dtype=float)
