"""Noetix M1 ROS2 cartesian / MIT interface (status + command builders)."""

from __future__ import annotations

import os
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence

import yaml
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from config import (
    ARM_DOF,
    ARM_JOINTS,
    CARTESIAN_CMD_TOPIC,
    CONTROL_MODE_TOPIC,
    DEFAULT_HARDWARE_CONFIG,
    LEFT_GRIPPER_ID,
    LEG_BASE_ID,
    LEG_DOF,
    MOTOR_CMD_TOPIC,
    RIGHT_GRIPPER_ID,
    ROBOT_STATUS_TOPIC,
)

from noetix_m1.msg import (  # type: ignore[import-not-found]
    AgvVelocityCmd,
    CartesianAgvVelocityCmd,
    CartesianArmCartesianCmd,
    CartesianMitMotorCmd1,
    CartesianMitMotorCmd2,
    CartesianServoMotorCmd,
    MitMotorCmd,
    MotorCmdArray,
    RobotCartesianCmd,
    ServoMotorCmd,
    SetMode,
    StatusData,
)


@dataclass
class ArmCartesianPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    # q0..q3 == wxyz
    qw: float = 1.0
    qx: float = 0.0
    qy: float = 0.0
    qz: float = 0.0

    def to_cmd(self) -> CartesianArmCartesianCmd:
        cmd = CartesianArmCartesianCmd()
        cmd.x, cmd.y, cmd.z = float(self.x), float(self.y), float(self.z)
        cmd.q0, cmd.q1, cmd.q2, cmd.q3 = (
            float(self.qw),
            float(self.qx),
            float(self.qy),
            float(self.qz),
        )
        return cmd

    @classmethod
    def from_status(cls, arm_status) -> "ArmCartesianPose":
        return cls(
            x=float(arm_status.x),
            y=float(arm_status.y),
            z=float(arm_status.z),
            qw=float(arm_status.q0),
            qx=float(arm_status.q1),
            qy=float(arm_status.q2),
            qz=float(arm_status.q3),
        )

    def copy(self) -> "ArmCartesianPose":
        return ArmCartesianPose(
            self.x, self.y, self.z, self.qw, self.qx, self.qy, self.qz
        )


@dataclass
class RobotSnapshot:
    right_joints: List[float]
    left_joints: List[float]
    legs: List[float]
    right_ee: ArmCartesianPose
    left_ee: ArmCartesianPose
    right_grip: float
    left_grip: float
    work_mode: int = 0


@dataclass
class HardwareGains:
    node1_kp: List[float] = field(default_factory=lambda: [27.0] * ARM_DOF)
    node1_kd: List[float] = field(default_factory=lambda: [10.0] * ARM_DOF)
    node2_kp: List[float] = field(default_factory=lambda: [27.0] * ARM_DOF)
    node2_kd: List[float] = field(default_factory=lambda: [10.0] * ARM_DOF)
    node3_kp: List[float] = field(default_factory=lambda: [0.0] * ARM_DOF)
    node3_kd: List[float] = field(default_factory=lambda: [0.0] * ARM_DOF)
    node3_aux: List[float] = field(default_factory=lambda: [0.2, 125.0, 0.0, 0.05])
    gripper_kp: float = 10.0
    gripper_kd: float = 0.5
    gripper_min: float = -3.10
    gripper_max: float = 2.30

    @property
    def gripper_open(self) -> float:
        return float(self.gripper_max)

    @classmethod
    def from_yaml(cls, path: str) -> "HardwareGains":
        gains = cls()
        if not path or not os.path.exists(path):
            return gains
        with open(path, "r", encoding="utf-8") as fh:
            hw = yaml.safe_load(fh) or {}
        gains.node1_kp = list(hw.get("Node1_KP_Param", gains.node1_kp))
        gains.node1_kd = list(hw.get("Node1_KD_Param", gains.node1_kd))
        gains.node2_kp = list(hw.get("Node2_KP_Param", gains.node2_kp))
        gains.node2_kd = list(hw.get("Node2_KD_Param", gains.node2_kd))
        gains.node3_kp = list(hw.get("Node3_KP_Param", gains.node3_kp))
        gains.node3_kd = list(hw.get("Node3_KD_Param", gains.node3_kd))
        gains.node3_aux = list(hw.get("Node3_Param_AUX", gains.node3_aux))
        gains.gripper_kp = float(hw.get("Gripper_Kp", gains.gripper_kp))
        gains.gripper_kd = float(hw.get("Gripper_Kd", gains.gripper_kd))
        lim = hw.get("Node1_Motor8_Limits")
        if lim and len(lim) >= 2:
            gains.gripper_min = float(lim[0])
            gains.gripper_max = float(lim[1])
        return gains


class NoetixRosCartesian(Node):
    """Subscribe robot status; publish mode / MIT / cartesian commands."""

    def __init__(self, *, hardware_config: str = DEFAULT_HARDWARE_CONFIG) -> None:
        super().__init__("noetix_vr_teleop")
        self.gains = HardwareGains.from_yaml(hardware_config)
        self.work_mode = 0
        self.motor_pos: Dict[int, float] = {}
        self.motor_vel: Dict[int, float] = {}
        self.latest_right_ee: Optional[ArmCartesianPose] = None
        self.latest_left_ee: Optional[ArmCartesianPose] = None
        self.right_joints: Optional[List[float]] = None
        self.left_joints: Optional[List[float]] = None
        self.legs: Optional[List[float]] = None
        self.right_grip: float = self.gains.gripper_open
        self.left_grip: float = self.gains.gripper_open
        self._status_count = 0

        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            deadline=Duration(nanoseconds=10_000_000),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(nanoseconds=100_000_000),
            lifespan=Duration(nanoseconds=100_000_000),
        )
        cmd_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            deadline=Duration(nanoseconds=2_000_000),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(nanoseconds=100_000_000),
        )
        mode_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(nanoseconds=100_000_000),
        )

        self.create_subscription(StatusData, ROBOT_STATUS_TOPIC, self._on_status, status_qos)
        self.motor_pub = self.create_publisher(MotorCmdArray, MOTOR_CMD_TOPIC, cmd_qos)
        self.cart_pub = self.create_publisher(RobotCartesianCmd, CARTESIAN_CMD_TOPIC, cmd_qos)
        self.mode_pub = self.create_publisher(SetMode, CONTROL_MODE_TOPIC, mode_qos)

    def _on_status(self, msg: StatusData) -> None:
        self._status_count += 1
        self.work_mode = int(msg.work_mode)
        if len(msg.arms) >= 2:
            self.latest_right_ee = ArmCartesianPose.from_status(msg.arms[0])
            self.latest_left_ee = ArmCartesianPose.from_status(msg.arms[1])

        temp_right: List[Optional[float]] = [None] * ARM_JOINTS
        temp_left: List[Optional[float]] = [None] * ARM_JOINTS
        temp_legs: List[Optional[float]] = [None] * LEG_DOF
        right_grip = None
        left_grip = None
        for m in msg.motor_state_array.motor_states:
            mid = int(m.motor_id)
            pos = float(m.position)
            self.motor_pos[mid] = pos
            self.motor_vel[mid] = float(getattr(m, "velocity", 0.0) or 0.0)
            if 0 <= mid <= 6:
                temp_right[mid] = pos
            elif 8 <= mid <= 14:
                temp_left[mid - 8] = pos
            elif mid == RIGHT_GRIPPER_ID:
                right_grip = pos
            elif mid == LEFT_GRIPPER_ID:
                left_grip = pos
            elif LEG_BASE_ID <= mid < LEG_BASE_ID + LEG_DOF:
                temp_legs[mid - LEG_BASE_ID] = pos

        if all(v is not None for v in temp_right):
            self.right_joints = list(temp_right)  # type: ignore[arg-type]
        if all(v is not None for v in temp_left):
            self.left_joints = list(temp_left)  # type: ignore[arg-type]
        if all(v is not None for v in temp_legs):
            self.legs = list(temp_legs)  # type: ignore[arg-type]
        if right_grip is not None:
            self.right_grip = self._safe_grip(right_grip)
        if left_grip is not None:
            self.left_grip = self._safe_grip(left_grip)

    def _safe_grip(self, measured: float) -> float:
        if self.gains.gripper_min <= measured <= self.gains.gripper_max:
            return measured
        return self.gains.gripper_open

    def snapshot_ready(self) -> bool:
        return (
            self.right_joints is not None
            and self.left_joints is not None
            and self.legs is not None
            and self.latest_right_ee is not None
            and self.latest_left_ee is not None
        )

    def get_snapshot(self) -> Optional[RobotSnapshot]:
        if not self.snapshot_ready():
            return None
        assert self.right_joints is not None
        assert self.left_joints is not None
        assert self.legs is not None
        assert self.latest_right_ee is not None
        assert self.latest_left_ee is not None
        return RobotSnapshot(
            right_joints=list(self.right_joints),
            left_joints=list(self.left_joints),
            legs=list(self.legs),
            right_ee=self.latest_right_ee.copy(),
            left_ee=self.latest_left_ee.copy(),
            right_grip=float(self.right_grip),
            left_grip=float(self.left_grip),
            work_mode=int(self.work_mode),
        )

    def publish_mode(self, mode: int) -> None:
        msg = SetMode()
        msg.mode = int(mode)
        self.mode_pub.publish(msg)

    def _now_ms(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1_000_000)

    def build_motor_cmd(
        self,
        right: Sequence[float],
        left: Sequence[float],
        legs: Sequence[float],
        right_grip: float,
        left_grip: float,
    ) -> MotorCmdArray:
        msg = MotorCmdArray()
        msg.mit_motor_cmds = []
        msg.servo_motor_cmds = []

        for i in range(ARM_JOINTS):
            cmd = MitMotorCmd()
            cmd.motor_id = i
            cmd.position = float(right[i])
            cmd.velocity = 0.0
            cmd.torque = 0.0
            cmd.kp = float(self.gains.node1_kp[i])
            cmd.kd = float(self.gains.node1_kd[i])
            msg.mit_motor_cmds.append(cmd)

        rg = MitMotorCmd()
        rg.motor_id = RIGHT_GRIPPER_ID
        rg.position = float(right_grip)
        rg.velocity = 0.0
        rg.torque = 0.0
        rg.kp = float(self.gains.gripper_kp)
        rg.kd = float(self.gains.gripper_kd)
        msg.mit_motor_cmds.append(rg)

        for i in range(ARM_JOINTS):
            cmd = MitMotorCmd()
            cmd.motor_id = 8 + i
            cmd.position = float(left[i])
            cmd.velocity = 0.0
            cmd.torque = 0.0
            cmd.kp = float(self.gains.node2_kp[i])
            cmd.kd = float(self.gains.node2_kd[i])
            msg.mit_motor_cmds.append(cmd)

        lg = MitMotorCmd()
        lg.motor_id = LEFT_GRIPPER_ID
        lg.position = float(left_grip)
        lg.velocity = 0.0
        lg.torque = 0.0
        lg.kp = float(self.gains.gripper_kp)
        lg.kd = float(self.gains.gripper_kd)
        msg.mit_motor_cmds.append(lg)

        for i in range(LEG_DOF):
            motor_id = LEG_BASE_ID + i
            if len(msg.mit_motor_cmds) < 19:
                cmd = MitMotorCmd()
                cmd.motor_id = motor_id
                cmd.position = float(legs[i])
                cmd.velocity = 0.0
                cmd.torque = 0.0
                cmd.kp = float(self.gains.node3_kp[i])
                cmd.kd = float(self.gains.node3_kd[i])
                msg.mit_motor_cmds.append(cmd)
            else:
                cmd = ServoMotorCmd()
                cmd.motor_id = motor_id
                cmd.position = float(legs[i])
                cmd.position_kp = float(self.gains.node3_kp[i])
                cmd.position_kd = float(self.gains.node3_kd[i])
                cmd.velocity_limit = float(self.gains.node3_aux[0])
                cmd.velocity_kp = float(self.gains.node3_aux[1])
                cmd.velocity_kd = float(self.gains.node3_aux[2])
                cmd.velocity_ki = float(self.gains.node3_aux[3])
                msg.servo_motor_cmds.append(cmd)

        msg.agv_cmd = AgvVelocityCmd()
        msg.agv_cmd.x = msg.agv_cmd.y = msg.agv_cmd.w = 0.0
        msg.timestamp = self._now_ms()
        return msg

    def build_cartesian_cmd(
        self,
        right_ee: ArmCartesianPose,
        left_ee: ArmCartesianPose,
        right_grip: float,
        left_grip: float,
        legs_hold: Optional[Sequence[float]] = None,
    ) -> RobotCartesianCmd:
        cmd = RobotCartesianCmd()
        cmd.mit_motor_cmds1 = []
        for i in range(ARM_JOINTS):
            c = CartesianMitMotorCmd1()
            c.motor_id = i
            c.kp = float(self.gains.node1_kp[i])
            c.kd = float(self.gains.node1_kd[i])
            c.velocity = 0.0
            c.torque = 0.0
            cmd.mit_motor_cmds1.append(c)
        for i in range(ARM_JOINTS):
            c = CartesianMitMotorCmd1()
            c.motor_id = 8 + i
            c.kp = float(self.gains.node2_kp[i])
            c.kd = float(self.gains.node2_kd[i])
            c.velocity = 0.0
            c.torque = 0.0
            cmd.mit_motor_cmds1.append(c)

        cmd.mit_motor_cmds2 = []
        for motor_id, pos, kp, kd in (
            (RIGHT_GRIPPER_ID, right_grip, self.gains.gripper_kp, self.gains.gripper_kd),
            (LEFT_GRIPPER_ID, left_grip, self.gains.gripper_kp, self.gains.gripper_kd),
        ):
            c = CartesianMitMotorCmd2()
            c.motor_id = motor_id
            c.position = float(pos)
            c.velocity = 0.0
            c.torque = 0.0
            c.kp = float(kp)
            c.kd = float(kd)
            cmd.mit_motor_cmds2.append(c)

        for m_id in (16, 17, 18):
            idx = m_id - 16
            c = CartesianMitMotorCmd2()
            c.motor_id = m_id
            default = float(legs_hold[idx]) if legs_hold is not None else 0.0
            c.position = float(self.motor_pos.get(m_id, default))
            c.velocity = 0.0
            c.torque = 0.0
            c.kp = float(self.gains.node3_kp[idx])
            c.kd = float(self.gains.node3_kd[idx])
            cmd.mit_motor_cmds2.append(c)

        cmd.servo_motor_cmds = []
        for m_id in (19, 20, 21):
            idx = m_id - 16
            sc = CartesianServoMotorCmd()
            sc.motor_id = m_id
            default = float(legs_hold[idx]) if legs_hold is not None else 0.0
            sc.position = float(self.motor_pos.get(m_id, default))
            sc.position_kp = float(self.gains.node3_kp[idx])
            sc.position_kd = float(self.gains.node3_kd[idx])
            sc.velocity_limit = float(self.gains.node3_aux[0])
            sc.velocity_kp = float(self.gains.node3_aux[1])
            sc.velocity_kd = float(self.gains.node3_aux[2])
            sc.velocity_ki = float(self.gains.node3_aux[3])
            cmd.servo_motor_cmds.append(sc)

        cmd.arms = [right_ee.to_cmd(), left_ee.to_cmd()]
        cmd.agv_cmd = CartesianAgvVelocityCmd()
        cmd.agv_cmd.x = cmd.agv_cmd.y = cmd.agv_cmd.w = 0.0
        cmd.timestamp = self._now_ms()
        return cmd

    def publish_motor(
        self,
        right: Sequence[float],
        left: Sequence[float],
        legs: Sequence[float],
        right_grip: float,
        left_grip: float,
    ) -> None:
        self.motor_pub.publish(
            self.build_motor_cmd(right, left, legs, right_grip, left_grip)
        )

    def publish_cartesian(
        self,
        right_ee: ArmCartesianPose,
        left_ee: ArmCartesianPose,
        right_grip: float,
        left_grip: float,
        legs_hold: Optional[Sequence[float]] = None,
    ) -> None:
        self.cart_pub.publish(
            self.build_cartesian_cmd(
                right_ee, left_ee, right_grip, left_grip, legs_hold=legs_hold
            )
        )

    def wait_snapshot(self, timeout_s: float = 30.0) -> RobotSnapshot:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            # Caller should spin; this only polls cached fields.
            snap = self.get_snapshot()
            if snap is not None:
                return snap
            time.sleep(0.02)
        raise TimeoutError(
            f"Noetix snapshot timeout ({timeout_s:.1f}s): waiting /Robot_Status_Topic"
        )
