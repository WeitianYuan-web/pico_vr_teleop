"""ROS2 helpers: TF TCP lookup + /endposetarget_* publish + arm prepare."""

from __future__ import annotations

import time
from typing import Literal, Sequence

import numpy as np

Side = Literal["left", "right"]


class TianyiRosEndpose:
    """Requires sourced Humble/Jazzy env with eai_manipulator_msgs."""

    def __init__(
        self,
        *,
        from_frame: str = "waist_yaw_link",
        to_frame_left: str = "left_tcp_link",
        to_frame_right: str = "right_tcp_link",
        node_name: str = "tianyee_vr_endpose",
    ) -> None:
        import rclpy
        from geometry_msgs.msg import Pose
        from rclpy.node import Node
        from std_msgs.msg import Float64MultiArray, Header
        from tf2_ros import Buffer, TransformListener
        from eai_manipulator_msgs.msg import ArmTargetPose
        from std_srvs.srv import SetBool
        from eai_manipulator_msgs.srv import Mode

        if not rclpy.ok():
            rclpy.init()
        self._rclpy = rclpy
        self.node = Node(node_name)
        self.from_frame = from_frame
        self.to_frames = {"left": to_frame_left, "right": to_frame_right}
        self._buf = Buffer()
        self._listener = TransformListener(self._buf, self.node)
        self._pubs = {
            "left": self.node.create_publisher(ArmTargetPose, "/endposetarget_L", 10),
            "right": self.node.create_publisher(ArmTargetPose, "/endposetarget_R", 10),
        }
        from sensor_msgs.msg import JointState

        self._joint_pubs = {
            "left": self.node.create_publisher(Float64MultiArray, "/jointspace_commands_L", 10),
            "right": self.node.create_publisher(Float64MultiArray, "/jointspace_commands_R", 10),
        }
        self._joint_names = {
            "left": [
                "shoulder_pitch_l_joint",
                "shoulder_roll_l_joint",
                "shoulder_yaw_l_joint",
                "elbow_pitch_l_joint",
                "elbow_yaw_l_joint",
                "wrist_pitch_l_joint",
                "wrist_roll_l_joint",
            ],
            "right": [
                "shoulder_pitch_r_joint",
                "shoulder_roll_r_joint",
                "shoulder_yaw_r_joint",
                "elbow_pitch_r_joint",
                "elbow_yaw_r_joint",
                "wrist_pitch_r_joint",
                "wrist_roll_r_joint",
            ],
        }
        self._joint_pos: dict[str, float] = {}
        self._joint_vel: dict[str, float] = {}
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self._Float64MultiArray = Float64MultiArray
        self._Pose = Pose
        self._Header = Header
        self._ArmTargetPose = ArmTargetPose
        self._cli_en = self.node.create_client(SetBool, "/EAIHardware/set_arm_enable")
        self._cli_mode = self.node.create_client(Mode, "/EAIHardware/set_arm_mode")
        self._SetBool = SetBool
        self._Mode = Mode
        self._cli_switch = None
        self._SwitchController = None
        try:
            from controller_manager_msgs.srv import SwitchController

            self._SwitchController = SwitchController
            self._cli_switch = self.node.create_client(
                SwitchController, "/controller_manager/switch_controller"
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"SwitchController client unavailable: {exc}")

    def _on_joint_state(self, msg) -> None:  # noqa: ANN001
        names = [str(n) for n in msg.name]
        positions = [float(v) for v in msg.position]
        velocities = [float(v) for v in getattr(msg, "velocity", [])]
        for name, pos in zip(names, positions):
            self._joint_pos[name] = pos
        # Pass through robot-reported velocity only; missing/short arrays stay 0.
        if len(velocities) == len(names) and len(names) > 0:
            for name, vel in zip(names, velocities):
                self._joint_vel[name] = vel
        else:
            for name in names:
                self._joint_vel.setdefault(name, 0.0)

    def arm_joint_names(self, side: Side) -> list[str]:
        """Return the canonical seven-axis order used by Tianyee controllers."""
        return list(self._joint_names[side])

    def arm_q_snapshot(self, side: Side) -> list[float] | None:
        """Return cached joint positions without spinning or blocking the UDP loop."""
        names = self._joint_names[side]
        if not all(name in self._joint_pos for name in names):
            return None
        return [self._joint_pos[name] for name in names]

    def arm_dq_snapshot(self, side: Side) -> list[float] | None:
        """Return cached joint velocities (rad/s), matching arm_q_snapshot order."""
        names = self._joint_names[side]
        if not all(name in self._joint_pos for name in names):
            return None
        return [float(self._joint_vel.get(name, 0.0)) for name in names]

    def current_arm_q(self, side: Side, timeout_s: float = 2.0) -> list[float] | None:
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.spin_once(0.05)
            q = self.arm_q_snapshot(side)
            if q is not None:
                return q
        return None

    def spin_once(self, timeout_sec: float = 0.0) -> None:
        self._rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def wait_tf(self, side: Side, timeout_s: float = 8.0) -> tuple[np.ndarray, np.ndarray]:
        deadline = time.time() + timeout_s
        last_exc: Exception | None = None
        while time.time() < deadline:
            self.spin_once(0.05)
            try:
                return self.lookup_tcp(side)
            except Exception as exc:  # noqa: BLE001
                last_exc = exc
        raise RuntimeError(f"TF timeout {self.from_frame}->{self.to_frames[side]}: {last_exc}")

    def lookup_tcp(self, side: Side) -> tuple[np.ndarray, np.ndarray]:
        tf = self._buf.lookup_transform(
            self.from_frame,
            self.to_frames[side],
            self._rclpy.time.Time(),
        )
        t = tf.transform.translation
        r = tf.transform.rotation
        pos = np.array([t.x, t.y, t.z], dtype=float)
        quat_wxyz = np.array([r.w, r.x, r.y, r.z], dtype=float)
        return pos, quat_wxyz

    def publish_pose(self, side: Side, pos_m: np.ndarray, quat_wxyz: np.ndarray) -> None:
        msg = self._ArmTargetPose()
        msg.header = self._Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = self.from_frame
        msg.target.position.x = float(pos_m[0])
        msg.target.position.y = float(pos_m[1])
        msg.target.position.z = float(pos_m[2])
        w, x, y, z = [float(v) for v in quat_wxyz]
        msg.target.orientation.x = x
        msg.target.orientation.y = y
        msg.target.orientation.z = z
        msg.target.orientation.w = w
        msg.from_frame = self.from_frame
        msg.to_frame = self.to_frames[side]
        msg.offset_x = 0.0
        msg.offset_y = 0.0
        msg.offset_z = 0.0
        self._pubs[side].publish(msg)

    def publish_joints(self, side: Side, q7: Sequence[float]) -> None:
        msg = self._Float64MultiArray()
        msg.data = [float(v) for v in q7]
        self._joint_pubs[side].publish(msg)

    def _switch_controllers(
        self,
        *,
        deactivate: list[str],
        activate: list[str],
        timeout: float = 10.0,
    ) -> bool:
        """Prefer native /controller_manager/switch_controller (CLI often times out)."""
        if self._cli_switch is not None and self._SwitchController is not None:
            try:
                if not self._cli_switch.wait_for_service(timeout_sec=min(3.0, timeout)):
                    raise RuntimeError("switch_controller service not ready")
                req = self._SwitchController.Request()
                req.activate_controllers = list(activate)
                req.deactivate_controllers = list(deactivate)
                # BEST_EFFORT: don't fail if a named controller is already in desired state
                if hasattr(self._SwitchController.Request, "BEST_EFFORT"):
                    req.strictness = self._SwitchController.Request.BEST_EFFORT
                else:
                    req.strictness = 1
                if hasattr(req, "activate_asap"):
                    req.activate_asap = True
                if hasattr(req, "timeout"):
                    from builtin_interfaces.msg import Duration

                    sec = int(max(1.0, timeout))
                    req.timeout = Duration(sec=sec, nanosec=0)
                fut = self._cli_switch.call_async(req)
                self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout + 2.0)
                if not fut.done() or fut.result() is None:
                    raise RuntimeError("switch_controller no reply")
                ok = bool(getattr(fut.result(), "ok", True))
                self.node.get_logger().info(
                    f"switch_controller activate={activate} deactivate={deactivate} ok={ok}"
                )
                if ok:
                    return True
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"native switch_controller failed: {exc}")

        import subprocess

        cmd = ["ros2", "control", "switch_controllers"]
        if deactivate:
            cmd.append("--deactivate")
            cmd.extend(deactivate)
        if activate:
            cmd.append("--activate")
            cmd.extend(activate)
        try:
            r = subprocess.run(cmd, check=False, capture_output=True, text=True, timeout=timeout)
            out = (r.stdout or "") + (r.stderr or "")
            self.node.get_logger().info(f"{' '.join(cmd)} → rc={r.returncode} {out.strip()[:240]}")
            return r.returncode == 0
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"switch_controllers CLI failed: {exc}")
            return False

    def activate_jointspace_controllers(self) -> None:
        """Activate jointspace so /jointspace_commands_* can move the arms."""
        ok = self._switch_controllers(
            deactivate=[
                "endpose_single_arm_qp_L_controller",
                "endpose_single_arm_qp_R_controller",
            ],
            activate=[
                "jointspace_arm_L_controller",
                "jointspace_arm_R_controller",
            ],
        )
        if not ok:
            import subprocess

            try:
                subprocess.run(
                    [
                        "ros2",
                        "control",
                        "activate_controllers",
                        "jointspace_arm_L_controller",
                        "jointspace_arm_R_controller",
                    ],
                    check=False,
                    capture_output=True,
                    text=True,
                    timeout=8,
                )
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"activate jointspace failed: {exc}")

    def move_joints_ready(
        self,
        *,
        q_left: Sequence[float],
        q_right: Sequence[float],
        duration_s: float = 3.0,
        hz: float = 30.0,
    ) -> None:
        """Interpolate dual-arm joints to ready pose via jointspace controllers."""
        self.activate_jointspace_controllers()
        q0_l = self.current_arm_q("left") or list(q_left)
        q0_r = self.current_arm_q("right") or list(q_right)
        q1_l = [float(v) for v in q_left]
        q1_r = [float(v) for v in q_right]
        self.node.get_logger().info(
            f"joint home L0={[round(v, 3) for v in q0_l]} -> {[round(v, 3) for v in q1_l]}"
        )
        period = 1.0 / max(1.0, float(hz))
        steps = max(1, int(max(0.5, float(duration_s)) / period))
        for i in range(steps + 1):
            a = i / steps
            ql = [(1.0 - a) * q0_l[j] + a * q1_l[j] for j in range(7)]
            qr = [(1.0 - a) * q0_r[j] + a * q1_r[j] for j in range(7)]
            self.publish_joints("left", ql)
            self.publish_joints("right", qr)
            self.spin_once(0.0)
            time.sleep(period)
        # hold final
        for _ in range(int(0.4 / period)):
            self.publish_joints("left", q1_l)
            self.publish_joints("right", q1_r)
            self.spin_once(0.0)
            time.sleep(period)

    def activate_endpose_controllers(self) -> None:
        """Leave jointspace and activate Cartesian QP so /endposetarget_* takes effect."""
        # switch 偶发卡住；优先短超时，失败则直接 activate
        ok = self._switch_controllers(
            deactivate=[
                "jointspace_arm_L_controller",
                "jointspace_arm_R_controller",
            ],
            activate=[
                "endpose_single_arm_qp_L_controller",
                "endpose_single_arm_qp_R_controller",
            ],
            timeout=6.0,
        )
        if ok:
            return
        import subprocess

        try:
            r = subprocess.run(
                [
                    "ros2",
                    "control",
                    "activate_controllers",
                    "endpose_single_arm_qp_L_controller",
                    "endpose_single_arm_qp_R_controller",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=6,
            )
            out = (r.stdout or "") + (r.stderr or "")
            self.node.get_logger().info(
                f"activate_controllers endpose → rc={r.returncode} {out.strip()[:200]}"
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"activate_controllers failed: {exc}")

    def snapshot_tcp_targets(self) -> dict[Side, tuple[np.ndarray, np.ndarray]]:
        """Freeze both measured TCP poses for a controller hand-off."""
        targets: dict[Side, tuple[np.ndarray, np.ndarray]] = {}
        for side in ("left", "right"):
            try:
                pos, quat = self.lookup_tcp(side)  # type: ignore[arg-type]
                targets[side] = (pos.copy(), quat.copy())  # type: ignore[index]
            except Exception as exc:  # noqa: BLE001
                self.node.get_logger().warn(f"snapshot TCP {side} failed: {exc}")
        return targets

    def hold_endpose_targets(
        self,
        targets: dict[Side, tuple[np.ndarray, np.ndarray]],
        *,
        seconds: float = 0.4,
        hz: float = 30.0,
    ) -> None:
        """Repeatedly publish one frozen target; never chase a moving TCP."""
        period = 1.0 / max(1.0, float(hz))
        end = time.time() + max(0.15, float(seconds))
        while time.time() < end:
            for side, (pos, quat) in targets.items():
                try:
                    self.publish_pose(side, pos, quat)  # type: ignore[arg-type]
                except Exception:  # noqa: BLE001
                    pass
            self.spin_once(0.0)
            time.sleep(period)

    def hold_current_endpose(self, *, seconds: float = 0.4, hz: float = 30.0) -> None:
        """Snapshot current TCP once, then hold that fixed endpose target."""
        targets = self.snapshot_tcp_targets()
        self.hold_endpose_targets(targets, seconds=seconds, hz=hz)

    def _cli_service_call(self, argv: list[str], *, timeout: float) -> bool:
        """Fallback when in-process service clients miss discovery."""
        import subprocess

        cmd = ["ros2", "service", "call", *argv]
        try:
            r = subprocess.run(
                cmd,
                check=False,
                capture_output=True,
                text=True,
                timeout=max(5.0, timeout),
            )
            out = ((r.stdout or "") + (r.stderr or "")).strip()
            self.node.get_logger().info(f"{' '.join(cmd)} → rc={r.returncode} {out[:220]}")
            return r.returncode == 0
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"cli service call failed: {exc}")
            return False

    def call_enable(self, enable: bool, timeout: float = 20.0) -> None:
        # Spin a bit so discovery settles after bridge startup / service restarts.
        deadline = time.time() + min(8.0, timeout)
        while time.time() < deadline and not self._cli_en.service_is_ready():
            self.spin_once(0.1)

        if self._cli_en.wait_for_service(timeout_sec=min(5.0, timeout)):
            req = self._SetBool.Request()
            req.data = bool(enable)
            fut = self._cli_en.call_async(req)
            self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            if fut.done() and fut.result() is not None:
                self.node.get_logger().info(f"set_arm_enable({enable}): {fut.result().message}")
                return

        ok = self._cli_service_call(
            [
                "/EAIHardware/set_arm_enable",
                "std_srvs/srv/SetBool",
                f"{{data: {str(bool(enable)).lower()}}}",
            ],
            timeout=timeout,
        )
        if not ok:
            raise RuntimeError(
                "set_arm_enable unavailable — 机器人上需先启动 body_control 与 "
                "tianyi2_bringup（hardware:=real）"
            )

    def call_mode(self, mode: int = 3, timeout: float = 20.0) -> None:
        deadline = time.time() + min(8.0, timeout)
        while time.time() < deadline and not self._cli_mode.service_is_ready():
            self.spin_once(0.1)

        if self._cli_mode.wait_for_service(timeout_sec=min(5.0, timeout)):
            req = self._Mode.Request()
            req.mode = int(mode)
            fut = self._cli_mode.call_async(req)
            self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
            if fut.done() and fut.result() is not None:
                self.node.get_logger().info(f"set_arm_mode({mode}): {fut.result().info}")
                return

        ok = self._cli_service_call(
            [
                "/EAIHardware/set_arm_mode",
                "eai_manipulator_msgs/srv/Mode",
                f"{{mode: {int(mode)}}}",
            ],
            timeout=timeout,
        )
        if not ok:
            raise RuntimeError(
                "set_arm_mode unavailable — 请确认 body_control / XARM 已启动且 DDS 正常"
            )

    def enable_auto_switch(self) -> None:
        import subprocess

        try:
            subprocess.run(
                ["ros2", "control", "auto_switch_mode", "--enable"],
                check=False,
                capture_output=True,
                text=True,
                timeout=8,
            )
        except Exception as exc:  # noqa: BLE001
            self.node.get_logger().warn(f"auto_switch_mode failed: {exc}")

    def prepare_for_teleop(self, *, enable: bool = True, mode: int = 3) -> None:
        if enable:
            self.call_enable(True)
            self.call_mode(mode)
        self.enable_auto_switch()
        # Boot bridge skips --prepare; without active endpose controllers UDP poses do nothing.
        self.activate_endpose_controllers()
        self.hold_current_endpose(seconds=0.25)

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if self._rclpy.ok():
            self._rclpy.shutdown()
