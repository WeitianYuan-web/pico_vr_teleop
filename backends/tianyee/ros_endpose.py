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
        self.node.create_subscription(JointState, "/joint_states", self._on_joint_state, 50)
        self._Float64MultiArray = Float64MultiArray
        self._Pose = Pose
        self._Header = Header
        self._ArmTargetPose = ArmTargetPose
        self._cli_en = self.node.create_client(SetBool, "/EAIHardware/set_arm_enable")
        self._cli_mode = self.node.create_client(Mode, "/EAIHardware/set_arm_mode")
        self._SetBool = SetBool
        self._Mode = Mode

    def _on_joint_state(self, msg) -> None:  # noqa: ANN001
        for name, pos in zip(msg.name, msg.position):
            self._joint_pos[str(name)] = float(pos)

    def current_arm_q(self, side: Side, timeout_s: float = 2.0) -> list[float] | None:
        names = self._joint_names[side]
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            self.spin_once(0.05)
            if all(n in self._joint_pos for n in names):
                return [self._joint_pos[n] for n in names]
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
            self.node.get_logger().warn(f"switch_controllers failed: {exc}")
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

    def hold_current_endpose(self, *, seconds: float = 0.4, hz: float = 30.0) -> None:
        """Publish current TCP on /endposetarget_* to claim endpose control."""
        period = 1.0 / max(1.0, float(hz))
        end = time.time() + max(0.15, float(seconds))
        while time.time() < end:
            for side in ("left", "right"):
                try:
                    pos, quat = self.lookup_tcp(side)  # type: ignore[arg-type]
                    self.publish_pose(side, pos, quat)
                except Exception:  # noqa: BLE001
                    pass
            self.spin_once(0.0)
            time.sleep(period)

    def call_enable(self, enable: bool, timeout: float = 10.0) -> None:
        if not self._cli_en.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "set_arm_enable unavailable — 机器人上需先启动 body_control 与 "
                "tianyi2_bringup（hardware:=real）。例如：\n"
                "  ros2 launch body_control body.launch.py\n"
                "  ros2 launch tianyi2_bringup tianyi2.launch.py hardware:=real"
            )
        req = self._SetBool.Request()
        req.data = bool(enable)
        fut = self._cli_en.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_arm_enable failed")
        self.node.get_logger().info(f"set_arm_enable({enable}): {fut.result().message}")

    def call_mode(self, mode: int = 3, timeout: float = 10.0) -> None:
        if not self._cli_mode.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(
                "set_arm_mode unavailable — 请确认 body_control / XARM 已启动且 DDS 正常"
            )
        req = self._Mode.Request()
        req.mode = int(mode)
        fut = self._cli_mode.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_arm_mode failed")
        self.node.get_logger().info(f"set_arm_mode({mode}): {fut.result().info}")

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

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
        except Exception:  # noqa: BLE001
            pass
        if self._rclpy.ok():
            self._rclpy.shutdown()
