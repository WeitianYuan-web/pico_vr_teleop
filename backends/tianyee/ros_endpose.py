"""ROS2 helpers: TF TCP lookup + /endposetarget_* publish + arm prepare."""

from __future__ import annotations

import time
from typing import Literal

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
        from std_msgs.msg import Header
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
        self._Pose = Pose
        self._Header = Header
        self._ArmTargetPose = ArmTargetPose
        self._cli_en = self.node.create_client(SetBool, "/EAIHardware/set_arm_enable")
        self._cli_mode = self.node.create_client(Mode, "/EAIHardware/set_arm_mode")
        self._SetBool = SetBool
        self._Mode = Mode

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
        # TF quat is xyzw → store wxyz
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

    def call_enable(self, enable: bool, timeout: float = 10.0) -> None:
        if not self._cli_en.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("set_arm_enable unavailable")
        req = self._SetBool.Request()
        req.data = bool(enable)
        fut = self._cli_en.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_arm_enable failed")
        self.node.get_logger().info(f"set_arm_enable({enable}): {fut.result().message}")

    def call_mode(self, mode: int = 3, timeout: float = 10.0) -> None:
        if not self._cli_mode.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("set_arm_mode unavailable")
        req = self._Mode.Request()
        req.mode = int(mode)
        fut = self._cli_mode.call_async(req)
        self._rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_arm_mode failed")
        self.node.get_logger().info(f"set_arm_mode({mode}): {fut.result().info}")

    def enable_auto_switch(self) -> None:
        import subprocess

        subprocess.run(
            ["ros2", "control", "auto_switch_mode", "--enable"],
            check=False,
            capture_output=True,
            text=True,
        )

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
