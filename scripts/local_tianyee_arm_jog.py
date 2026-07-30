#!/usr/bin/env python3
"""Local joint-space jog for Tianyi left arm via /jointspace_commands_L."""
from __future__ import annotations

import argparse
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool

try:
    from eai_manipulator_msgs.srv import Mode
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "eai_manipulator_msgs missing; source third_party/tianyee_ros_ws/install/setup.bash"
    ) from exc

LEFT_NAMES = [
    "shoulder_pitch_l_joint",
    "shoulder_roll_l_joint",
    "shoulder_yaw_l_joint",
    "elbow_pitch_l_joint",
    "elbow_yaw_l_joint",
    "wrist_pitch_l_joint",
    "wrist_roll_l_joint",
]


class LocalArmJog(Node):
    def __init__(self) -> None:
        super().__init__("local_tianyee_arm_jog")
        self._got: dict[str, float] = {}
        self.create_subscription(JointState, "/joint_states", self._on_js, 50)
        self._pub = self.create_publisher(Float64MultiArray, "/jointspace_commands_L", 10)
        self._cli_en = self.create_client(SetBool, "/EAIHardware/set_arm_enable")
        self._cli_mode = self.create_client(Mode, "/EAIHardware/set_arm_mode")

    def _on_js(self, msg: JointState) -> None:
        for name, pos in zip(msg.name, msg.position):
            if name in LEFT_NAMES:
                self._got[name] = float(pos)

    def wait_joints(self, timeout: float = 8.0) -> list[float]:
        deadline = time.time() + timeout
        while time.time() < deadline and len(self._got) < len(LEFT_NAMES):
            rclpy.spin_once(self, timeout_sec=0.1)
        missing = [n for n in LEFT_NAMES if n not in self._got]
        if missing:
            raise RuntimeError(f"missing joints: {missing}")
        return [self._got[n] for n in LEFT_NAMES]

    def call_enable(self, enable: bool, timeout: float = 10.0) -> None:
        if not self._cli_en.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("set_arm_enable service unavailable")
        req = SetBool.Request()
        req.data = enable
        fut = self._cli_en.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_arm_enable call failed")
        self.get_logger().info(f"set_arm_enable({enable}): {fut.result().message}")

    def call_mode(self, mode: int, timeout: float = 10.0) -> None:
        if not self._cli_mode.wait_for_service(timeout_sec=timeout):
            raise RuntimeError("set_arm_mode service unavailable")
        req = Mode.Request()
        req.mode = int(mode)
        fut = self._cli_mode.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("set_arm_mode call failed")
        self.get_logger().info(f"set_arm_mode({mode}): {fut.result().info}")

    def stream(self, q: list[float], seconds: float, hz: float = 20.0) -> None:
        period = 1.0 / hz
        msg = Float64MultiArray()
        msg.data = [float(x) for x in q]
        # wait for subscription match
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.05)
            if self._pub.get_subscription_count() > 0:
                break
        self.get_logger().info(f"subscribers={self._pub.get_subscription_count()}")
        end = time.time() + seconds
        while time.time() < end:
            self._pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--delta-deg", type=float, default=3.0, help="elbow pitch delta degrees")
    parser.add_argument("--hold", type=float, default=2.5, help="seconds to stream each target")
    parser.add_argument("--no-enable", action="store_true", help="skip enable/mode services")
    parser.add_argument("--leave-enabled", action="store_true", help="do not disable arm at end")
    args = parser.parse_args()

    rclpy.init()
    node = LocalArmJog()
    try:
        if not args.no_enable:
            node.call_enable(True)
            node.call_mode(3)

        q0 = node.wait_joints()
        node.get_logger().info(f"q0={[round(x, 4) for x in q0]}")
        q1 = list(q0)
        q1[3] = q0[3] + math.radians(args.delta_deg)
        node.get_logger().info(
            f"jog elbow {math.degrees(q0[3]):.2f} -> {math.degrees(q1[3]):.2f} deg"
        )

        node.stream(q1, args.hold)
        time.sleep(0.3)
        node._got.clear()
        q_mid = node.wait_joints(timeout=3.0)
        dz = math.degrees(q_mid[3] - q0[3])
        node.get_logger().info(f"MID elbow delta_deg={dz:.3f}")

        node.get_logger().info("return")
        node.stream(q0, args.hold)
        time.sleep(0.3)
        node._got.clear()
        q_back = node.wait_joints(timeout=3.0)
        dzb = math.degrees(q_back[3] - q0[3])
        node.get_logger().info(f"BACK elbow delta_deg={dzb:.3f}")
        print(f"RESULT mid_delta_deg={dz:.3f} back_delta_deg={dzb:.3f}")
    finally:
        if not args.no_enable and not args.leave_enabled:
            try:
                node.call_enable(False)
            except Exception as exc:  # noqa: BLE001
                node.get_logger().warn(f"disable failed: {exc}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
