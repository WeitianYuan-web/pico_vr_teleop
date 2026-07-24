#!/usr/bin/env python3
"""订阅 IO 重定向 JointState，经 InspireHandSDK_Y（默认 RH56F2）下发。

默认话题（与原 inspire_rh56f2_teleop_bridge / zenoh2ros 一致）:
  /io_teleop/Inspire_RH56F2/joint_cmd_finger_left
  /io_teleop/Inspire_RH56F2/joint_cmd_finger_right
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

CONTROLLER_DIR = os.path.dirname(os.path.abspath(__file__))
if CONTROLLER_DIR not in sys.path:
    sys.path.insert(0, CONTROLLER_DIR)

from inspire_sdk_driver import (  # noqa: E402
    DEFAULT_FORCE,
    DEFAULT_MODEL,
    DEFAULT_SPEED,
    InspireSdkHand,
)
from mapping import CYLINDER_NAMES, FINGER_MAPPINGS, map_named_positions_to_angles  # noqa: E402

IO_TELEOP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)


@dataclass
class HandChannel:
    label: str
    joint_prefix: str
    driver: InspireSdkHand
    log_mapped: bool


class IoRh56f2Bridge(Node):
    def __init__(self) -> None:
        super().__init__("io_rh56f2_bridge")

        self.declare_parameter(
            "right_input_topic",
            "/io_teleop/Inspire_RH56F2/joint_cmd_finger_right",
        )
        self.declare_parameter(
            "left_input_topic",
            "/io_teleop/Inspire_RH56F2/joint_cmd_finger_left",
        )
        self.declare_parameter("right_joint_prefix", "right_")
        self.declare_parameter("left_joint_prefix", "left_")
        self.declare_parameter("right_serial_port", "/dev/ttyUSB0")
        self.declare_parameter("left_serial_port", "/dev/ttyUSB1")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("right_hand_id", 1)
        self.declare_parameter("left_hand_id", 1)
        self.declare_parameter("enable_right_hand", True)
        self.declare_parameter("enable_left_hand", True)
        self.declare_parameter("hand_model", DEFAULT_MODEL)
        self.declare_parameter("hand_force", DEFAULT_FORCE)
        self.declare_parameter("hand_speed", DEFAULT_SPEED)
        self.declare_parameter("hand_control_hz", 100)
        self.declare_parameter("hand_io_hz", 30)
        self.declare_parameter("log_mapped_positions", False)

        model = str(self.get_parameter("hand_model").value)
        force = int(self.get_parameter("hand_force").value)
        speed = int(self.get_parameter("hand_speed").value)
        baud = int(self.get_parameter("baud_rate").value)
        control_hz = int(self.get_parameter("hand_control_hz").value)
        io_hz = int(self.get_parameter("hand_io_hz").value)
        log_mapped = bool(self.get_parameter("log_mapped_positions").value)

        self._right: Optional[HandChannel] = None
        self._left: Optional[HandChannel] = None

        if bool(self.get_parameter("enable_right_hand").value):
            port = str(self.get_parameter("right_serial_port").value)
            hand_id = int(self.get_parameter("right_hand_id").value)
            topic = str(self.get_parameter("right_input_topic").value)
            prefix = str(self.get_parameter("right_joint_prefix").value)
            driver = InspireSdkHand(
                port,
                hand_id=hand_id,
                model=model,
                baudrate=baud,
                control_hz=control_hz,
                io_hz=io_hz,
                force=force,
                speed=speed,
            )
            if not driver.connect():
                raise RuntimeError(f"右手连接失败: port={port}, id={hand_id}, model={model}")
            self._right = HandChannel("右手", prefix, driver, log_mapped)
            self.create_subscription(JointState, topic, self._right_callback, IO_TELEOP_QOS)
            self.get_logger().info(
                f"[右手] 订阅 {topic} -> inspire_hand_py({model}) {port} (id={hand_id})"
            )
            self._log_mapping("右手", prefix)

        if bool(self.get_parameter("enable_left_hand").value):
            port = str(self.get_parameter("left_serial_port").value)
            hand_id = int(self.get_parameter("left_hand_id").value)
            topic = str(self.get_parameter("left_input_topic").value)
            prefix = str(self.get_parameter("left_joint_prefix").value)
            driver = InspireSdkHand(
                port,
                hand_id=hand_id,
                model=model,
                baudrate=baud,
                control_hz=control_hz,
                io_hz=io_hz,
                force=force,
                speed=speed,
            )
            if not driver.connect():
                raise RuntimeError(f"左手连接失败: port={port}, id={hand_id}, model={model}")
            self._left = HandChannel("左手", prefix, driver, log_mapped)
            self.create_subscription(JointState, topic, self._left_callback, IO_TELEOP_QOS)
            self.get_logger().info(
                f"[左手] 订阅 {topic} -> inspire_hand_py({model}) {port} (id={hand_id})"
            )
            self._log_mapping("左手", prefix)

        if self._right is None and self._left is None:
            raise RuntimeError("至少需要启用一只手 (enable_right_hand / enable_left_hand)")

    def _log_mapping(self, label: str, joint_prefix: str) -> None:
        for idx, mapping in enumerate(FINGER_MAPPINGS):
            joints = "+".join(f"{joint_prefix}{s}" for s in mapping.joint_suffixes)
            self.get_logger().info(
                f"[{label}] {CYLINDER_NAMES[idx]} {joints}: "
                f"rad [{mapping.rad_sum_lower}, {mapping.rad_sum_upper}] "
                f"-> angle [{mapping.actuator_at_rad_lower}, {mapping.actuator_at_rad_upper}]"
            )

    def _handle_joint_state(self, msg: JointState, channel: HandChannel) -> None:
        angles = map_named_positions_to_angles(msg.name, msg.position, channel.joint_prefix)
        if angles is None:
            self.get_logger().warning(f"[{channel.label}] JointState 无效或缺少关节")
            return

        ok = channel.driver.submit_angles(angles)
        if not ok:
            self.get_logger().error(f"[{channel.label}] submit_angles 失败")
            return

        if channel.log_mapped:
            detail = " | ".join(f"{CYLINDER_NAMES[i]}={angles[i]}" for i in range(6))
            self.get_logger().info(f"[{channel.label}] 已下发: {detail}")

    def _right_callback(self, msg: JointState) -> None:
        if self._right is not None:
            self._handle_joint_state(msg, self._right)

    def _left_callback(self, msg: JointState) -> None:
        if self._left is not None:
            self._handle_joint_state(msg, self._left)

    def destroy_node(self) -> bool:
        if self._right is not None:
            self._right.driver.close()
        if self._left is not None:
            self._left.driver.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = IoRh56f2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
