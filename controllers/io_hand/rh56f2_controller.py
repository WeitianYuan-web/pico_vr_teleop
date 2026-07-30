#!/usr/bin/env python3
"""统一 RH56F2 手控：只执行，不产生遥操作指令。

指令源（任一，同侧最新优先）:
  /hand_cmd/left|right
  /io_teleop/Inspire_RH56F2/joint_cmd_finger_left|right  （兼容 zenoh2ros）

状态输出:
  /puppet/hand_left|right  （finger_1..6，rad；仅本节点占串口时有效）
"""

from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Any, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

CONTROLLER_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.abspath(os.path.join(CONTROLLER_DIR, "../.."))
PUBLISHER_DIR = os.path.join(PROJECT_ROOT, "publisher")
if CONTROLLER_DIR not in sys.path:
    sys.path.insert(0, CONTROLLER_DIR)
if PUBLISHER_DIR not in sys.path:
    sys.path.insert(0, PUBLISHER_DIR)

from inspire_sdk_driver import (  # noqa: E402
    DEFAULT_FORCE,
    DEFAULT_MODEL,
    DEFAULT_SPEED,
    InspireSdkHand,
)
from mapping import CYLINDER_NAMES, FINGER_MAPPINGS, map_named_positions_to_angles  # noqa: E402
from teleop_state_bridge import hand_registers_to_radians  # noqa: E402

CMD_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
STATE_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

PUPPET_HAND_JOINT_NAMES = [f"finger_{i}" for i in range(1, 7)]


@dataclass
class HandChannel:
    label: str
    side: str
    joint_prefix: str
    driver: InspireSdkHand
    log_mapped: bool
    state_pub: Any


class Rh56f2Controller(Node):
    def __init__(self) -> None:
        super().__init__("rh56f2_controller")

        self.declare_parameter("right_hand_cmd_topic", "/hand_cmd/right")
        self.declare_parameter("left_hand_cmd_topic", "/hand_cmd/left")
        self.declare_parameter(
            "right_io_topic",
            "/io_teleop/Inspire_RH56F2/joint_cmd_finger_right",
        )
        self.declare_parameter(
            "left_io_topic",
            "/io_teleop/Inspire_RH56F2/joint_cmd_finger_left",
        )
        self.declare_parameter("right_state_topic", "/puppet/hand_right")
        self.declare_parameter("left_state_topic", "/puppet/hand_left")
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
        self.declare_parameter("publish_hand_state", True)
        self.declare_parameter("state_publish_hz", 50.0)

        # 兼容旧参数名
        self.declare_parameter(
            "right_input_topic",
            "/io_teleop/Inspire_RH56F2/joint_cmd_finger_right",
        )
        self.declare_parameter(
            "left_input_topic",
            "/io_teleop/Inspire_RH56F2/joint_cmd_finger_left",
        )

        model = str(self.get_parameter("hand_model").value)
        force = int(self.get_parameter("hand_force").value)
        speed = int(self.get_parameter("hand_speed").value)
        baud = int(self.get_parameter("baud_rate").value)
        control_hz = int(self.get_parameter("hand_control_hz").value)
        io_hz = int(self.get_parameter("hand_io_hz").value)
        log_mapped = bool(self.get_parameter("log_mapped_positions").value)
        self._publish_state = bool(self.get_parameter("publish_hand_state").value)
        state_hz = float(self.get_parameter("state_publish_hz").value)

        self._right: Optional[HandChannel] = None
        self._left: Optional[HandChannel] = None

        if bool(self.get_parameter("enable_right_hand").value):
            self._right = self._setup_side(
                side="right",
                label="右手",
                port=str(self.get_parameter("right_serial_port").value),
                hand_id=int(self.get_parameter("right_hand_id").value),
                prefix=str(self.get_parameter("right_joint_prefix").value),
                cmd_topic=str(self.get_parameter("right_hand_cmd_topic").value),
                io_topic=str(self.get_parameter("right_io_topic").value),
                legacy_topic=str(self.get_parameter("right_input_topic").value),
                state_topic=str(self.get_parameter("right_state_topic").value),
                model=model,
                baud=baud,
                control_hz=control_hz,
                io_hz=io_hz,
                force=force,
                speed=speed,
                log_mapped=log_mapped,
            )

        if bool(self.get_parameter("enable_left_hand").value):
            self._left = self._setup_side(
                side="left",
                label="左手",
                port=str(self.get_parameter("left_serial_port").value),
                hand_id=int(self.get_parameter("left_hand_id").value),
                prefix=str(self.get_parameter("left_joint_prefix").value),
                cmd_topic=str(self.get_parameter("left_hand_cmd_topic").value),
                io_topic=str(self.get_parameter("left_io_topic").value),
                legacy_topic=str(self.get_parameter("left_input_topic").value),
                state_topic=str(self.get_parameter("left_state_topic").value),
                model=model,
                baud=baud,
                control_hz=control_hz,
                io_hz=io_hz,
                force=force,
                speed=speed,
                log_mapped=log_mapped,
            )

        if self._right is None and self._left is None:
            raise RuntimeError("至少需要启用一只手 (enable_right_hand / enable_left_hand)")

        if self._publish_state and state_hz > 0.0:
            self.create_timer(1.0 / state_hz, self._on_state_timer)

    def _setup_side(
        self,
        *,
        side: str,
        label: str,
        port: str,
        hand_id: int,
        prefix: str,
        cmd_topic: str,
        io_topic: str,
        legacy_topic: str,
        state_topic: str,
        model: str,
        baud: int,
        control_hz: int,
        io_hz: int,
        force: int,
        speed: int,
        log_mapped: bool,
    ) -> HandChannel:
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
            raise RuntimeError(f"{label}连接失败: port={port}, id={hand_id}, model={model}")

        state_pub = self.create_publisher(JointState, state_topic, STATE_QOS)
        channel = HandChannel(label, side, prefix, driver, log_mapped, state_pub)

        topics = []
        for t in (cmd_topic, io_topic, legacy_topic):
            if t and t not in topics:
                topics.append(t)
        for topic in topics:
            self.create_subscription(
                JointState,
                topic,
                (self._right_callback if side == "right" else self._left_callback),
                CMD_QOS,
            )
            self.get_logger().info(
                f"[{label}] 订阅 {topic} -> inspire_hand_py({model}) {port} (id={hand_id})"
            )
        if self._publish_state:
            self.get_logger().info(f"[{label}] 状态发布 {state_topic}")
        self._log_mapping(label, prefix)
        return channel

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

    def _publish_side_state(self, channel: Optional[HandChannel]) -> None:
        if channel is None or not self._publish_state:
            return
        angles = channel.driver.get_angles()
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f"hand_{channel.side}"
        msg.name = list(PUPPET_HAND_JOINT_NAMES)
        if angles and len(angles) >= 6:
            msg.position = hand_registers_to_radians(list(angles[:6]))
        else:
            msg.position = []
        channel.state_pub.publish(msg)

    def _on_state_timer(self) -> None:
        self._publish_side_state(self._right)
        self._publish_side_state(self._left)

    def destroy_node(self) -> bool:
        if self._right is not None:
            self._right.driver.close()
        if self._left is not None:
            self._left.driver.close()
        return super().destroy_node()


# 旧类名兼容
IoRh56f2Bridge = Rh56f2Controller


def main() -> None:
    rclpy.init()
    node = Rh56f2Controller()
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
