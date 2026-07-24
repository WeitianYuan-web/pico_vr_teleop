#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RH56E2 遥操作一体化桥接：订阅 io_teleop 关节指令，使用 Setpos 映射下发位置。

映射逻辑:
  pos0 little_1+little_2  rad_sum [0, 3.0]  -> [0, 2000]
  pos1 ring_1+ring_2        rad_sum [0, 3.0]  -> [0, 2000]
  pos2 middle_1+middle_2    rad_sum [0, 3.0]  -> [0, 2000]
  pos3 index_1+index_2      rad_sum [0, 3.0]  -> [0, 2000]
  pos4 thumb_2+3+4          rad_sum [0, 1.2]  -> [0, 2000]
  pos5 thumb_1              rad_sum [0, 1.8]  -> [0, 2000]

硬件: RS485 写位置寄存器 0x05C2 (1474), 与 Hand_control Setpos 服务相同。

默认订阅:
  - /io_teleop/Inspire_RH56E2/joint_cmd_finger_right
  - /io_teleop/Inspire_RH56E2/joint_cmd_finger_left

启动示例:
  python3 src/inspire_hand/src/scripts/inspire_rh56e2_teleop_bridge.py --ros-args \\
    -p right_serial_port:=/dev/ttyUSB0 \\
    -p left_serial_port:=/dev/ttyUSB1 \\
    -p log_mapped_positions:=true
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

POS_NAMES: Tuple[str, ...] = (
    'pos0 (Pinky)',
    'pos1 (Ring)',
    'pos2 (Middle)',
    'pos3 (Index)',
    'pos4 (Thumb Flexion)',
    'pos5 (Thumb Abduction)',
)


@dataclass(frozen=True)
class FingerMapping:

    joint_suffixes: Tuple[str, ...]
    pos_min: int
    pos_max: int
    rad_sum_lower: float
    rad_sum_upper: float


FINGER_MAPPINGS: Tuple[FingerMapping, ...] = (
    FingerMapping(('little_1_joint', 'little_2_joint'), 0, 2000, 0.0, 3.0),
    FingerMapping(('ring_1_joint', 'ring_2_joint'), 0, 2000, 0.0, 3.0),
    FingerMapping(('middle_1_joint', 'middle_2_joint'), 0, 2000, 0.0, 3.0),
    FingerMapping(('index_1_joint', 'index_2_joint'), 0, 2000, 0.0, 3.0),
    FingerMapping(('thumb_2_joint', 'thumb_3_joint', 'thumb_4_joint'), 0, 2000, 0.0, 1.2),
    FingerMapping(('thumb_1_joint',), 0, 2000, 0.0, 1.8),
)

POS_SET_BASE_ADDRESS = 0x05C2  # 1474, 与 Hand_control.cpp Setpos 一致

IO_TELEOP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)


def map_joint_state_to_positions(
        msg: JointState,
        joint_prefix: str) -> Optional[List[int]]:

    if not msg.name or not msg.position:
        return None

    joint_dic = dict(zip(msg.name, msg.position))
    joint_position: List[int] = []

    for mapping in FINGER_MAPPINGS:
        joint_names = [f'{joint_prefix}{suffix}' for suffix in mapping.joint_suffixes]
        if any(name not in joint_dic for name in joint_names):
            return None

        rad_sum = sum(joint_dic[name] for name in joint_names)
        span = mapping.rad_sum_upper - mapping.rad_sum_lower
        if span <= 0.0:
            joint_dir = 0.0
        else:
            joint_dir = (rad_sum - mapping.rad_sum_lower) / span
            joint_dir = max(0.0, min(1.0, joint_dir))

        joint_val = mapping.pos_min + int(joint_dir * (mapping.pos_max - mapping.pos_min))
        joint_position.append(joint_val)

    return joint_position


class InspireHand485Writer:
    """Inspire Hand RS485 写位置寄存器 (Setpos 协议)。"""

    REQUEST_FRAME_HEADER = (0xEB, 0x90)

    def __init__(
            self,
            serial_port: str,
            baud_rate: int,
            hand_id: int,
            logger) -> None:
        self._hand_id = hand_id
        self._logger = logger
        self._ser: Optional[serial.Serial] = None
        self._open(serial_port, baud_rate)

    def _open(self, port: str, baud_rate: int) -> None:
        try:
            ser = serial.Serial(port, baud_rate, timeout=0.001)
            if not ser.is_open:
                ser.open()
            self._ser = ser
            self._logger.info(f'串口已打开: {port}, hand_id={self._hand_id}')
        except serial.SerialException as exc:
            self._logger.error(f'串口打开失败 {port}: {exc}')
            self._ser = None

    @property
    def available(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def write_positions(self, positions: List[int], log_serial: bool = False) -> bool:
        """写入 pos0~pos5 (各 0-2000) 到位置寄存器 0x05C2。"""
        if not self.available:
            self._logger.error('串口不可用，跳过写入')
            return False

        if len(positions) != 6:
            self._logger.error(f'位置数量错误: {len(positions)}, 期望 6')
            return False

        payload: List[int] = []
        for pos in positions:
            if not 0 <= pos <= 2000:
                self._logger.warn(f'位置超范围: {pos}，有效 0-2000')
                return False
            payload.extend([pos & 0xFF, (pos >> 8) & 0xFF])

        frame = [
            *self.REQUEST_FRAME_HEADER,
            self._hand_id,
            0x0F,
            0x12,
            POS_SET_BASE_ADDRESS & 0xFF,
            (POS_SET_BASE_ADDRESS >> 8) & 0xFF,
            *payload,
        ]
        frame.append(sum(frame[2:]) & 0xFF)

        if log_serial:
            self._logger.info(f'串口写入 Setpos: {[hex(b) for b in frame]}')

        self._ser.write(bytes(frame))
        time.sleep(0.020)
        if self._ser.in_waiting:
            self._ser.read(self._ser.in_waiting)
        return True


@dataclass
class HandChannel:
    label: str
    joint_prefix: str
    writer: InspireHand485Writer
    log_mapped: bool


class InspireRh56e2TeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__('inspire_rh56e2_teleop_bridge')

        self.declare_parameter(
            'right_input_topic',
            '/io_teleop/Inspire_RH56E2/joint_cmd_finger_right')
        self.declare_parameter(
            'left_input_topic',
            '/io_teleop/Inspire_RH56E2/joint_cmd_finger_left')
        self.declare_parameter('right_joint_prefix', 'right_')
        self.declare_parameter('left_joint_prefix', 'left_')
        self.declare_parameter('right_serial_port', '/dev/ttyUSB0')
        self.declare_parameter('left_serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('right_hand_id', 1)
        self.declare_parameter('left_hand_id', 1)
        self.declare_parameter('enable_right_hand', True)
        self.declare_parameter('enable_left_hand', True)
        self.declare_parameter('log_mapped_positions', False)
        self.declare_parameter('log_serial', False)

        baud_rate = int(self.get_parameter('baud_rate').value)
        log_mapped = bool(self.get_parameter('log_mapped_positions').value)
        log_serial = bool(self.get_parameter('log_serial').value)
        self._log_serial = log_serial

        self._right: Optional[HandChannel] = None
        self._left: Optional[HandChannel] = None

        if bool(self.get_parameter('enable_right_hand').value):
            port = self.get_parameter('right_serial_port').value
            hand_id = int(self.get_parameter('right_hand_id').value)
            topic = self.get_parameter('right_input_topic').value
            prefix = self.get_parameter('right_joint_prefix').value
            writer = InspireHand485Writer(port, baud_rate, hand_id, self.get_logger())
            self._right = HandChannel('右手', prefix, writer, log_mapped)
            self.create_subscription(
                JointState, topic, self._right_callback, IO_TELEOP_QOS)
            self.get_logger().info(
                f'[右手] 订阅 {topic} -> Setpos {port} (hand_id={hand_id})')
            self._log_mapping('右手', prefix)

        if bool(self.get_parameter('enable_left_hand').value):
            port = self.get_parameter('left_serial_port').value
            hand_id = int(self.get_parameter('left_hand_id').value)
            topic = self.get_parameter('left_input_topic').value
            prefix = self.get_parameter('left_joint_prefix').value
            writer = InspireHand485Writer(port, baud_rate, hand_id, self.get_logger())
            self._left = HandChannel('左手', prefix, writer, log_mapped)
            self.create_subscription(
                JointState, topic, self._left_callback, IO_TELEOP_QOS)
            self.get_logger().info(
                f'[左手] 订阅 {topic} -> Setpos {port} (hand_id={hand_id})')
            self._log_mapping('左手', prefix)

        if self._right is None and self._left is None:
            raise RuntimeError('至少需要启用一只手 (enable_right_hand / enable_left_hand)')

    def _log_mapping(self, label: str, joint_prefix: str) -> None:
        for idx, mapping in enumerate(FINGER_MAPPINGS):
            joints = '+'.join(f'{joint_prefix}{s}' for s in mapping.joint_suffixes)
            self.get_logger().info(
                f'[{label}] {POS_NAMES[idx]} {joints}: '
                f'rad_sum [{mapping.rad_sum_lower}, {mapping.rad_sum_upper}] '
                f'-> pos [{mapping.pos_min}, {mapping.pos_max}]')

    def _handle_joint_state(self, msg: JointState, channel: HandChannel) -> None:
        positions = map_joint_state_to_positions(msg, channel.joint_prefix)
        if positions is None:
            self.get_logger().warn(f'[{channel.label}] JointState 无效或缺少关节')
            return

        ok = channel.writer.write_positions(positions, log_serial=self._log_serial)
        if not ok:
            return

        if channel.log_mapped:
            detail = ' | '.join(
                f'{POS_NAMES[i]}={positions[i]}' for i in range(6))
            self.get_logger().info(f'[{channel.label}] 已下发 Setpos: {detail}')

    def _right_callback(self, msg: JointState) -> None:
        if self._right is not None:
            self._handle_joint_state(msg, self._right)

    def _left_callback(self, msg: JointState) -> None:
        if self._left is not None:
            self._handle_joint_state(msg, self._left)

    def destroy_node(self) -> bool:
        if self._right is not None:
            self._right.writer.close()
        if self._left is not None:
            self._left.writer.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = InspireRh56e2TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
