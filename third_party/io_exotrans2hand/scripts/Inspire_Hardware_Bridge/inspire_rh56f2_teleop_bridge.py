#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RH56F2 遥操作一体化桥接：订阅 io_teleop 关节指令，使用 angleSet 映射下发电缸角度。

映射逻辑 (与 inspire_hand_left_controller.py 一致):
  电缸1 pinky_1_joint       rad [0, 1.47]  -> [1740, 900]
  电缸2 ring_1_joint        rad [0, 1.47]  -> [1740, 900]
  电缸3 middle_1_joint      rad [0, 1.47]  -> [1740, 900]
  电缸4 index_1_joint       rad [0, 1.47]  -> [1740, 900]
  电缸5 thumb_2_joint       rad [0, 0.79]  -> [1450, 1100]
  电缸6 thumb_1_joint       rad [0, 2.0]   -> [1750, 500]

硬件: RS485 写 angleSet 寄存器 1040。

默认订阅:
  - /io_teleop/Inspire_RH56F2/joint_cmd_finger_right
  - /io_teleop/Inspire_RH56F2/joint_cmd_finger_left

启动示例:
  python3 inspire_rh56f2_teleop_bridge.py --ros-args \\
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

CYLINDER_NAMES: Tuple[str, ...] = (
    '电缸1 (Pinky)',
    '电缸2 (Ring)',
    '电缸3 (Middle)',
    '电缸4 (Index)',
    '电缸5 (Thumb Flexion)',
    '电缸6 (Thumb Abduction)',
)

_REG_ANGLE_SET = 1040
_REG_MODE = 1100
_REG_FORCE_SET = 1046
_REG_SPEED_SET = 1052


@dataclass(frozen=True)
class FingerMapping:
    joint_suffixes: Tuple[str, ...]
    actuator_at_rad_lower: int
    actuator_at_rad_upper: int
    rad_sum_lower: float
    rad_sum_upper: float


FINGER_MAPPINGS: Tuple[FingerMapping, ...] = (
    FingerMapping(('pinky_1_joint',), 1740, 900, 0.0, 1.47),
    FingerMapping(('ring_1_joint',), 1740, 900, 0.0, 1.47),
    FingerMapping(('middle_1_joint',), 1740, 900, 0.0, 1.47),
    FingerMapping(('index_1_joint',), 1740, 900, 0.0, 1.47),
    FingerMapping(('thumb_2_joint',), 1450, 1100, 0.0, 0.79),
    FingerMapping(('thumb_1_joint',), 1750, 500, 0.0, 2.0),
)

IO_TELEOP_QOS = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)


def map_joint_state_to_angles(
        msg: JointState,
        joint_prefix: str) -> Optional[List[int]]:

    if not msg.name or not msg.position:
        return None

    joint_dic = dict(zip(msg.name, msg.position))
    angle_vals: List[int] = []

    for mapping in FINGER_MAPPINGS:
        joint_names = [f'{joint_prefix}{suffix}' for suffix in mapping.joint_suffixes]
        if any(name not in joint_dic for name in joint_names):
            return None

        rad_sum = sum(joint_dic[name] for name in joint_names)
        span = mapping.rad_sum_upper - mapping.rad_sum_lower
        if span <= 0.0:
            ratio = 0.0
        else:
            ratio = (rad_sum - mapping.rad_sum_lower) / span
            ratio = max(0.0, min(1.0, ratio))

        angle_val = int(
            mapping.actuator_at_rad_lower
            + ratio * (mapping.actuator_at_rad_upper - mapping.actuator_at_rad_lower))
        angle_vals.append(angle_val)

    return angle_vals


class InspireHand485Writer:
    """Inspire RH56F2 RS485 写 angleSet 寄存器。"""

    REQUEST_FRAME_HEADER = (0xEB, 0x90)

    def __init__(
            self,
            serial_port: str,
            baud_rate: int,
            hand_id: int,
            logger,
            init_hand: bool = True) -> None:
        self._hand_id = hand_id
        self._logger = logger
        self._ser: Optional[serial.Serial] = None
        self._open(serial_port, baud_rate)
        if init_hand and self.available:
            self._init_hand()

    def _open(self, port: str, baud_rate: int) -> None:
        try:
            ser = serial.Serial(port, baud_rate, timeout=0.1)
            if not ser.is_open:
                ser.open()
            self._ser = ser
            self._logger.info(f'串口已打开: {port}, hand_id={self._hand_id}')
        except serial.SerialException as exc:
            self._logger.error(f'串口打开失败 {port}: {exc}')
            self._ser = None

    def _write_register(self, address: int, payload: List[int]) -> None:
        if not self.available:
            return
        frame = [
            *self.REQUEST_FRAME_HEADER,
            self._hand_id,
            len(payload) + 3,
            0x12,
            address & 0xFF,
            (address >> 8) & 0xFF,
            *payload,
        ]
        frame.append(sum(frame[2:]) & 0xFF)
        self._ser.write(bytes(frame))
        time.sleep(0.01)
        if self._ser.in_waiting:
            self._ser.read(self._ser.in_waiting)

    def _write6(self, address: int, values: List[int]) -> None:
        payload: List[int] = []
        for value in values:
            payload.extend([value & 0xFF, (value >> 8) & 0xFF])
        self._write_register(address, payload)

    def _init_hand(self) -> None:
        self._write6(_REG_MODE, [0, 0, 0, 0, 0, 0])
        time.sleep(0.1)
        self._write6(_REG_SPEED_SET, [4000, 4000, 4000, 4000, 4000, 4000])
        time.sleep(0.1)
        self._write6(_REG_FORCE_SET, [6000, 6000, 6000, 6000, 6000, 6000])
        time.sleep(0.1)
        self._logger.info('灵巧手初始化完成（mode=0, speed=4000, force=6000）')

    @property
    def available(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def close(self) -> None:
        if self._ser is not None and self._ser.is_open:
            self._ser.close()

    def write_angles(self, angles: List[int], log_serial: bool = False) -> bool:
        """写入 6 路电缸角度到 angleSet 寄存器 1040。"""
        if not self.available:
            self._logger.error('串口不可用，跳过写入')
            return False

        if len(angles) != 6:
            self._logger.error(f'角度数量错误: {len(angles)}, 期望 6')
            return False

        if log_serial:
            self._logger.info(f'串口写入 angleSet: {angles}')

        self._write6(_REG_ANGLE_SET, angles)
        return True


@dataclass
class HandChannel:
    label: str
    joint_prefix: str
    writer: InspireHand485Writer
    log_mapped: bool


class InspireRh56f2TeleopBridge(Node):
    def __init__(self) -> None:
        super().__init__('inspire_rh56f2_teleop_bridge')

        self.declare_parameter(
            'right_input_topic',
            '/io_teleop/Inspire_RH56F2/joint_cmd_finger_right')
        self.declare_parameter(
            'left_input_topic',
            '/io_teleop/Inspire_RH56F2/joint_cmd_finger_left')
        self.declare_parameter('right_joint_prefix', 'right_')
        self.declare_parameter('left_joint_prefix', 'left_')
        self.declare_parameter('right_serial_port', '/dev/ttyUSB0')
        self.declare_parameter('left_serial_port', '/dev/ttyUSB1')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('right_hand_id', 1)
        self.declare_parameter('left_hand_id', 1)
        self.declare_parameter('enable_right_hand', True)
        self.declare_parameter('enable_left_hand', True)
        self.declare_parameter('init_hand_on_start', True)
        self.declare_parameter('log_mapped_positions', False)
        self.declare_parameter('log_serial', False)

        baud_rate = int(self.get_parameter('baud_rate').value)
        init_hand = bool(self.get_parameter('init_hand_on_start').value)
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
            writer = InspireHand485Writer(
                port, baud_rate, hand_id, self.get_logger(), init_hand=init_hand)
            self._right = HandChannel('右手', prefix, writer, log_mapped)
            self.create_subscription(
                JointState, topic, self._right_callback, IO_TELEOP_QOS)
            self.get_logger().info(
                f'[右手] 订阅 {topic} -> angleSet {port} (hand_id={hand_id})')
            self._log_mapping('右手', prefix)

        if bool(self.get_parameter('enable_left_hand').value):
            port = self.get_parameter('left_serial_port').value
            hand_id = int(self.get_parameter('left_hand_id').value)
            topic = self.get_parameter('left_input_topic').value
            prefix = self.get_parameter('left_joint_prefix').value
            writer = InspireHand485Writer(
                port, baud_rate, hand_id, self.get_logger(), init_hand=init_hand)
            self._left = HandChannel('左手', prefix, writer, log_mapped)
            self.create_subscription(
                JointState, topic, self._left_callback, IO_TELEOP_QOS)
            self.get_logger().info(
                f'[左手] 订阅 {topic} -> angleSet {port} (hand_id={hand_id})')
            self._log_mapping('左手', prefix)

        if self._right is None and self._left is None:
            raise RuntimeError('至少需要启用一只手 (enable_right_hand / enable_left_hand)')

    def _log_mapping(self, label: str, joint_prefix: str) -> None:
        for idx, mapping in enumerate(FINGER_MAPPINGS):
            joints = '+'.join(f'{joint_prefix}{s}' for s in mapping.joint_suffixes)
            self.get_logger().info(
                f'[{label}] {CYLINDER_NAMES[idx]} {joints}: '
                f'rad [{mapping.rad_sum_lower}, {mapping.rad_sum_upper}] '
                f'-> angle [{mapping.actuator_at_rad_lower}, {mapping.actuator_at_rad_upper}]')

    def _handle_joint_state(self, msg: JointState, channel: HandChannel) -> None:
        angles = map_joint_state_to_angles(msg, channel.joint_prefix)
        if angles is None:
            self.get_logger().warn(f'[{channel.label}] JointState 无效或缺少关节')
            return

        ok = channel.writer.write_angles(angles, log_serial=self._log_serial)
        if not ok:
            return

        if channel.log_mapped:
            detail = ' | '.join(
                f'{CYLINDER_NAMES[i]}={angles[i]}' for i in range(6))
            self.get_logger().info(f'[{channel.label}] 已下发 angleSet: {detail}')

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
    node = InspireRh56f2TeleopBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
