#!/usr/bin/env python3

import sys
import time
import serial
import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class DexterousHandController:
    """灵巧手串口通信控制器"""
    
    REG_ANGLE_SET = 1080
    REG_ANGLE_ACT = 1136
    REG_SPEED_SET = 0x0454  # 1108，各自由度速度设置寄存器（断电不保存）
    REG_ACTION_SEQ = 0x0597
    REG_ACTION_RUN = 0x0598
    
    FRAME_HEADER = [0xEB, 0x90]
    FRAME_HEADER_RESP = [0x90, 0xEB]
    CMD_WRITE = 0x12
    CMD_READ = 0x11
    
    def __init__(
        self,
        port: str,
        baudrate: int,
        hand_id: int,
        logger,
        post_write_sleep: float = 0.025,
        angle_write_settle_s: float = 0.006,
    ):
        self.hand_id = hand_id
        self.logger = logger
        self.post_write_sleep = post_write_sleep
        self.angle_write_settle_s = angle_write_settle_s
        
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            self.logger.info(f"Serial port opened: {port} @ {baudrate}")
        except Exception as e:
            self.logger.error(f"Failed to open serial port {port}: {e}")
            raise
    
    @staticmethod
    def _checksum(data: List[int]) -> int:
        return sum(data) & 0xFF
    
    @staticmethod
    def _int16_to_angle(value: int) -> float:
        if value > 32767:
            value -= 65536
        return math.radians(value / 10.0)
    
    @staticmethod
    def _pack_int16_list(values: List[int]) -> List[int]:
        byte_list = []
        for val in values:
            byte_list.append(val & 0xFF)
            byte_list.append((val >> 8) & 0xFF)
        return byte_list
    
    def _make_write_frame(self, addr: int, data_bytes: List[int]) -> bytes:
        addr_l = addr & 0xFF
        addr_h = (addr >> 8) & 0xFF
        length = len(data_bytes) + 3
        body = [self.hand_id, length, self.CMD_WRITE, addr_l, addr_h] + data_bytes
        frame = self.FRAME_HEADER + body + [self._checksum(body)]
        return bytes(frame)
    
    def _make_read_frame(self, addr: int, num_regs: int) -> bytes:
        addr_l = addr & 0xFF
        addr_h = (addr >> 8) & 0xFF
        body = [self.hand_id, 0x04, self.CMD_READ, addr_l, addr_h, num_regs]
        frame = self.FRAME_HEADER + body + [self._checksum(body)]
        return bytes(frame)
    
    def write_register(
        self,
        addr: int,
        data_bytes: List[int],
        post_sleep: Optional[float] = None,
    ) -> bool:
        delay = self.post_write_sleep if post_sleep is None else post_sleep
        try:
            frame = self._make_write_frame(addr, data_bytes)
            self.ser.write(frame)
            time.sleep(delay)
            self.ser.read_all()
            return True
        except Exception as e:
            self.logger.error(f"Write register failed: {e}")
            return False
    
    def read_register(self, addr: int, num_regs: int, timeout: float = 0.5) -> Optional[List[int]]:
        try:
            self.ser.read_all()
            time.sleep(0.01)
            frame = self._make_read_frame(addr, num_regs)
            self.ser.write(frame)
            time.sleep(self.post_write_sleep)

            expected_length = 7 + num_regs
            recv = bytearray()
            start_time = time.time()

            while len(recv) < expected_length + 16:
                if time.time() - start_time > timeout:
                    break
                chunk = self.ser.read_all()
                if chunk:
                    recv.extend(chunk)
                time.sleep(0.01)

            header = bytes(self.FRAME_HEADER_RESP)
            offset = recv.find(header)
            if offset == -1:
                self.logger.warn(
                    f"Frame header not found in {len(recv)} bytes, "
                    f"raw hex: {recv.hex(' ')}"
                )
                return None

            recv = recv[offset:]
            if len(recv) < expected_length:
                self.logger.warn(f"Incomplete frame: need {expected_length}, got {len(recv)}")
                return None

            data = list(recv[7:7 + num_regs])
            return data
                
        except Exception as e:
            self.logger.error(f"Read register failed: {e}")
            return None
    
    JOINT_NAMES = [
        "pinky_mcp", "ring_mcp", "middle_mcp", "middle_yaw",
        "index_mcp", "index_yaw", "pinky_pip", "ring_pip",
        "middle_pip", "index_pip", "thumb_yaw", "thumb_mcp", "thumb_dip",
    ]

    def set_joint_angle_ticks(self, ticks: List[int]) -> bool:
        """下发各关节寄存器整型值"""
        if len(ticks) != 13:
            self.logger.error(f"Expected 13 angle ticks, got {len(ticks)}")
            return False

        debug_lines = [f"  {self.JOINT_NAMES[i]:12s} = {ticks[i]}" for i in range(13)]
        self.logger.info("set_joint_angle_ticks:\n" + "\n".join(debug_lines))

        data_bytes = self._pack_int16_list(ticks)
        return self.write_register(
            self.REG_ANGLE_SET,
            data_bytes,
            post_sleep=self.angle_write_settle_s,
        )
    
    def set_joint_speed(self, speed: int) -> bool:
        """设置全部13个自由度的速度（断电不保存，上电需重新设置）"""
        if not 0 <= speed <= 65535:
            self.logger.error(f"Invalid speed: {speed} (must be 0-65535)")
            return False

        data_bytes = self._pack_int16_list([speed] * 13)
        self.logger.info(f"set_joint_speed: all 13 DOF -> {speed}")
        return self.write_register(self.REG_SPEED_SET, data_bytes)

    def read_joint_angles(self) -> Optional[tuple]:
        """读取关节角度，返回 (angles_rad, raw_ticks) 元组"""
        data = self.read_register(self.REG_ANGLE_ACT, 26)
        if data is None or len(data) < 26:
            return None

        angles_rad = []
        raw_ticks = []
        for i in range(13):
            low_byte = data[2 * i]
            high_byte = data[2 * i + 1]
            value = (low_byte & 0xFF) + (high_byte << 8)
            raw_ticks.append(value)
            angles_rad.append(self._int16_to_angle(value))

        return (angles_rad, raw_ticks)
    
    def run_builtin_action(self, action_id: int) -> bool:
        if not 1 <= action_id <= 30:
            self.logger.error(f"Invalid action_id: {action_id} (must be 1-30)")
            return False
        
        try:
            index_bytes = self._pack_int16_list([action_id])
            if not self.write_register(self.REG_ACTION_SEQ, index_bytes):
                return False
            
            time.sleep(0.05)
            
            run_bytes = [0x01, 0x00]
            if not self.write_register(self.REG_ACTION_RUN, run_bytes):
                return False
            
            self.logger.info(f"Executed builtin action {action_id}")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to run builtin action: {e}")
            return False
    
    def close(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.logger.info("Serial port closed")


class DexterousHandNode(Node):
    """灵巧手ROS2控制节点"""

    # 关节名模板，side 占位符在运行时替换
    JOINT_NAME_TEMPLATE = [
        '{side}_pinky_mcp_joint',
        '{side}_ring_mcp_joint',
        '{side}_middle_mcp_joint',
        '{side}_middle_yaw_joint',
        '{side}_index_mcp_joint',
        '{side}_index_yaw_joint',
        '{side}_pinky_pip_joint',
        '{side}_ring_pip_joint',
        '{side}_middle_pip_joint',
        '{side}_index_pip_joint',
        '{side}_thumb_yaw_joint',
        '{side}_thumb_mcp_joint',
        '{side}_thumb_dip_joint',
    ]


    FINGER_RANGE_TEMPLATE = [
        (['{side}_pinky_mcp_joint'], [1650, 900], [0.0, 1.31]),
        (['{side}_ring_mcp_joint'], [1650, 900], [0.0, 1.31]),
        (['{side}_middle_mcp_joint'], [1650, 900], [0.0, 1.31]),
        (['{side}_middle_yaw_joint'], [-150, 150], [-0.26, 0.26]),
        (['{side}_index_mcp_joint'], [1650, 900], [0.0, 1.31]),
        (['{side}_index_yaw_joint'], [-150, 150], [-0.26, 0.26]),
        (['{side}_pinky_pip_joint'], [1900, 1050], [0.0, 1.48]),
        (['{side}_ring_pip_joint'], [1900, 1050], [0.0, 1.48]),
        (['{side}_middle_pip_joint'], [1900, 1050], [0.0, 1.48]),
        (['{side}_index_pip_joint'], [1900, 1050], [0.0, 1.48]),
        (['{side}_thumb_yaw_joint'], [1750, 650], [0, 1.92]),
        (['{side}_thumb_mcp_joint'], [1600, 1250], [0.1, 0.61]),
        (['{side}_thumb_dip_joint'], [2040, 1500], [0.1, 0.94]),
    ]

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        hand_id: int = 0x01,
        side: str = 'left',
        angle_write_settle_ms: float = 6.0,
        generic_post_write_ms: float = 25.0,
    ):
        if side not in ('left', 'right'):
            raise ValueError(f"side must be 'left' or 'right', got '{side}'")

        super().__init__(f'{side}_hand_node')
        self.side = side

        # 根据 side 动态生成手指映射配置
        self.finger_range = [
            (
                [name.format(side=side) for name in joint_names],
                actuator_range,
                limit_sum
            )
            for joint_names, actuator_range, limit_sum in self.FINGER_RANGE_TEMPLATE
        ]

        # 初始化硬件控制器（角度流式下发用更短 settle，减少对吞吐的限制）
        self.controller = DexterousHandController(
            port,
            baudrate,
            hand_id,
            self.get_logger(),
            post_write_sleep=generic_post_write_ms / 1000.0,
            angle_write_settle_s=angle_write_settle_ms / 1000.0,
        )

        # QoS
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        topic = f'/io_teleop/RH5DG2/joint_cmd_finger_{side}'
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            topic,
            self.joint_cmd_callback,
            qos_profile,
        )

        self.get_logger().info(f"Subscribing to: {topic}")

        # 设置所有自由度速度（断电不保存，每次上电需重新设置）
        self.get_logger().info("Setting all DOF speed to 2500...")
        self.controller.set_joint_speed(2500)

        # 初始化为张开状态
        self.get_logger().info("Initializing hand to open position...")
        if self.controller.run_builtin_action(2):
            time.sleep(0.5)
            self.get_logger().info("Hand initialized to open position")

        self.get_logger().info(
            f"[{side}] Node initialized with {len(self.finger_range)} actuator mappings"
        )

        # 创建定时器定期读取关节角度（每2秒读取一次）
        # self.angle_read_timer = self.create_timer(
        #     2.0,
        #     self._read_angles_callback,
        # )
        # self.get_logger().info("Angle feedback timer started (0.5Hz)")

    def _read_angles_callback(self):
        """定时读取关节角度反馈"""
        result = self.controller.read_joint_angles()
        if result is not None:
            angles_rad, raw_ticks = result
            self.get_logger().info(f"[{self.side}] Angles (rad): {[round(a, 4) for a in angles_rad]}")
            self.get_logger().info(f"[{self.side}] Raw ticks: {raw_ticks}")
        else:
            self.get_logger().warn(f"[{self.side}] Failed to read joint angles")

    def joint_cmd_callback(self, msg: JointState):
        if not msg.name or not msg.position:
            self.get_logger().warn("Received empty joint command")
            return

        joint_dic = dict(zip(msg.name, msg.position))

        angle_vals = []
        for joint_names, actuator_range, limit_sum in self.finger_range:
            total = sum(joint_dic.get(name, 0.0) for name in joint_names)
            ratio = (total - limit_sum[0]) / (limit_sum[1] - limit_sum[0])
            ratio = max(0.0, min(1.0, ratio))
            val = int(actuator_range[0] + ratio * (actuator_range[1] - actuator_range[0]))
            angle_vals.append(val)

        joint_info = ", ".join([f"{name}: {round(pos, 4)}" for name, pos in zip(msg.name, msg.position)])
        self.get_logger().info(f"[{self.side}] Received joint cmd: {joint_info}")
        self.get_logger().info(f"[{self.side}] Calculated ticks: {angle_vals}")

        self.controller.set_joint_angle_ticks(angle_vals)

    def destroy_node(self):
        self.controller.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    import argparse
    parser = argparse.ArgumentParser(description='Dexterous Hand ROS2 Controller')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0',
                        help='Serial port for single-hand mode (--side left|right) (default: /dev/ttyUSB0)')
    parser.add_argument('--left-port', type=str, default='/dev/ttyUSB0',
                        help='Left hand serial port for --side both (default: /dev/ttyUSB0)')
    parser.add_argument('--right-port', type=str, default='/dev/ttyUSB1',
                        help='Right hand serial port for --side both (default: /dev/ttyUSB1)')
    parser.add_argument('--baudrate', type=int, default=115200,
                        help='Serial baudrate (default: 115200)')
    parser.add_argument('--hand_id', type=int, default=0x01,
                        help='Hand ID for serial communication (default: 0x01)')
    parser.add_argument('--side', type=str, default='left', choices=['left', 'right', 'both'],
                        help='Which hand(s) to control: left, right, or both (default: left)')
    parser.add_argument('--angle-write-settle-ms', type=float, default=6.0,
                        help='关节角度寄存器每次写后的等待(ms)；过小可能影响可靠性，过大限制帧率 (default: 6)')
    parser.add_argument('--generic-post-write-ms', type=float, default=25.0,
                        help='内置动作/读寄存器等其它串口写后的等待(ms) (default: 25)')

    parsed_args, unknown = parser.parse_known_args()

    common_kw = dict(
        baudrate=parsed_args.baudrate,
        angle_write_settle_ms=parsed_args.angle_write_settle_ms,
        generic_post_write_ms=parsed_args.generic_post_write_ms,
    )

    nodes: List[DexterousHandNode] = []
    if parsed_args.side == 'both':
        if parsed_args.left_port == parsed_args.right_port:
            parser.error('--left-port and --right-port must differ when --side both')
        for port, side in (
            (parsed_args.left_port, 'left'),
            (parsed_args.right_port, 'right'),
        ):
            nodes.append(DexterousHandNode(
                port=port,
                hand_id=parsed_args.hand_id,
                side=side,
                **common_kw,
            ))
    else:
        nodes.append(DexterousHandNode(
            port=parsed_args.port,
            hand_id=parsed_args.hand_id,
            side=parsed_args.side,
            **common_kw,
        ))

    executor = SingleThreadedExecutor()
    for node in nodes:
        executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        for node in nodes:
            node.get_logger().info('Shutting down...')
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

