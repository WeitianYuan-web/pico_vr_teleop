#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional

import pyrealsense2 as rs
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState

PUBLISHER_DIR = os.path.dirname(os.path.abspath(__file__))
if PUBLISHER_DIR not in sys.path:
    sys.path.insert(0, PUBLISHER_DIR)

from teleop_state_bridge import SideTeleopState, TeleopStateReceiver  # noqa: E402


SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass
class CameraConfig:
    name: str
    topic: str
    frame_id: str
    serial_param: str


class RealSenseCameraPublisher:
    def __init__(
        self,
        node: Node,
        camera_cfg: CameraConfig,
        width: int,
        height: int,
        fps: int,
        serial: Optional[str],
    ) -> None:
        self.node = node
        self.camera_cfg = camera_cfg
        self.width = width
        self.height = height
        self.fps = fps
        self.serial = serial
        self.publisher = node.create_publisher(Image, camera_cfg.topic, SENSOR_QOS)
        self.pipeline: Optional[rs.pipeline] = None
        self.enabled = False

    def start(self) -> None:
        if not self.serial:
            self.node.get_logger().warning(
                f"{self.camera_cfg.name} 未分配设备序列号，跳过图像发布。"
            )
            return
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps)
        self.pipeline = rs.pipeline()
        self.pipeline.start(config)
        self.enabled = True
        self.node.get_logger().info(
            f"{self.camera_cfg.name} 已启动: serial={self.serial}, "
            f"{self.width}x{self.height}@{self.fps}"
        )

    def stop(self) -> None:
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
        self.enabled = False

    def poll_and_publish(self, stamp_msg) -> None:
        if not self.enabled or self.pipeline is None:
            return
        frames = self.pipeline.poll_for_frames()
        if not frames:
            return
        color_frame = frames.get_color_frame()
        if not color_frame:
            return

        msg = Image()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = self.camera_cfg.frame_id
        msg.height = color_frame.get_height()
        msg.width = color_frame.get_width()
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = memoryview(color_frame.get_data()).tobytes()
        self.publisher.publish(msg)


class TeleopPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_realsense_publisher")

        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        self.declare_parameter("placeholder_hz", 120.0)
        self.declare_parameter("state_udp_host", "127.0.0.1")
        self.declare_parameter("state_udp_port", 17981)
        self.declare_parameter("state_stale_timeout_s", 1.0)
        self.declare_parameter("camera_f_serial", "")
        self.declare_parameter("camera_l_serial", "")
        self.declare_parameter("camera_r_serial", "")

        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.camera_fps = int(self.get_parameter("camera_fps").value)
        self.placeholder_hz = float(self.get_parameter("placeholder_hz").value)
        self.state_udp_host = str(self.get_parameter("state_udp_host").value)
        self.state_udp_port = int(self.get_parameter("state_udp_port").value)
        self.state_stale_timeout_s = float(self.get_parameter("state_stale_timeout_s").value)

        self.joint_left_pub = self.create_publisher(JointState, "/puppet/joint_left", SENSOR_QOS)
        self.joint_right_pub = self.create_publisher(JointState, "/puppet/joint_right", SENSOR_QOS)
        self.pose_left_pub = self.create_publisher(PoseStamped, "/puppet/end_pose_left", SENSOR_QOS)
        self.pose_right_pub = self.create_publisher(PoseStamped, "/puppet/end_pose_right", SENSOR_QOS)
        self.hand_left_pub = self.create_publisher(JointState, "/puppet/hand_left", SENSOR_QOS)
        self.hand_right_pub = self.create_publisher(JointState, "/puppet/hand_right", SENSOR_QOS)

        self.arm_joint_names = [f"joint_{i}" for i in range(1, 7)]
        self.hand_joint_names = [f"finger_{i}" for i in range(1, 7)]

        self.state_receiver = TeleopStateReceiver(
            host=self.state_udp_host,
            port=self.state_udp_port,
            stale_timeout_s=self.state_stale_timeout_s,
        )

        self.camera_configs = [
            CameraConfig(
                name="camera_f",
                topic="/camera_f/color/image_raw",
                frame_id="camera_f_color_optical_frame",
                serial_param="camera_f_serial",
            ),
            CameraConfig(
                name="camera_l",
                topic="/camera_l/color/image_raw",
                frame_id="camera_l_color_optical_frame",
                serial_param="camera_l_serial",
            ),
            CameraConfig(
                name="camera_r",
                topic="/camera_r/color/image_raw",
                frame_id="camera_r_color_optical_frame",
                serial_param="camera_r_serial",
            ),
        ]

        self.cameras = self._create_camera_publishers()
        for camera in self.cameras:
            try:
                camera.start()
            except Exception as exc:
                self.get_logger().error(
                    f"{camera.camera_cfg.name} 启动失败: {exc!r}，该相机将不会发布。"
                )

        camera_period = 1.0 / max(self.camera_fps, 1)
        placeholder_period = 1.0 / max(self.placeholder_hz, 1e-3)
        self.camera_timer = self.create_timer(camera_period, self._on_camera_timer)
        self.placeholder_timer = self.create_timer(placeholder_period, self._on_state_timer)
        self.get_logger().info(
            "teleop 发布者已启动：RealSense 发布相机图像；"
            f"臂/手状态监听 udp://{self.state_udp_host}:{self.state_udp_port}，"
            "收到遥操作数据后发布真实 /puppet/* 话题。"
        )

    def _list_device_serials(self) -> List[str]:
        ctx = rs.context()
        serials: List[str] = []
        for dev in ctx.query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            if serial:
                serials.append(serial)
        serials.sort()
        return serials

    def _resolve_camera_serials(self) -> Dict[str, Optional[str]]:
        available = self._list_device_serials()
        used = set()
        mapping: Dict[str, Optional[str]] = {}

        for camera_cfg in self.camera_configs:
            serial = str(self.get_parameter(camera_cfg.serial_param).value).strip()
            if serial:
                mapping[camera_cfg.name] = serial
                used.add(serial)
            else:
                mapping[camera_cfg.name] = None

        free_serials = [s for s in available if s not in used]
        free_idx = 0
        for camera_cfg in self.camera_configs:
            if mapping[camera_cfg.name] is None and free_idx < len(free_serials):
                mapping[camera_cfg.name] = free_serials[free_idx]
                free_idx += 1

        if available:
            self.get_logger().info(f"检测到 RealSense 设备: {available}")
        else:
            self.get_logger().warning("未检测到 RealSense 设备。")
        return mapping

    def _create_camera_publishers(self) -> List[RealSenseCameraPublisher]:
        serial_mapping = self._resolve_camera_serials()
        cameras: List[RealSenseCameraPublisher] = []
        for camera_cfg in self.camera_configs:
            cameras.append(
                RealSenseCameraPublisher(
                    node=self,
                    camera_cfg=camera_cfg,
                    width=self.camera_width,
                    height=self.camera_height,
                    fps=self.camera_fps,
                    serial=serial_mapping.get(camera_cfg.name),
                )
            )
        return cameras

    def _on_camera_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for camera in self.cameras:
            try:
                camera.poll_and_publish(stamp)
            except Exception as exc:
                self.get_logger().error(f"{camera.camera_cfg.name} 发布图像失败: {exc!r}")

    def _make_joint_state(
        self,
        stamp_msg,
        frame_id: str,
        names: List[str],
        positions: Optional[List[float]] = None,
    ) -> JointState:
        msg = JointState()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = frame_id
        msg.name = names
        if positions is None:
            positions = [0.0] * len(names)
        elif len(positions) < len(names):
            positions = list(positions) + [0.0] * (len(names) - len(positions))
        else:
            positions = positions[: len(names)]
        msg.position = [float(v) for v in positions]
        msg.velocity = [0.0] * len(names)
        msg.effort = [0.0] * len(names)
        return msg

    def _make_pose(
        self,
        stamp_msg,
        frame_id: str,
        end_pose: Optional[dict[str, float]] = None,
    ) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = stamp_msg
        msg.header.frame_id = frame_id
        if end_pose is None:
            msg.pose.position.x = 0.0
            msg.pose.position.y = 0.0
            msg.pose.position.z = 0.0
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = 0.0
            msg.pose.orientation.w = 1.0
            return msg
        msg.pose.position.x = float(end_pose.get("x", 0.0))
        msg.pose.position.y = float(end_pose.get("y", 0.0))
        msg.pose.position.z = float(end_pose.get("z", 0.0))
        msg.pose.orientation.x = float(end_pose.get("qx", 0.0))
        msg.pose.orientation.y = float(end_pose.get("qy", 0.0))
        msg.pose.orientation.z = float(end_pose.get("qz", 0.0))
        msg.pose.orientation.w = float(end_pose.get("qw", 1.0))
        return msg

    def _side_from_snapshot(
        self, side: SideTeleopState | None
    ) -> tuple[dict[str, float], list[float], list[float]]:
        if side is None:
            return {}, [], []
        arm_pose = side.end_pose if side.arm_valid else {}
        arm_joints = side.arm_joints if side.arm_valid else []
        hand_joints = side.hand_joints if side.hand_valid else []
        return arm_pose, arm_joints, hand_joints

    def _on_state_timer(self) -> None:
        stamp = self.get_clock().now().to_msg()
        snapshot = self.state_receiver.get_latest()

        left_pose, left_arm_joints, left_hand_joints = self._side_from_snapshot(
            snapshot.left if snapshot else None
        )
        right_pose, right_arm_joints, right_hand_joints = self._side_from_snapshot(
            snapshot.right if snapshot else None
        )

        self.joint_left_pub.publish(
            self._make_joint_state(stamp, "base_left", self.arm_joint_names, left_arm_joints)
        )
        self.joint_right_pub.publish(
            self._make_joint_state(stamp, "base_right", self.arm_joint_names, right_arm_joints)
        )
        self.pose_left_pub.publish(self._make_pose(stamp, "base_left", left_pose or None))
        self.pose_right_pub.publish(self._make_pose(stamp, "base_right", right_pose or None))
        self.hand_left_pub.publish(
            self._make_joint_state(stamp, "hand_left", self.hand_joint_names, left_hand_joints)
        )
        self.hand_right_pub.publish(
            self._make_joint_state(stamp, "hand_right", self.hand_joint_names, right_hand_joints)
        )

    def destroy_node(self) -> bool:
        try:
            self.state_receiver.close()
        except Exception as exc:
            self.get_logger().warning(f"状态接收器关闭失败: {exc!r}")
        for camera in self.cameras:
            try:
                camera.stop()
            except Exception as exc:
                self.get_logger().warning(f"{camera.camera_cfg.name} 停止失败: {exc!r}")
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TeleopPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
