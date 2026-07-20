#!/usr/bin/env python3
"""Teleop 状态 + RealSense 三相机 ROS2 发布。"""

from __future__ import annotations

import os
import sys
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import numpy as np

# ROS2 topic 名称、类型和 QoS 均不变。
# fastdds_local_image.xml：保留内置传输（兼容 ros2 CLI 发现），
# 另加足够大的 SHM segment，同机大图走共享内存。
PUBLISHER_DIR = os.path.dirname(os.path.abspath(__file__))
os.environ.setdefault(
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    os.path.join(PUBLISHER_DIR, "fastdds_local_image.xml"),
)
os.environ.setdefault(
    "FASTDDS_DEFAULT_PROFILES_FILE",
    os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"],
)

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    cv2 = None  # type: ignore[assignment]
    HAS_CV2 = False

try:
    import pyrealsense2 as rs

    HAS_REALSENSE = True
except ImportError:
    rs = None  # type: ignore[assignment]
    HAS_REALSENSE = False
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CompressedImage, Image, JointState

if PUBLISHER_DIR not in sys.path:
    sys.path.insert(0, PUBLISHER_DIR)

from teleop_state_bridge import SideTeleopState, TeleopStateReceiver  # noqa: E402


# 状态话题：小消息，depth=1 即可
SENSOR_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# 图像：加大 depth，短暂订阅卡顿时少丢帧（仍无法解决 900KB raw + 过小 UDP buffer）
IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


@dataclass
class CameraConfig:
    name: str
    topic: str
    compressed_topic: str
    frame_id: str
    serial_param: str


class RealSenseCameraPublisher:
    """
    单路 RealSense 彩色发布。

    默认发 JPEG CompressedImage（约几十 KB），避免本机 UDP rmem 过小时
    900KB raw Image 分片大量丢包导致订阅端只有十几 Hz。
    """

    def __init__(
        self,
        node: Node,
        camera_cfg: CameraConfig,
        width: int,
        height: int,
        fps: int,
        serial: Optional[str],
        publish_raw: bool,
        publish_compressed: bool,
        jpeg_quality: int,
    ) -> None:
        self.node = node
        self.camera_cfg = camera_cfg
        self.width = width
        self.height = height
        self.fps = fps
        self.serial = serial
        self.publish_raw = publish_raw
        self.publish_compressed = publish_compressed
        self.jpeg_quality = int(np.clip(jpeg_quality, 1, 100))
        self.raw_publisher = (
            node.create_publisher(Image, camera_cfg.topic, IMAGE_QOS) if publish_raw else None
        )
        self.compressed_publisher = (
            node.create_publisher(CompressedImage, camera_cfg.compressed_topic, IMAGE_QOS)
            if publish_compressed
            else None
        )
        self.pipeline: Optional[rs.pipeline] = None
        self.enabled = False
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._pipeline_lock = threading.Lock()
        self._consecutive_timeouts = 0
        self._last_timeout_log_mono = 0.0
        self._published_count = 0
        self._last_stats_mono = time.monotonic()
        self._jpeg_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality] if HAS_CV2 else []

    def start(self) -> None:
        if not self.serial:
            self.node.get_logger().warning(
                f"{self.camera_cfg.name} 未分配设备序列号，跳过图像发布。"
            )
            return
        if self.publish_compressed and not HAS_CV2:
            raise RuntimeError("publish_compressed 需要 opencv-python（cv2）")
        if not self.publish_raw and not self.publish_compressed:
            self.node.get_logger().warning(
                f"{self.camera_cfg.name} publish_raw/compressed 均为 false，跳过。"
            )
            return

        self._start_pipeline()
        self.enabled = True
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._publish_loop,
            name=f"{self.camera_cfg.name}_publisher",
            daemon=True,
        )
        self._thread.start()
        modes = []
        if self.publish_compressed:
            modes.append(f"jpeg@{self.camera_cfg.compressed_topic}")
        if self.publish_raw:
            modes.append(f"raw@{self.camera_cfg.topic}")
        self.node.get_logger().info(
            f"{self.camera_cfg.name} 已启动: serial={self.serial}, "
            f"{self.width}x{self.height}@{self.fps}, {', '.join(modes)}"
        )

    def _configure_device(self, profile: rs.pipeline_profile) -> None:
        """缩小驱动队列并优先帧率，降低 USB/AE 造成的长间隔。"""
        try:
            device = profile.get_device()
            for sensor in device.query_sensors():
                if sensor.supports(rs.option.frames_queue_size):
                    sensor.set_option(rs.option.frames_queue_size, 1.0)
            color_sensor = device.first_color_sensor()
            if color_sensor is not None and color_sensor.supports(
                rs.option.auto_exposure_priority
            ):
                # 0 = 优先帧率
                color_sensor.set_option(rs.option.auto_exposure_priority, 0.0)
        except Exception as exc:
            self.node.get_logger().warning(
                f"{self.camera_cfg.name} 设备选项设置失败（可忽略）: {exc!r}"
            )

    def _start_pipeline(self) -> None:
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(
            rs.stream.color, self.width, self.height, rs.format.rgb8, self.fps
        )
        pipeline = rs.pipeline()
        profile = pipeline.start(config)
        self._configure_device(profile)
        with self._pipeline_lock:
            self.pipeline = pipeline

    def _restart_pipeline(self) -> None:
        """连续取帧超时时重启该相机，尝试恢复短暂 USB/UVC 故障。"""
        if self._stop_event.is_set():
            return
        with self._pipeline_lock:
            old_pipeline = self.pipeline
            self.pipeline = None
        if old_pipeline is not None:
            try:
                old_pipeline.stop()
            except Exception:
                pass
        self.node.get_logger().warning(
            f"{self.camera_cfg.name} 连续取帧超时，正在重启 RealSense pipeline"
        )
        time.sleep(1.0)
        if self._stop_event.is_set():
            return
        self._start_pipeline()
        self._consecutive_timeouts = 0
        self.node.get_logger().info(f"{self.camera_cfg.name} pipeline 重启成功")

    def stop(self) -> None:
        self.enabled = False
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        with self._pipeline_lock:
            pipeline = self.pipeline
            self.pipeline = None
        if pipeline is not None:
            try:
                pipeline.stop()
            except Exception:
                pass

    def _publish_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self.wait_and_publish()
            except RuntimeError as exc:
                if self._stop_event.is_set() or not self.enabled or self.pipeline is None:
                    break
                msg = str(exc)
                if "before start" in msg or "not started" in msg:
                    break
                self._consecutive_timeouts += 1
                now = time.monotonic()
                if now - self._last_timeout_log_mono >= 5.0:
                    self.node.get_logger().warning(
                        f"{self.camera_cfg.name} 等待图像超时或失败 "
                        f"(连续 {self._consecutive_timeouts} 次): {exc!r}"
                    )
                    self._last_timeout_log_mono = now
                if self._consecutive_timeouts >= 6:
                    try:
                        self._restart_pipeline()
                    except Exception as restart_exc:
                        self.node.get_logger().error(
                            f"{self.camera_cfg.name} pipeline 重启失败: {restart_exc!r}"
                        )
                        time.sleep(2.0)
                time.sleep(0.005)
            except Exception as exc:
                if self._stop_event.is_set() or not self.enabled:
                    break
                self.node.get_logger().error(f"{self.camera_cfg.name} 发布图像失败: {exc!r}")
                time.sleep(0.01)

    def wait_and_publish(self) -> None:
        pipeline = self.pipeline
        if not self.enabled or pipeline is None:
            return

        frames = pipeline.wait_for_frames(500)
        color_frame = frames.get_color_frame()
        if not color_frame:
            return
        self._consecutive_timeouts = 0

        # 取最新帧：队列里还有更新的则丢旧帧
        while True:
            more = pipeline.poll_for_frames()
            if not more:
                break
            color_more = more.get_color_frame()
            if color_more:
                color_frame = color_more

        stamp = self.node.get_clock().now().to_msg()
        # asanyarray 零拷贝视图；后续 tobytes / imencode 才会真正拷贝
        img = np.asanyarray(color_frame.get_data())

        if self.compressed_publisher is not None:
            bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            ok, buf = cv2.imencode(".jpg", bgr, self._jpeg_params)
            if not ok:
                raise RuntimeError("cv2.imencode(.jpg) failed")
            cmsg = CompressedImage()
            cmsg.header.stamp = stamp
            cmsg.header.frame_id = self.camera_cfg.frame_id
            cmsg.format = "jpeg"
            cmsg.data = buf.tobytes()
            self.compressed_publisher.publish(cmsg)

        if self.raw_publisher is not None:
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = self.camera_cfg.frame_id
            msg.height = int(img.shape[0])
            msg.width = int(img.shape[1])
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = msg.width * 3
            msg.data = img.tobytes()
            self.raw_publisher.publish(msg)

        self._published_count += 1
        now = time.monotonic()
        if now - self._last_stats_mono >= 5.0:
            rate = self._published_count / (now - self._last_stats_mono)
            self.node.get_logger().info(f"{self.camera_cfg.name} 发布帧率: {rate:.1f} Hz")
            self._published_count = 0
            self._last_stats_mono = now


class TeleopPublisherNode(Node):
    def __init__(self) -> None:
        super().__init__("teleop_realsense_publisher")

        self.declare_parameter("camera_width", 640)
        self.declare_parameter("camera_height", 480)
        self.declare_parameter("camera_fps", 30)
        # 过高会与三路图像抢 GIL/DDS；30Hz 足够下游使用
        self.declare_parameter("placeholder_hz", 30.0)
        self.declare_parameter("state_udp_host", "127.0.0.1")
        self.declare_parameter("state_udp_port", 17981)
        self.declare_parameter("state_stale_timeout_s", 1.0)
        self.declare_parameter("camera_f_serial", "")
        self.declare_parameter("camera_l_serial", "")
        self.declare_parameter("camera_r_serial", "")
        # compressed 为可选小包备份，默认关
        self.declare_parameter("publish_raw_image", True)
        self.declare_parameter("publish_compressed_image", False)
        self.declare_parameter("jpeg_quality", 80)

        self.camera_width = int(self.get_parameter("camera_width").value)
        self.camera_height = int(self.get_parameter("camera_height").value)
        self.camera_fps = int(self.get_parameter("camera_fps").value)
        self.placeholder_hz = float(self.get_parameter("placeholder_hz").value)
        self.state_udp_host = str(self.get_parameter("state_udp_host").value)
        self.state_udp_port = int(self.get_parameter("state_udp_port").value)
        self.state_stale_timeout_s = float(self.get_parameter("state_stale_timeout_s").value)
        self.publish_raw_image = bool(self.get_parameter("publish_raw_image").value)
        self.publish_compressed_image = bool(
            self.get_parameter("publish_compressed_image").value
        )
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)

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
                compressed_topic="/camera_f/color/image_raw/compressed",
                frame_id="camera_f_color_optical_frame",
                serial_param="camera_f_serial",
            ),
            CameraConfig(
                name="camera_l",
                topic="/camera_l/color/image_raw",
                compressed_topic="/camera_l/color/image_raw/compressed",
                frame_id="camera_l_color_optical_frame",
                serial_param="camera_l_serial",
            ),
            CameraConfig(
                name="camera_r",
                topic="/camera_r/color/image_raw",
                compressed_topic="/camera_r/color/image_raw/compressed",
                frame_id="camera_r_color_optical_frame",
                serial_param="camera_r_serial",
            ),
        ]

        self.cameras: List[RealSenseCameraPublisher] = []
        if HAS_REALSENSE:
            self.cameras = self._create_camera_publishers()
            for camera in self.cameras:
                try:
                    camera.start()
                except Exception as exc:
                    self.get_logger().error(
                        f"{camera.camera_cfg.name} 启动失败: {exc!r}，该相机将不会发布。"
                    )
        else:
            self.get_logger().warning(
                "未安装 pyrealsense2，跳过相机话题；/puppet/* 仍会正常发布。"
            )

        placeholder_period = 1.0 / max(self.placeholder_hz, 1e-3)
        self.placeholder_timer = self.create_timer(placeholder_period, self._on_state_timer)
        camera_hint = "RealSense 相机已启用" if HAS_REALSENSE else "相机未启用(pyrealsense2 缺失)"
        img_modes = []
        if self.publish_compressed_image:
            img_modes.append(f"jpeg q={self.jpeg_quality}")
        if self.publish_raw_image:
            img_modes.append("raw rgb8")
        self.get_logger().info(
            f"teleop 发布者已启动：{camera_hint}；"
            f"图像目标 {self.camera_width}x{self.camera_height}@{self.camera_fps} "
            f"[{'|'.join(img_modes) or '无'}]；"
            f"状态 {self.placeholder_hz:.0f} Hz；"
            f"臂/手 udp://{self.state_udp_host}:{self.state_udp_port}"
        )

    def _list_device_serials(self) -> List[str]:
        if not HAS_REALSENSE:
            return []
        # USB 抖动时偶发 map_device_descriptor /dev/video* 不存在，重试几次。
        last_exc: Optional[BaseException] = None
        for attempt in range(1, 6):
            try:
                ctx = rs.context()
                serials: List[str] = []
                for dev in ctx.query_devices():
                    serial = dev.get_info(rs.camera_info.serial_number)
                    if serial:
                        serials.append(serial)
                serials.sort()
                return serials
            except Exception as exc:
                last_exc = exc
                self.get_logger().warning(
                    f"枚举 RealSense 失败 (第 {attempt}/5 次): {exc!r}"
                )
                time.sleep(0.5 * attempt)
        self.get_logger().error(f"枚举 RealSense 最终失败: {last_exc!r}")
        return []

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
                    publish_raw=self.publish_raw_image,
                    publish_compressed=self.publish_compressed_image,
                    jpeg_quality=self.jpeg_quality,
                )
            )
        return cameras

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
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
