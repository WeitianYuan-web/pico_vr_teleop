#!/usr/bin/env python3
"""用与 teleop_realsense_publisher 匹配的 BEST_EFFORT QoS 测话题真实接收 FPS。

Jazzy 的 `ros2 topic hz` 默认 RELIABLE，与图像发布端不匹配，读数会严重偏低。
本节点订阅 QoS 与 IMAGE_QOS 一致，统计的是订阅端实际收到的帧率。
"""

from __future__ import annotations

import argparse
import os
import time
from collections import deque
from typing import Deque, Optional

# 与 publisher 使用同一 Fast DDS profile（大 SHM + 保留内置发现）。
PUBLISHER_DIR = os.path.dirname(os.path.abspath(__file__))
_PROFILE = os.path.join(PUBLISHER_DIR, "fastdds_local_image.xml")
os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", _PROFILE)
os.environ.setdefault("FASTDDS_DEFAULT_PROFILES_FILE", _PROFILE)

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message

IMAGE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class TopicHzBestEffort(Node):
    def __init__(
        self,
        topic: str,
        window_size: int,
        print_period_s: float,
        msg_type: Optional[str],
    ) -> None:
        super().__init__("topic_hz_best_effort")
        self._topic = topic
        self._window_size = max(2, window_size)
        self._print_period_s = max(0.1, print_period_s)
        self._stamps: Deque[float] = deque(maxlen=self._window_size)
        self._total = 0
        self._last_recv_mono: Optional[float] = None

        resolved_type = msg_type or self._discover_type(topic)
        msg_cls = get_message(resolved_type)
        # raw=True：不反序列化大包，避免订阅端解码拖慢测到的 FPS
        self.create_subscription(msg_cls, topic, self._on_msg, IMAGE_QOS, raw=True)
        self.create_timer(self._print_period_s, self._print_stats)

        self.get_logger().info(
            f"监听 {topic} ({resolved_type})，QoS=BEST_EFFORT/KEEP_LAST/depth=10/VOLATILE，raw"
        )

    def _discover_type(self, topic: str) -> str:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and rclpy.ok():
            names_and_types = dict(self.get_topic_names_and_types())
            types = names_and_types.get(topic)
            if types:
                return types[0]
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.0)
        raise RuntimeError(
            f"5s 内未发现话题 {topic}；可先启动 publisher，"
            "或用 --msg-type sensor_msgs/msg/Image"
        )

    def _on_msg(self, msg) -> None:
        now = time.monotonic()
        self._stamps.append(now)
        self._total += 1
        self._last_recv_mono = now
        del msg

    def _print_stats(self) -> None:
        now = time.monotonic()
        if len(self._stamps) < 2:
            age = (
                f"{now - self._last_recv_mono:.1f}s ago"
                if self._last_recv_mono is not None
                else "never"
            )
            self.get_logger().warn(
                f"{self._topic}: 尚无足够样本 (total={self._total}, last={age})"
            )
            return

        elapsed = self._stamps[-1] - self._stamps[0]
        hz = (len(self._stamps) - 1) / elapsed if elapsed > 0 else 0.0
        gaps = [
            self._stamps[i] - self._stamps[i - 1] for i in range(1, len(self._stamps))
        ]
        min_ms = min(gaps) * 1000.0
        max_ms = max(gaps) * 1000.0
        mean_ms = (sum(gaps) / len(gaps)) * 1000.0
        self.get_logger().info(
            f"{self._topic}: {hz:6.2f} Hz  "
            f"(window={len(self._stamps)}, total={self._total}, "
            f"dt min/mean/max={min_ms:.1f}/{mean_ms:.1f}/{max_ms:.1f} ms)"
        )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="BEST_EFFORT QoS 话题 Hz 监听（匹配 teleop_realsense_publisher）"
    )
    parser.add_argument(
        "topic",
        nargs="?",
        default="/camera_f/color/image_raw",
        help="话题名（默认 /camera_f/color/image_raw）",
    )
    parser.add_argument(
        "--window",
        type=int,
        default=60,
        help="滑动窗口样本数（默认 60）",
    )
    parser.add_argument(
        "--period",
        type=float,
        default=1.0,
        help="打印周期秒（默认 1.0）",
    )
    parser.add_argument(
        "--msg-type",
        default=None,
        help="消息类型；默认自动发现",
    )
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = TopicHzBestEffort(
        topic=args.topic,
        window_size=args.window,
        print_period_s=args.period,
        msg_type=args.msg_type,
    )
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
