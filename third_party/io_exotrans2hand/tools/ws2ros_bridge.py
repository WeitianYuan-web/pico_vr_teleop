"""python3 tools/ws2ros_bridge.py --hands DexcelRobotics_Apex DexcelRobotics_Apex_1"""
from __future__ import annotations

import argparse
import asyncio
import json
import os
import signal
import sys
import threading
from queue import Empty, Full, Queue
from typing import Any

import rclpy
import urllib.request
import websockets
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import JointState, Joy
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage

REST_API = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080").rstrip("/")
WS_URL = os.environ.get("GATEWAY_WS", "ws://127.0.0.1:8080/ws")

_running = True



def _on_signal(_sig, _frame):
    global _running
    _running = False
    print("接收到信号，程序退出")


# stream_id 与 ROS topic（与 gateway.yaml streams 一致）
GLOBAL_STREAMS = [
    ("io_esk.tf", "/io_fusion/tf_exoskeleton", "TFMessage"),
    ("io_esk.joint_data", "/io_esk/joint_data", "JointState"),
    ("io_esk.joystick_data", "/io_esk/joystick_data", "Joy"),
]
# ROS 订阅 -> WS 发布
ROS2WS_STREAMS = [
    ("io_esk.vibration_feedback", "/io_esk/vibration_feedback", "Float64MultiArray"),
]
HAND_STREAMS = [
    ("io_align.tf.{hand}", "/io_align/{hand}/tf_hand", "TFMessage"),
    # ("io_align.poses_left.{hand}", "/io_align/{hand}/poses_left_hand_ee_link", "PoseArray"),
    # ("io_align.poses_right.{hand}", "/io_align/{hand}/poses_right_hand_ee_link", "PoseArray"),
    ("io_teleop.joint_cmd_left.{hand}", "/io_teleop/{hand}/joint_cmd_finger_left", "JointState"),
    ("io_teleop.joint_cmd_right.{hand}", "/io_teleop/{hand}/joint_cmd_finger_right", "JointState"),
]

ROS_MSG_TYPES = {
    "TFMessage": TFMessage,
    "JointState": JointState,
    "Joy": Joy,
    "PoseArray": PoseArray,
    "Float64MultiArray": Float64MultiArray,
}

# 自动获取streams内的id，并与手有关的分离出hand名称
def get_stream_id_list():
    with urllib.request.urlopen(f"{REST_API}/api/v1/streams", timeout=5) as resp:
        items = json.loads(resp.read())
    return [x["id"] for x in items if x.get("direction") == "subscribe" and x.get("id")]

# 获取灵巧手名称
def get_hand_list():
    hand_name_list = []
    for stream_id in get_stream_id_list():
        if stream_id.startswith("io_align.tf."):
            hand_name_list.append(stream_id.split(".")[2])
    return hand_name_list


# 桥接topic名称
def get_topic_list():
    topic_list = list(GLOBAL_STREAMS)
    hands = get_hand_list()
    print(f'hands: {hands}')
    if hands:
        for hand in hands:
            for stream_id, ros_topic, msg_type in HAND_STREAMS:
                topic_list.append(
                    (stream_id.format(hand=hand), ros_topic.format(hand=hand), msg_type)
                )
    return topic_list


def _ns_to_time(stamp_ns):
    t = Time()
    t.sec = stamp_ns // 1_000_000_000
    t.nanosec = stamp_ns % 1_000_000_000
    return t

# 格式校验
def _to_tf_msg(d):
    msg = TFMessage()
    for t in d["transforms"]:
        ts = TransformStamped()
        ts.header.stamp = _ns_to_time(d["stamp_ns"])
        ts.header.frame_id = t["parent"]
        ts.child_frame_id = t["child"]
        tx, ty, tz = t["translation"][:3]
        qx, qy, qz, qw = t["rotation"][:4]
        ts.transform.translation.x = tx
        ts.transform.translation.y = ty
        ts.transform.translation.z = tz
        ts.transform.rotation.x = qx
        ts.transform.rotation.y = qy
        ts.transform.rotation.z = qz
        ts.transform.rotation.w = qw
        msg.transforms.append(ts)
    return msg


def _to_joint_state_msg(d):
    msg = JointState()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    msg.name = list(d["names"])
    msg.position = list(d["position"])
    msg.velocity = list(d["velocity"])
    msg.effort = list(d["effort"])
    return msg


def _to_joy_msg(d):
    msg = Joy()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    msg.axes = [float(x) for x in d["axes"]]
    msg.buttons = [int(x) for x in d["buttons"]]
    return msg


def _to_pose_array_msg(d):
    msg = PoseArray()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    for p in d["poses"]:
        pose = Pose()
        px, py, pz = p["position"][:3]
        qx, qy, qz, qw = p["orientation"][:4]
        pose.position.x = px
        pose.position.y = py
        pose.position.z = pz
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        msg.poses.append(pose)
    return msg


def _to_float64_array_msg(d):
    msg = Float64MultiArray()
    msg.data = [float(x) for x in d["data"]]
    return msg


def _to_ros_msg(msg_type, d):
    if msg_type == "TFMessage":
        return _to_tf_msg(d)
    if msg_type == "JointState":
        return _to_joint_state_msg(d)
    if msg_type == "Joy":
        return _to_joy_msg(d)
    if msg_type == "PoseArray":
        return _to_pose_array_msg(d)
    if msg_type == "Float64MultiArray":
        return _to_float64_array_msg(d)
    return None


class WS2RosBridge(Node):
    def __init__(self):
        super().__init__("ws2ros_bridge")

        self._ws_url = WS_URL
        self._pending: Queue[tuple[Publisher, Any]] = Queue(maxsize=128)
        self._publish_queue: Queue[tuple[str, dict]] = Queue(maxsize=64)
        self._routes: dict[str, tuple[Publisher, str]] = {}  # stream_id -> (pub, msg_type)
        self._topic_list = get_topic_list()
        self._stream_ids = [sid for sid, _, _ in self._topic_list]
        self._ws_running = True
        self._loop: asyncio.AbstractEventLoop = None
        self._thread: threading.Thread = None

        self.create_timer(0.001, self._flush)

        # 转ros
        for stream_id, ros_topic, msg_type in self._topic_list:
            ros_cls = ROS_MSG_TYPES[msg_type]
            pub = self.create_publisher(ros_cls, ros_topic, 10)
            self._routes[stream_id] = (pub, msg_type)
            self.get_logger().info(f"{ros_topic} ({msg_type})")
        # 订阅ros振动
        for stream_id, ros_topic, msg_type in ROS2WS_STREAMS:
            ros_cls = ROS_MSG_TYPES[msg_type]
            self.create_subscription(
                ros_cls,
                ros_topic,
                lambda msg, sid=stream_id: self._on_ros_publish(sid, msg),
                10,
            )
            self.get_logger().info(f"{ros_topic} ({msg_type}) -> {stream_id}")
        # 启动ws线程
        self._start_ws_thread()
    
    # 发布到ws
    def _on_ros_publish(self, stream_id, msg):
        try:
            self._publish_queue.put_nowait((stream_id, {"data": list(msg.data)}))
        except Full:
            try:
                self._publish_queue.get_nowait()
            except Empty:
                pass
            self._publish_queue.put_nowait((stream_id, {"data": list(msg.data)}))
        except Exception as e:
            print(f"error: {e}")
    
    # 接收ws数据
    def _on_ws_message(self, stream_id, data):
        route = self._routes.get(stream_id)
        if route is None:
            return
        pub, msg_type = route
        try:
            ros_msg = _to_ros_msg(msg_type, data)
            # 缓存队列中添加消息
            if ros_msg is not None:
                self._pending.put_nowait((pub, ros_msg))
            # 缓存满了就丢弃最旧的一条
        except Full:
            try:
                self._pending.get_nowait()
            except Empty:
                pass
            self._pending.put_nowait((pub, ros_msg))
        except Exception as e:
            print(f"error: {e}")

    # 启动ws线程
    def _start_ws_thread(self):
        self._thread = threading.Thread(target=self._run_ws_loop, daemon=True)
        self._thread.start()

    # 运行ws线程
    def _run_ws_loop(self):
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(self._ws_loop())
        finally:
            self._loop.close()
            self._loop = None

    # 运行ws循环
    async def _ws_loop(self):
        while self._ws_running and _running:
            try:
                async with websockets.connect(self._ws_url) as ws:
                    await ws.send(
                        json.dumps({"op": "subscribe", "streams": self._stream_ids})
                    )
                    self.get_logger().info(
                        f"WebSocket 已连接，订阅 {len(self._stream_ids)} 个流"
                    )
                    while self._ws_running and _running:
                        # 发送到ws
                        await self._flush_ws_publish(ws)
                        try:
                            # 获取ws数据
                            raw = await asyncio.wait_for(ws.recv(), timeout=0.05)
                        except asyncio.TimeoutError:
                            continue
                        try:
                            # 解析ws数据
                            obj = json.loads(raw)
                        except json.JSONDecodeError:
                            continue
                        # 判断ws数据是否有效
                        if obj.get("op") in ("published", "error"):
                            continue
                        stream_id = obj.get("stream")
                        data = obj.get("data")
                        if stream_id and isinstance(data, dict):
                            # 转ws
                            self._on_ws_message(stream_id, data)
            except Exception as exc:
                if self._ws_running and _running:
                    self.get_logger().warning(f"WebSocket 断开: {exc}，1s 后重连")
                    await asyncio.sleep(1.0)

    # 发布到ws
    async def _flush_ws_publish(self, ws):
        while self._ws_running:
            try:
                stream_id, data = self._publish_queue.get_nowait()
            except Empty:
                break
            try:
                await ws.send(
                    json.dumps({"op": "publish", "stream": stream_id, "data": data})
                )
            except Exception:
                break
    # 发布到ros
    def _flush(self):
        while self._ws_running:
            try:
                pub, msg = self._pending.get_nowait()
            except Empty:
                break
            pub.publish(msg)

    # 关闭ws2ros桥接
    def close(self):
        self._ws_running = False
        if self._loop is not None and self._loop.is_running():
            self._loop.call_soon_threadsafe(lambda: None)
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None


def main():
    # 设置信号处理
    global _running
    _running = True
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = None
    try:
        rclpy.init()
        node = WS2RosBridge()
        while _running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os._exit(0)


if __name__ == "__main__":
    main()
    # try:
    #     print(create_streams_list())
    # except Exception as e:
    #     print(f"error: {e}")
    # finally:
    #     sys.exit(0)