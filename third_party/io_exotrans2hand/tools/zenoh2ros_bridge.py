'''python3 tools/zenoh2ros_bridge.py --hands  DexcelRobotics_Apex DexcelRobotics_Apex_1'''
import argparse
import os
import signal
import sys
import time
from queue import Empty, Full, Queue
from typing import Any, Callable

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray, TransformStamped
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import Imu, JointState, Joy
from std_msgs.msg import Float64MultiArray
from tf2_msgs.msg import TFMessage
import zenoh

from io_bus_proto.io_bus_codec import encode_float64_multi_array, proto_to_dict

zenoh_cf = zenoh.Config.from_file('./configs/config/zenoh.json5')

# 全局变量，用于控制程序运行
_running = True
def _on_signal(_sig, _frame):
    global _running
    _running = False
    print("接收到信号，程序退出")

# msg_type: TFMessage | JointState | Joy | PoseArray | Float64MultiArray
# 固定名称key与topic
GLOBAL_TOPICS = [
    ("io_fusion/tf_exoskeleton", "/io_fusion/tf_exoskeleton", "TFMessage"),
    ("io_esk/joint_data", "/io_esk/joint_data", "JointState"),
    ("io_esk/joystick_data", "/io_esk/joystick_data", "Joy"),
    ("io_esk/imu_data_right", "/io_esk/imu_data_right", "Imu"),
    ("io_esk/imu_data_left", "/io_esk/imu_data_left", "Imu"),
]
# ROS 订阅 -> Zenoh 发布（外骨骼等设备从 Zenoh 收振动指令）
ROS2ZENOH_TOPICS = [
    ("io_esk/vibration_feedback", "/io_esk/vibration_feedback", "Float64MultiArray"),
]
# 灵巧手名称key与topic
HAND_TOPICS = [
    ("io_align/{hand}/tf_hand", "/io_align/{hand}/tf_hand", "TFMessage"),
    # ("io_align/{hand}/poses_left_hand_ee_link", "/io_align/{hand}/poses_left_hand_ee_link", "PoseArray"),
    # ("io_align/{hand}/poses_right_hand_ee_link", "/io_align/{hand}/poses_right_hand_ee_link", "PoseArray"),
    ("io_teleop/{hand}/joint_cmd_finger_left", "/io_teleop/{hand}/joint_cmd_finger_left", "JointState"),
    ("io_teleop/{hand}/joint_cmd_finger_right", "/io_teleop/{hand}/joint_cmd_finger_right", "JointState"),
]
# ros类型
ROS_MSG_TYPES = {
    "TFMessage": TFMessage,
    "JointState": JointState,
    "Joy": Joy,
    "Imu": Imu,
    "PoseArray": PoseArray,
    "Float64MultiArray": Float64MultiArray,
}
# 获取当前有数据的所有key
def get_active_keys(duration=1):
    t0=time.time()
    keys=[]
    with zenoh.open(zenoh_cf) as session:
        # 订阅所有key,打印key
        sub = session.declare_subscriber('**')
        for sample in sub:
            if sample.key_expr not in keys:
                keys.append(str(sample.key_expr))
            if time.time()-t0>=duration:
                break
    
    # print(f'active keys: {keys}')
    return keys
# 提取key中的hand名称
def get_hand_names():
    keys = get_active_keys()
    hand_names = []
    for key in keys:
        if 'tf_hand' in key:
            hand_names.append(key.split('/')[1])
    return hand_names
# 获取所有需要转换的zenoh key与topic-----根据传入的灵巧手名称，添加对应的key与topic
def create_topics_list():
    topics = list(GLOBAL_TOPICS)
    hands = get_hand_names()
    print(f'hands: {hands}')
    if hands:
        for hand in hands:
            for zenoh_key, ros_topic, msg_type in HAND_TOPICS:
            
                # 添加对应的key与topic
                topics.append((zenoh_key.format(hand=hand), ros_topic.format(hand=hand), msg_type))
    return topics

# 转成ros时间戳
def _ns_to_time(stamp_ns):
    t = Time()
    t.sec = stamp_ns // 1_000_000_000
    t.nanosec = stamp_ns % 1_000_000_000
    return t

# tf消息转换
def _to_tf_msg(d):
    msg = TFMessage()
    for t in d["transforms"]:
        ts = TransformStamped()
        ts.header.stamp = _ns_to_time(d["stamp_ns"])
        ts.header.frame_id = t["parent"]
        ts.child_frame_id = t["child"]
        tx, ty, tz = t["translation"][:3]
        qx, qy, qz, qw = t["rotation"][:4]
        ts.transform.translation.x, ts.transform.translation.y, ts.transform.translation.z = tx, ty, tz
        ts.transform.rotation.x, ts.transform.rotation.y, ts.transform.rotation.z, ts.transform.rotation.w = qx, qy, qz, qw
        msg.transforms.append(ts)
    return msg

# 关节状态消息转换
def _to_joint_state_msg(d):
    msg = JointState()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    msg.name = list(d["names"])
    msg.position = list(d["position"])
    msg.velocity = list(d["velocity"])
    msg.effort = list(d["effort"])
    return msg

# 手柄消息转换
def _to_joy_msg(d):
    msg = Joy()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    msg.axes = [float(x) for x in d["axes"]]
    msg.buttons = [int(x) for x in d["buttons"]]
    return msg


# IMU 消息转换（sensor_msgs/Imu）
def _to_imu_msg(d):
    msg = Imu()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    msg.header.frame_id = d["frame_id"]
    o = d["orientation"]
    msg.orientation.x = float(o["x"])
    msg.orientation.y = float(o["y"])
    msg.orientation.z = float(o["z"])
    msg.orientation.w = float(o["w"])
    msg.orientation_covariance = list(d["orientation_covariance"])
    av = d["angular_velocity"]
    msg.angular_velocity.x = float(av["x"])
    msg.angular_velocity.y = float(av["y"])
    msg.angular_velocity.z = float(av["z"])
    msg.angular_velocity_covariance = list(d["angular_velocity_covariance"])
    la = d["linear_acceleration"]
    msg.linear_acceleration.x = float(la["x"])
    msg.linear_acceleration.y = float(la["y"])
    msg.linear_acceleration.z = float(la["z"])
    msg.linear_acceleration_covariance = list(d["linear_acceleration_covariance"])
    return msg


# 姿态数组消息转换
def _to_pose_array_msg(d):
    msg = PoseArray()
    msg.header.stamp = _ns_to_time(d["stamp_ns"])
    for p in d["poses"]:
        pose = Pose()
        px, py, pz = p["position"][:3]
        qx, qy, qz, qw = p["orientation"][:4]
        pose.position.x, pose.position.y, pose.position.z = px, py, pz
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        msg.poses.append(pose)
    return msg


# 浮点数数组消息转换
def _to_float64_array_msg(d):
    msg = Float64MultiArray()
    msg.data = [float(x) for x in d["data"]]
    return msg

# 根据msg_type转换成ros消息
def _to_ros_msg(msg_type,d):
    if msg_type == "TFMessage":
        return _to_tf_msg(d)
    elif msg_type == "JointState":
        return _to_joint_state_msg(d)
    elif msg_type == "Joy":
        return _to_joy_msg(d)
    elif msg_type == "Imu":
        return _to_imu_msg(d)
    elif msg_type == "PoseArray":
        return _to_pose_array_msg(d)
    elif msg_type == "Float64MultiArray":
        return _to_float64_array_msg(d)
    return None


# zenoh转Ros
class Zenoh2RosBridge(Node):
    def __init__(self):
        super().__init__("zenoh2ros_bridge")

        self._pending: Queue[tuple[Publisher, Any]] = Queue(maxsize=128) # 缓存队列
        self.create_timer(0.001, self._flush) # 定时器，每0.001s刷新一次缓存队列

        self._subs = []
        self._zenoh_pubs = []
        self._session = zenoh.open(zenoh_cf)
        # Zenoh 订阅 -> ROS 发布
        for zenoh_key, ros_topic, msg_type in create_topics_list():
            ros_cls = ROS_MSG_TYPES[msg_type]
            pub = self.create_publisher(ros_cls, ros_topic, 10)
            sub = self._session.declare_subscriber(zenoh_key, self._make_cb(pub, msg_type))
            self._subs.append(sub)
            self.get_logger().info(f"{zenoh_key} ({msg_type}) -> {ros_topic}")

        # ROS 订阅 -> Zenoh 发布
        for zenoh_key, ros_topic, msg_type in ROS2ZENOH_TOPICS:
            ros_cls = ROS_MSG_TYPES[msg_type]
            zpub = self._session.declare_publisher(zenoh_key)
            self._zenoh_pubs.append(zpub)
            self.create_subscription(
                ros_cls,
                ros_topic,
                lambda msg, zp=zpub: zp.put(encode_float64_multi_array(list(msg.data))),
                10,
            )
            self.get_logger().info(f"{ros_topic} ({msg_type}) -> {zenoh_key}")

    def close(self):
        for sub in self._subs:
            try:
                sub.undeclare()
            except Exception:
                pass
        self._subs.clear()
        for zpub in self._zenoh_pubs:
            try:
                zpub.undeclare()
            except Exception:
                pass
        self._zenoh_pubs.clear()
        if self._session is not None:
            try:
                self._session.close()
            except Exception:
                pass
            self._session = None

    # 创建zenoh订阅回调函数
    def _make_cb(self, pub, msg_type):
        # zenoh订阅回调函数
        def on_sample(sample):
            try:
                d = proto_to_dict(msg_type, sample.payload.to_bytes())
                ros_msg = _to_ros_msg(msg_type, d)
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
        return on_sample

    # 刷新缓存队列
    def _flush(self):
        while True:
            try:
                pub, msg = self._pending.get_nowait()
            except Empty:
                break
            pub.publish(msg) # 发布消息


def main():

    global _running
    _running = True
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    node = None
    try:
        rclpy.init()
        node = Zenoh2RosBridge()
        while _running and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"error: {e}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        # os._exit(0)


if __name__ == "__main__":
    main()
