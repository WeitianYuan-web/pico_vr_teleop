"""Pure-Python fallback for io_bus_codec (Cython .so is Python 3.10 only).

Used by zenoh2ros_bridge under ROS Jazzy / Python 3.12. Dict shapes match the
Cython module so callers do not need changes.
"""
from __future__ import annotations

from typing import Any

from io_msgs import messages_pb2


def encode_float64_multi_array(data: list[float]) -> bytes:
    msg = messages_pb2.Float64MultiArray()
    msg.data.extend(float(x) for x in data)
    return msg.SerializeToString()


def decode_float64_multi_array(payload: bytes) -> dict[str, Any]:
    msg = messages_pb2.Float64MultiArray()
    msg.ParseFromString(payload)
    return {"data": list(msg.data)}


def decode_joint_state(payload: bytes) -> dict[str, Any]:
    msg = messages_pb2.JointState()
    msg.ParseFromString(payload)
    return {
        "stamp_ns": int(msg.stamp_ns),
        "names": list(msg.names),
        "position": list(msg.position),
        "velocity": list(msg.velocity),
        "effort": list(msg.effort),
    }


def decode_tf_message(payload: bytes) -> dict[str, Any]:
    msg = messages_pb2.TFMessage()
    msg.ParseFromString(payload)
    return {
        "stamp_ns": int(msg.stamp_ns),
        "transforms": [
            {
                "parent": t.parent,
                "child": t.child,
                "translation": list(t.translation),
                "rotation": list(t.rotation),
            }
            for t in msg.transforms
        ],
    }


def decode_joy(payload: bytes) -> dict[str, Any]:
    msg = messages_pb2.Joy()
    msg.ParseFromString(payload)
    return {
        "stamp_ns": int(msg.stamp_ns),
        "axes": list(msg.axes),
        "buttons": list(msg.buttons),
    }


def decode_pose_array(payload: bytes) -> dict[str, Any]:
    msg = messages_pb2.PoseArray()
    msg.ParseFromString(payload)
    return {
        "stamp_ns": int(msg.stamp_ns),
        "poses": [
            {
                "position": list(p.position),
                "orientation": list(p.orientation),
            }
            for p in msg.poses
        ],
    }


def decode_imu(payload: bytes) -> dict[str, Any]:
    msg = messages_pb2.Imu()
    msg.ParseFromString(payload)
    return {
        "stamp_ns": int(msg.stamp_ns),
        "frame_id": msg.frame_id,
        "orientation": {
            "x": float(msg.orientation_x),
            "y": float(msg.orientation_y),
            "z": float(msg.orientation_z),
            "w": float(msg.orientation_w),
        },
        "orientation_covariance": list(msg.orientation_covariance),
        "angular_velocity": {
            "x": float(msg.angular_velocity_x),
            "y": float(msg.angular_velocity_y),
            "z": float(msg.angular_velocity_z),
        },
        "angular_velocity_covariance": list(msg.angular_velocity_covariance),
        "linear_acceleration": {
            "x": float(msg.linear_acceleration_x),
            "y": float(msg.linear_acceleration_y),
            "z": float(msg.linear_acceleration_z),
        },
        "linear_acceleration_covariance": list(msg.linear_acceleration_covariance),
    }


_DECODERS = {
    "TFMessage": decode_tf_message,
    "JointState": decode_joint_state,
    "Joy": decode_joy,
    "Imu": decode_imu,
    "PoseArray": decode_pose_array,
    "Float64MultiArray": decode_float64_multi_array,
}


def proto_to_dict(msg_type: str, payload: bytes) -> dict[str, Any]:
    decoder = _DECODERS.get(msg_type)
    if decoder is None:
        raise ValueError(f"unsupported msg_type: {msg_type}")
    return decoder(payload)
