#!/usr/bin/env python3
"""Robot-side UDP → /endposetarget_L|R bridge (run under Humble + XARM)."""

from __future__ import annotations

import argparse
import os
import socket
import sys
import time

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.abspath(os.path.join(_HERE, "../.."))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from config import (  # noqa: E402
    DEFAULT_FROM_FRAME,
    DEFAULT_TO_FRAME_LEFT,
    DEFAULT_TO_FRAME_RIGHT,
    DEFAULT_UDP_PORT,
)
from ros_endpose import TianyiRosEndpose  # noqa: E402
from udp_protocol import decode_pose_packet  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description="Tianyi UDP→ROS endpose bridge")
    p.add_argument("--udp-bind", default="0.0.0.0")
    p.add_argument("--udp-port", type=int, default=DEFAULT_UDP_PORT)
    p.add_argument("--from-frame", default=DEFAULT_FROM_FRAME)
    p.add_argument("--to-frame-left", default=DEFAULT_TO_FRAME_LEFT)
    p.add_argument("--to-frame-right", default=DEFAULT_TO_FRAME_RIGHT)
    p.add_argument("--prepare", action="store_true", help="enable arm + mode3 + auto_switch")
    p.add_argument("--no-disable-on-exit", action="store_true")
    args = p.parse_args()

    ros = TianyiRosEndpose(
        from_frame=args.from_frame,
        to_frame_left=args.to_frame_left,
        to_frame_right=args.to_frame_right,
        node_name="tianyee_udp_ros_bridge",
    )
    if args.prepare:
        print("[Bridge] prepare arm...")
        ros.prepare_for_teleop()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((args.udp_bind, int(args.udp_port)))
    sock.settimeout(0.05)
    print(f"[Bridge] listening udp://{args.udp_bind}:{args.udp_port} → /endposetarget_L|R")

    try:
        while True:
            ros.spin_once(0.0)
            try:
                data, addr = sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                pkt = decode_pose_packet(data)
            except Exception as exc:  # noqa: BLE001
                print(f"[Bridge] bad packet: {exc}")
                continue

            if pkt.get("cmd") == "get_tf":
                reply = {"t": time.time(), "left": None, "right": None}
                for side in ("left", "right"):
                    try:
                        pos, quat = ros.lookup_tcp(side)
                        reply[side] = {
                            "xyz": pos.tolist(),
                            "quat_wxyz": quat.tolist(),
                        }
                    except Exception as exc:  # noqa: BLE001
                        reply[side] = {"error": str(exc)}
                sock.sendto(
                    __import__("json").dumps(reply, separators=(",", ":")).encode("utf-8"),
                    addr,
                )
                continue

            for side in ("left", "right"):
                side_data = pkt.get(side)
                if not isinstance(side_data, dict) or not side_data.get("active"):
                    continue
                xyz = side_data.get("xyz")
                quat = side_data.get("quat_wxyz")
                if not xyz or not quat or len(xyz) != 3 or len(quat) != 4:
                    continue
                ros.publish_pose(side, np.asarray(xyz, dtype=float), np.asarray(quat, dtype=float))
    except KeyboardInterrupt:
        print("\n[Bridge] stop")
    finally:
        sock.close()
        if args.prepare and not args.no_disable_on_exit:
            try:
                ros.call_enable(False)
            except Exception as exc:  # noqa: BLE001
                print(f"[Bridge] disable failed: {exc}")
        ros.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
