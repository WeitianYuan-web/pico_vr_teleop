#!/usr/bin/env python3
"""Zenoh 调试工具：扫描活跃 key、统计频率、解码打印 payload。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from typing import Any


STATIC_KEYS = (
    "io_fusion/tf_exoskeleton",
    "io_esk/joint_data",
    "io_esk/joystick_data",
    "io_esk/imu_data_right",
    "io_esk/imu_data_left",
    "io_esk/vibration_feedback",
    "io_align/<hand>/tf_hand",
    "io_align/<hand>/poses_<frame>",
    "io_teleop/joint_cmd_finger_left",
    "io_teleop/joint_cmd_finger_right",
)


def _project_root() -> str:
    root = os.environ.get("IO_EXOTRANS2HAND_ROOT", "").strip()
    if root:
        return root
    return os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))


def _ensure_src_on_path() -> None:
    src = os.path.join(_project_root(), "src")
    if src not in sys.path:
        sys.path.insert(0, src)


def _load_zenoh_config():
    cfg_path = os.environ.get("ZENOH_CONFIG", "").strip()
    if not cfg_path:
        cfg_path = os.path.join(_project_root(), "configs/config/zenoh.json5")

    import zenoh

    if cfg_path and os.path.isfile(cfg_path):
        print(f"Zenoh 配置: {cfg_path}")
        return zenoh.Config.from_file(cfg_path)

    if cfg_path:
        print(f"警告: Zenoh 配置文件不存在，使用默认配置: {cfg_path}", file=sys.stderr)

    return zenoh.Config()


def _infer_msg_type(key: str) -> str | None:
    """按 key 路径推断 Protobuf 消息类型（与 gateway.yaml 约定一致）。"""
    k = key.strip("/")
    last = k.rsplit("/", 1)[-1]
    if k.endswith("tf_exoskeleton") or last == "tf_hand":
        return "TFMessage"
    if "joint_data" in k or "joint_cmd" in k:
        return "JointState"
    if "joystick" in k:
        return "Joy"
    if "vibration_feedback" in k:
        return "Float64MultiArray"
    if last.startswith("poses") or last.startswith("pose"):
        return "PoseArray"
    return None


def _decode_payload(key: str, payload: bytes) -> dict[str, Any] | None:
    msg_type = _infer_msg_type(key)
    if not msg_type:
        return None
    _ensure_src_on_path()
    from io_bus_proto.io_bus_codec import proto_to_dict

    return proto_to_dict(msg_type, payload)


def _format_payload(key: str, payload: bytes, raw: bool) -> str:
    if raw:
        return payload.hex()

    decoded = _decode_payload(key, payload)
    if decoded is not None:
        return json.dumps(decoded, ensure_ascii=False, indent=2)

    msg_type = _infer_msg_type(key)
    hint = f"（未知类型，{len(payload)} 字节）" if msg_type is None else "（解码失败）"
    return f"{payload.hex()}\n# raw hex{hint}"


def _open_zenoh():
    try:
        import zenoh

        return zenoh, _load_zenoh_config()
    except ImportError:
        print("需要: pip install eclipse-zenoh", file=sys.stderr)
        raise SystemExit(1)
    except Exception as exc:
        print(f"Zenoh 配置加载失败: {exc}", file=sys.stderr)
        raise SystemExit(1)


def cmd_list_keys() -> int:
    print("项目常用 key（与 configs/config/topics.yaml / gateway.yaml 一致）：")
    for k in STATIC_KEYS:
        print(f"  {k}")
    print("\n扫描当前活跃 key：")
    print("  python3 scripts/verify_exo_zenoh.py --scan")
    return 0


def cmd_scan(zenoh, conf, expr: str, duration: float) -> int:
    print(f"扫描 key 表达式: {expr!r}，监听 {duration}s ...")
    stats: dict[str, dict[str, Any]] = {}
    t0 = time.time()
    deadline = t0 + duration

    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(expr)

        for sample in sub:
            key = str(sample.key_expr)
            now = time.time()
            row = stats.setdefault(
                key,
                {"count": 0, "first": now, "last": now, "last_len": 0},
            )
            row["count"] += 1
            row["last"] = now
            row["last_len"] = len(sample.payload.to_bytes())
            if now >= deadline:
                break

    elapsed = time.time() - t0
    if not stats:
        print("未发现活跃 key（无发布者或表达式不匹配）")
        return 2

    print(f"\n{'key':<48} {'count':>6}  {'avg_hz':>8}  {'last_len':>8}  type")
    print("-" * 90)
    for key in sorted(stats):
        row = stats[key]
        window = max(row["last"] - row["first"], 1e-6)
        hz = row["count"] / window
        msg_type = _infer_msg_type(key) or "-"
        print(
            f"{key:<48} {row['count']:>6}  {hz:>8.1f}  {row['last_len']:>8}  {msg_type}"
        )
    print(f"\n共 {len(stats)} 个活跃 key，监听 {elapsed:.1f}s")
    return 0


def cmd_dump(
    zenoh,
    conf,
    key: str,
    count: int,
    duration: float,
    raw: bool,
) -> int:
    print(f"订阅 key: {key}，最多 {count} 条，超时 {duration}s ...")
    msg_type = _infer_msg_type(key)
    if msg_type and not raw:
        print(f"推断消息类型: {msg_type}")
    elif not raw:
        print("警告: 无法推断消息类型，将输出原始 hex", file=sys.stderr)

    received = 0
    deadline = time.time() + duration

    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            payload = sample.payload.to_bytes()
            received += 1
            ts = time.strftime("%H:%M:%S")
            print(f"\n--- [{ts}] #{received} key={sample.key_expr} len={len(payload)} ---")
            print(_format_payload(key, payload, raw))
            if received >= count or time.time() >= deadline:
                break

    if received == 0:
        print("未收到数据")
        return 2
    return 0


def cmd_hz(zenoh, conf, key: str, duration: float) -> int:
    print(f"订阅 key: {key}，统计 {duration}s ...")
    count = 0
    count_at_last_print = 0
    sec_index = 0
    first_sample_time: float | None = None
    last_print: float | None = None
    last_len = 0
    instant_samples: list[float] = []

    t_open = time.time()
    deadline = t_open + duration

    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            count += 1
            last_len = len(sample.payload.to_bytes())
            now = time.time()

            if first_sample_time is None:
                first_sample_time = now
                last_print = now
                setup_ms = (first_sample_time - t_open) * 1000.0
                print(f"  首包延迟: {setup_ms:.0f} ms（Session 建立 → 首条 sample）")
                continue

            if now - last_print >= 1.0:
                sec_index += 1
                window = now - last_print
                instant_hz = (count - count_at_last_print) / window
                instant_samples.append(instant_hz)
                data_elapsed = now - first_sample_time
                cumulative_hz = count / data_elapsed if data_elapsed > 0 else 0.0
                print(
                    f"  sec {sec_index:2d}  instant={instant_hz:5.1f} Hz"
                    f"  cumulative={cumulative_hz:5.1f} Hz"
                    f"  total={count}  last_len={last_len}"
                )
                count_at_last_print = count
                last_print = now

            if now >= deadline:
                break

    elapsed = time.time() - t_open
    if count == 0:
        print("完成: 未收到任何数据")
        return 2

    data_elapsed = time.time() - first_sample_time if first_sample_time else elapsed
    overall_hz = count / data_elapsed if data_elapsed > 0 else 0.0
    print(f"完成: {count} 条 / {elapsed:.1f}s（收包 {data_elapsed:.1f}s）≈ {overall_hz:.1f} Hz")

    if len(instant_samples) >= 2:
        steady = instant_samples[1:]
        steady_avg = sum(steady) / len(steady)
        print(
            f"  稳态瞬时平均(跳过首秒): {steady_avg:.1f} Hz"
            f"（{len(steady)} 个窗口）"
        )
    elif len(instant_samples) == 1:
        print(f"  仅 1 个完整窗口，instant={instant_samples[0]:.1f} Hz")

    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Zenoh 调试：扫描活跃 key / 统计频率 / 解码打印数据",
    )
    parser.add_argument(
        "key",
        nargs="?",
        default="io_esk/joint_data",
        help="Zenoh key（--dump / 默认频率模式）",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="统计或扫描秒数")
    parser.add_argument(
        "--list-keys",
        action="store_true",
        help="打印项目常用 key（静态列表）",
    )
    parser.add_argument(
        "--scan",
        action="store_true",
        help="扫描当前网络上有数据的 key（默认表达式 **）",
    )
    parser.add_argument(
        "--expr",
        default="**",
        help="--scan 使用的 key 表达式，如 io_esk/**",
    )
    parser.add_argument(
        "--dump",
        action="store_true",
        help="解码并打印指定 key 的 payload（Protobuf → JSON）",
    )
    parser.add_argument(
        "--count",
        type=int,
        default=1,
        help="--dump 时最多打印几条消息",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="--dump 时输出原始 hex，不解码",
    )
    args = parser.parse_args()

    if args.list_keys:
        return cmd_list_keys()

    zenoh, conf = _open_zenoh()

    if args.scan:
        return cmd_scan(zenoh, conf, args.expr, args.duration)

    if args.dump:
        return cmd_dump(zenoh, conf, args.key, args.count, args.duration, args.raw)

    return cmd_hz(zenoh, conf, args.key, args.duration)


if __name__ == "__main__":
    raise SystemExit(main())
