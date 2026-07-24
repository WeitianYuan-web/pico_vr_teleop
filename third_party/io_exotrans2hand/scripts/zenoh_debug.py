#!/usr/bin/env python3
"""Zenoh 数据验证调试工具：列出可订阅 key、扫描活跃 key、持续输出数据或频率。"""

from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:
    yaml = None  # type: ignore[assignment]

_RUNNING = True


def _project_root() -> Path:
    env = os.environ.get("IO_EXOTRANS2HAND_ROOT", "").strip()
    if env:
        return Path(env).resolve()
    return Path(__file__).resolve().parent.parent


def _ensure_src_on_path() -> None:
    src = str(_project_root() / "src")
    if src not in sys.path:
        sys.path.insert(0, src)


def _load_zenoh_config():
    import zenoh

    cfg_path = os.environ.get("ZENOH_CONFIG", "").strip()
    if not cfg_path:
        cfg_path = str(_project_root() / "configs/config/zenoh.json5")
    if cfg_path and os.path.isfile(cfg_path):
        print(f"Zenoh 配置: {cfg_path}")
        return zenoh.Config.from_file(cfg_path)
    if cfg_path:
        print(f"警告: 配置文件不存在，使用默认: {cfg_path}", file=sys.stderr)
    return zenoh.Config()


def _open_zenoh():
    try:
        import zenoh

        return zenoh, _load_zenoh_config()
    except ImportError:
        print("需要: pip install eclipse-zenoh", file=sys.stderr)
        raise SystemExit(1)


def _on_signal(_sig, _frame) -> None:
    global _RUNNING
    _RUNNING = False


def _infer_msg_type(key: str) -> str | None:
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

    try:
        return proto_to_dict(msg_type, payload)
    except Exception:
        return None


def _format_payload(key: str, payload: bytes, raw: bool) -> str:
    if raw:
        return payload.hex()
    decoded = _decode_payload(key, payload)
    if decoded is not None:
        return json.dumps(decoded, ensure_ascii=False, indent=2)
    hint = "未知类型" if _infer_msg_type(key) is None else "解码失败"
    return f"{payload.hex()}\n# raw hex ({hint}, {len(payload)} B)"


def _namespaced_topic(hand: str, topic: str) -> str:
    ros_topic = topic if topic.startswith("/") else f"/{topic}"
    slash = ros_topic.find("/", 1)
    if slash < 0:
        return f"/{hand}{ros_topic}"
    return f"{ros_topic[:slash]}/{hand}{ros_topic[slash:]}"


def _ros_to_zenoh_key(path: str) -> str:
    return path[1:] if path.startswith("/") else path


def _load_expected_keys(hands: list[str]) -> list[dict[str, str]]:
    """从 gateway.yaml 展开可订阅 key（与 WebSocket streams 一致）。"""
    gw_path = _project_root() / "configs/config/gateway.yaml"
    if not gw_path.is_file() or yaml is None:
        return _fallback_expected_keys(hands)

    data = yaml.safe_load(gw_path.read_text(encoding="utf-8")) or {}
    rows: list[dict[str, str]] = []
    for item in data.get("streams") or []:
        if not isinstance(item, dict):
            continue
        stream_id = str(item.get("id") or "")
        topic = str(item.get("topic") or "")
        msg_type = str(item.get("type") or "")
        scope = str(item.get("scope") or "global")
        if scope == "hand":
            for hand in hands:
                key = _ros_to_zenoh_key(_namespaced_topic(hand, topic))
                rows.append(
                    {
                        "id": f"{stream_id}.{hand}",
                        "key": key,
                        "type": msg_type,
                        "scope": scope,
                        "hand": hand,
                    }
                )
        else:
            rows.append(
                {
                    "id": stream_id,
                    "key": _ros_to_zenoh_key(topic),
                    "type": msg_type,
                    "scope": scope,
                    "hand": "",
                }
            )

    for item in data.get("publish_streams") or []:
        if not isinstance(item, dict):
            continue
        topic = str(item.get("topic") or "")
        rows.append(
            {
                "id": str(item.get("id") or ""),
                "key": _ros_to_zenoh_key(topic),
                "type": str(item.get("type") or ""),
                "scope": "publish",
                "hand": "",
            }
        )
    return rows


def _fallback_expected_keys(hands: list[str]) -> list[dict[str, str]]:
    rows = [
        {"id": "io_esk.tf", "key": "io_fusion/tf_exoskeleton", "type": "TFMessage", "scope": "global", "hand": ""},
        {"id": "io_esk.joint_data", "key": "io_esk/joint_data", "type": "JointState", "scope": "global", "hand": ""},
        {"id": "io_esk.joystick_data", "key": "io_esk/joystick_data", "type": "Joy", "scope": "global", "hand": ""},
        {"id": "io_esk.imu_data_right", "key": "io_esk/imu_data_right", "type": "Imu", "scope": "global", "hand": ""},
        {"id": "io_esk.imu_data_left", "key": "io_esk/imu_data_left", "type": "Imu", "scope": "global", "hand": ""},
    ]
    for hand in hands:
        prefix = f"io_align/{hand}"
        rows.extend(
            [
                {"id": f"io_align.tf.{hand}", "key": f"{prefix}/tf_hand", "type": "TFMessage", "scope": "hand", "hand": hand},
                {"id": f"io_align.poses_left.{hand}", "key": f"{prefix}/poses_left_hand_ee_link", "type": "PoseArray", "scope": "hand", "hand": hand},
                {"id": f"io_align.poses_right.{hand}", "key": f"{prefix}/poses_right_hand_ee_link", "type": "PoseArray", "scope": "hand", "hand": hand},
                {"id": f"io_teleop.joint_cmd_left.{hand}", "key": f"io_teleop/{hand}/joint_cmd_finger_left", "type": "JointState", "scope": "hand", "hand": hand},
                {"id": f"io_teleop.joint_cmd_right.{hand}", "key": f"io_teleop/{hand}/joint_cmd_finger_right", "type": "JointState", "scope": "hand", "hand": hand},
            ]
        )
    return rows


def cmd_keys(args: argparse.Namespace) -> int:
    hands = [h.strip() for h in args.hand if h.strip()]
    rows = _load_expected_keys(hands)
    print(f"{'stream_id':<36} {'zenoh_key':<46} {'type':<18} scope")
    print("-" * 110)
    for row in rows:
        print(
            f"{row['id']:<36} {row['key']:<46} {row['type']:<18} {row['scope']}"
        )
    print(f"\n共 {len(rows)} 个 key（配置预期；实际活跃 key 请用 scan）")
    print("扫描活跃 key:  python3 scripts/zenoh_debug.py scan")
    return 0


def cmd_scan(args: argparse.Namespace) -> int:
    zenoh, conf = _open_zenoh()
    expr = args.expr
    duration = args.duration
    print(f"扫描表达式: {expr!r}，监听 {duration:.1f}s ...")

    stats: dict[str, dict[str, Any]] = {}
    t0 = time.time()
    deadline = t0 + duration

    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(expr)
        for sample in sub:
            if not _RUNNING:
                break
            key = str(sample.key_expr)
            now = time.time()
            row = stats.setdefault(key, {"count": 0, "first": now, "last": now, "last_len": 0})
            row["count"] += 1
            row["last"] = now
            row["last_len"] = len(sample.payload.to_bytes())
            if now >= deadline:
                break

    if not stats:
        print("未发现活跃 key")
        return 2

    print(f"\n{'zenoh_key':<52} {'count':>6}  {'avg_hz':>8}  {'last_len':>8}  type")
    print("-" * 92)
    for key in sorted(stats):
        row = stats[key]
        window = max(row["last"] - row["first"], 1e-6)
        hz = row["count"] / window
        print(
            f"{key:<52} {row['count']:>6}  {hz:>8.1f}  {row['last_len']:>8}  "
            f"{_infer_msg_type(key) or '-'}"
        )
    print(f"\n共 {len(stats)} 个活跃 key")
    return 0


def cmd_watch(args: argparse.Namespace) -> int:
    zenoh, conf = _open_zenoh()
    key = args.key
    limit = args.count
    raw = args.raw
    msg_type = _infer_msg_type(key)
    print(f"持续订阅: {key}")
    if msg_type and not raw:
        print(f"消息类型: {msg_type}")
    if limit == 0:
        print("模式: 持续输出（Ctrl+C 停止）")
    else:
        print(f"模式: 最多 {limit} 条")

    received = 0
    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            if not _RUNNING:
                break
            payload = sample.payload.to_bytes()
            received += 1
            ts = time.strftime("%H:%M:%S.") + f"{int(time.time() * 1000) % 1000:03d}"
            print(f"\n--- [{ts}] #{received} len={len(payload)} ---")
            print(_format_payload(key, payload, raw))
            sys.stdout.flush()
            if limit > 0 and received >= limit:
                break

    if received == 0:
        print("未收到数据")
        return 2
    return 0


def cmd_hz(args: argparse.Namespace) -> int:
    zenoh, conf = _open_zenoh()
    key = args.key
    duration = args.duration
    forever = duration <= 0

    print(f"频率统计: {key}")
    if forever:
        print("模式: 持续统计（Ctrl+C 停止）")
    else:
        print(f"模式: 统计 {duration:.1f}s")

    count = 0
    sec_count = 0
    sec_index = 0
    first_ts: float | None = None
    last_print: float | None = None
    last_len = 0
    t_open = time.time()
    deadline = t_open + duration if not forever else float("inf")

    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            if not _RUNNING:
                break
            count += 1
            last_len = len(sample.payload.to_bytes())
            now = time.time()
            if first_ts is None:
                first_ts = now
                last_print = now
                print(f"  首包延迟: {(first_ts - t_open) * 1000:.0f} ms")
                continue
            if now - last_print >= 1.0:
                sec_index += 1
                instant = (count - sec_count) / (now - last_print)
                cumulative = count / (now - first_ts)
                print(
                    f"  [{sec_index:3d}] instant={instant:6.1f} Hz  "
                    f"cumulative={cumulative:6.1f} Hz  total={count}  last_len={last_len}"
                )
                sec_count = count
                last_print = now
            if now >= deadline:
                break

    if count == 0:
        print("未收到任何数据")
        return 2

    if first_ts:
        overall = count / (time.time() - first_ts)
        print(f"汇总: {count} 条 ≈ {overall:.1f} Hz  last_len={last_len}")
    return 0


def cmd_monitor(args: argparse.Namespace) -> int:
    """同时输出频率窗口 + 周期性数据抽样。"""
    zenoh, conf = _open_zenoh()
    key = args.key
    sample_every = args.sample_every
    raw = args.raw
    print(f"监控: {key}（每秒 Hz + 每 {sample_every}s 打印 1 条数据，Ctrl+C 停止）")

    count = 0
    sec_count = 0
    first_ts: float | None = None
    last_hz_print = 0.0
    last_sample_print = 0.0
    t_open = time.time()

    with zenoh.open(conf) as session:
        sub = session.declare_subscriber(key)
        for sample in sub:
            if not _RUNNING:
                break
            payload = sample.payload.to_bytes()
            count += 1
            now = time.time()
            if first_ts is None:
                first_ts = now
                last_hz_print = now
                last_sample_print = now
                print(f"  首包: delay={(first_ts - t_open) * 1000:.0f} ms  len={len(payload)}")
                if args.sample_first:
                    print(_format_payload(key, payload, raw))
                continue

            if now - last_hz_print >= 1.0:
                instant = (count - sec_count) / (now - last_hz_print)
                cumulative = count / (now - first_ts)
                print(
                    f"  [Hz] instant={instant:6.1f}  cumulative={cumulative:6.1f}  "
                    f"total={count}  last_len={len(payload)}"
                )
                sec_count = count
                last_hz_print = now

            if now - last_sample_print >= sample_every:
                ts = time.strftime("%H:%M:%S")
                print(f"\n--- [{ts}] sample #{count} len={len(payload)} ---")
                print(_format_payload(key, payload, raw))
                last_sample_print = now
                sys.stdout.flush()

    return 0


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Zenoh 调试：列出 key / 扫描活跃 key / 持续输出数据或频率",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    p_keys = sub.add_parser("keys", help="列出 gateway.yaml 中可订阅的 key")
    p_keys.add_argument(
        "--hand",
        action="append",
        default=["test_hand_config"],
        help="手型目录名，可多次指定（展开 hand-scoped 流）",
    )
    p_keys.set_defaults(func=cmd_keys)

    p_scan = sub.add_parser("scan", help="扫描当前网络活跃 key 及平均 Hz")
    p_scan.add_argument("--expr", default="**", help="Zenoh 表达式，如 io_esk/**")
    p_scan.add_argument("--duration", type=float, default=10.0, help="扫描秒数")
    p_scan.set_defaults(func=cmd_scan)

    p_watch = sub.add_parser("watch", help="持续输出指定 key 的数据（Protobuf→JSON）")
    p_watch.add_argument("key", help="Zenoh key，如 io_esk/joint_data")
    p_watch.add_argument(
        "--count",
        type=int,
        default=0,
        help="最多打印条数，0=持续直到 Ctrl+C",
    )
    p_watch.add_argument("--raw", action="store_true", help="输出 hex，不解码")
    p_watch.set_defaults(func=cmd_watch)

    p_hz = sub.add_parser("hz", help="持续统计指定 key 的频率")
    p_hz.add_argument("key", help="Zenoh key")
    p_hz.add_argument(
        "--duration",
        type=float,
        default=0,
        help="统计秒数，0=持续直到 Ctrl+C",
    )
    p_hz.set_defaults(func=cmd_hz)

    p_mon = sub.add_parser("monitor", help="Hz + 周期性数据抽样（综合验证）")
    p_mon.add_argument("key", help="Zenoh key")
    p_mon.add_argument(
        "--sample-every",
        type=float,
        default=5.0,
        help="每隔多少秒打印一条解码数据",
    )
    p_mon.add_argument(
        "--sample-first",
        action="store_true",
        help="首包也打印解码内容",
    )
    p_mon.add_argument("--raw", action="store_true", help="抽样输出 hex")
    p_mon.set_defaults(func=cmd_monitor)

    return parser


def main() -> int:
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    parser = _build_parser()
    args = parser.parse_args()
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
