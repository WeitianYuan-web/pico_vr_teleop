#!/usr/bin/env python3
"""Piper 键盘末端位姿增量遥操作（Placo 多任务加权 QP IK -> move_j）。"""

from __future__ import annotations

import argparse
import math
import select
import sys
import tempfile
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from pyAgxArm import AgxArmFactory, create_agx_arm_config
from piper_placo_qp_ik import PiperPlacoConfig, PiperPlacoQPIK, transform_to_pose6
from run_piper_control import detect_firmware_version, resolve_can_backend, wait_motion_done, wait_robot_comm_ready

try:
    import pinocchio as pin
    from pinocchio.visualize import MeshcatVisualizer
except Exception:
    pin = None
    MeshcatVisualizer = None


HELP_TEXT = """
键盘末端位姿增量控制（Placo QP IK -> move_j）
  平移: W/S=X±  A/D=Y±  Q/E=Z±
  姿态: U/O=Roll±  I/K=Pitch±  J/L=Yaw±
  [ ]    减小/增大平移步长
  ; '    减小/增大旋转步长
  b      回到初始位姿（启动位姿）
  z      重置目标位姿到当前 FK
  h      显示帮助
  c      退出
"""


@dataclass
class TeleopStep:
    position_m: float = 0.005
    rotation_rad: float = math.radians(2.0)


def _wrap_angle(rad: float) -> float:
    return (rad + math.pi) % (2.0 * math.pi) - math.pi


def _default_urdf_path() -> Path:
    return Path(__file__).resolve().parent / "agx_arm_urdf-main/piper_h/urdf/piper_h_description.urdf"


def _fmt_pose(pose6):
    xyz = ", ".join(f"{v:.4f}" for v in pose6[:3])
    rpy = ", ".join(f"{math.degrees(v):.2f}" for v in pose6[3:])
    return f"xyz(m)=[{xyz}]  rpy(deg)=[{rpy}]"


def _apply_delta(pose6, axis: str, step: TeleopStep):
    p = list(pose6)
    delta_map = {
        "+x": (0, step.position_m), "-x": (0, -step.position_m),
        "+y": (1, step.position_m), "-y": (1, -step.position_m),
        "+z": (2, step.position_m), "-z": (2, -step.position_m),
        "+roll": (3, step.rotation_rad), "-roll": (3, -step.rotation_rad),
        "+pitch": (4, step.rotation_rad), "-pitch": (4, -step.rotation_rad),
        "+yaw": (5, step.rotation_rad), "-yaw": (5, -step.rotation_rad),
    }
    if axis in delta_map:
        idx, d = delta_map[axis]
        p[idx] += d
        if idx >= 3:
            p[idx] = _wrap_angle(p[idx])
    return p


def _parse_init_joints(joints_text: str):
    vals = [float(x.strip()) for x in joints_text.split(",") if x.strip()]
    if len(vals) != 6:
        raise ValueError("--init-joints 必须是 6 个逗号分隔浮点数，例如: 0,0.35,-0.35,0,0,0")
    return vals


class RawKeyboard:
    def __init__(self):
        self._fd = sys.stdin.fileno()
        self._old = termios.tcgetattr(self._fd)

    def __enter__(self):
        tty.setcbreak(self._fd)
        return self

    def __exit__(self, exc_type, exc, tb):
        termios.tcsetattr(self._fd, termios.TCSADRAIN, self._old)

    def read_key(self):
        if not select.select([sys.stdin], [], [], 0.0)[0]:
            return None
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch
        if not select.select([sys.stdin], [], [], 0.01)[0]:
            return ch
        return ch + sys.stdin.read(2)


class DualRobotViewer:
    def __init__(self, urdf_path: Path):
        if pin is None or MeshcatVisualizer is None:
            raise RuntimeError("未检测到 pinocchio/meshcat，可视化不可用。")
        self._tmp_urdf = self._prepare_urdf_for_visual(urdf_path)
        model_a, coll_a, vis_a = pin.buildModelsFromUrdf(str(self._tmp_urdf))
        model_t, coll_t, vis_t = pin.buildModelsFromUrdf(str(self._tmp_urdf))
        self.viz_actual = MeshcatVisualizer(model_a, coll_a, vis_a)
        self.viz_actual.initViewer(open=True)
        self.viz_actual.loadViewerModel(rootNodeName="actual")
        shared_viewer = getattr(self.viz_actual, "viewer", None)
        if shared_viewer is None:
            raise RuntimeError("MeshcatVisualizer 初始化异常：未获取到 viewer 实例")
        self.viz_target = MeshcatVisualizer(model_t, coll_t, vis_t)
        self.viz_target.initViewer(viewer=shared_viewer, open=False)
        self.viz_target.loadViewerModel(rootNodeName="target")
        self._set_target_tint()

    @staticmethod
    def _prepare_urdf_for_visual(urdf_path: Path) -> Path:
        urdf_path = Path(urdf_path).resolve()
        text = urdf_path.read_text(encoding="utf-8")
        package_prefix = "package://agx_arm_description/agx_arm_urdf/"
        if package_prefix in text:
            asset_root = urdf_path.parents[2]
            replacement = asset_root.as_posix().rstrip("/") + "/"
            text = text.replace(package_prefix, replacement)
        tmp = tempfile.NamedTemporaryFile(prefix="piper_meshcat_", suffix=".urdf", delete=False)
        tmp_path = Path(tmp.name)
        tmp.close()
        tmp_path.write_text(text, encoding="utf-8")
        return tmp_path

    def _set_target_tint(self):
        try:
            root = self.viz_target.viewer["target"]
            root.set_property("opacity", 0.45)
        except Exception:
            pass

    def update(self, q_actual: np.ndarray, q_target: np.ndarray):
        self.viz_actual.display(np.asarray(q_actual, dtype=float))
        self.viz_target.display(np.asarray(q_target, dtype=float))


def _connect_robot(can_port: str, robot_model: str):
    interface, _ = resolve_can_backend()
    probe_cfg = create_agx_arm_config(robot=robot_model, interface=interface, channel=can_port)
    probe = AgxArmFactory.create_arm(probe_cfg)
    probe.connect()
    robot_cfg, fw_ver = detect_firmware_version(probe, can_port, interface, robot_model)
    robot = AgxArmFactory.create_arm(robot_cfg)
    robot.connect()
    return robot, fw_ver


def main():
    parser = argparse.ArgumentParser(description="Piper 键盘末端位姿增量遥操作（Placo QP IK）")
    parser.add_argument("--robot", default="piper_h")
    parser.add_argument("--can_port", default=None)
    parser.add_argument("--urdf", default=None)
    parser.add_argument("--speed", type=int, default=30)
    parser.add_argument("--pos-step", type=float, default=0.005)
    parser.add_argument("--rot-step-deg", type=float, default=2.0, help="每次按键姿态步长(度)")
    parser.add_argument("--init-x", type=float, default=None, help="启动时末端绝对 X(m)")
    parser.add_argument("--init-y", type=float, default=None, help="启动时末端绝对 Y(m)")
    parser.add_argument("--init-z", type=float, default=None, help="启动时末端绝对 Z(m)")
    parser.add_argument("--wait-motion", action="store_true")
    parser.add_argument("--qp-dt", type=float, default=0.02)
    parser.add_argument("--max-joint-step", type=float, default=0.08)
    parser.add_argument(
        "--init-joints",
        type=str,
        default="0,0.35,-0.35,0,0,0",
        help="启动时先执行的绝对关节角(rad)，格式: j1,j2,j3,j4,j5,j6",
    )
    parser.add_argument("--init-joint-timeout", type=float, default=6.0, help="初始关节姿态等待到位超时(s)")
    parser.add_argument("--init-wait", type=float, default=2.0, help="启动初始姿态等待秒数")
    parser.add_argument("--disable-on-exit", action="store_true", help="退出时执行 disable（默认仅断开不失能）")
    parser.add_argument("--viewer", action="store_true", help="打开 MeshCat 可视化，显示当前反馈姿态和目标姿态")
    args = parser.parse_args()

    _, default_port = resolve_can_backend()
    can_port = args.can_port or default_port
    urdf_path = Path(args.urdf) if args.urdf else _default_urdf_path()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF 不存在: {urdf_path}")

    step = TeleopStep(position_m=args.pos_step, rotation_rad=math.radians(args.rot_step_deg))
    viewer = None
    if args.viewer:
        try:
            viewer = DualRobotViewer(urdf_path)
            print("MeshCat 可视化已启动：actual=当前反馈，target=目标姿态")
        except Exception as exc:
            print(f"可视化初始化失败，继续无可视化运行: {exc}")

    print(f"连接 Piper @ {can_port} ...")
    robot, fw = _connect_robot(can_port, args.robot)
    print(f"固件: {fw}")

    while not robot.enable():
        time.sleep(0.01)

    if not wait_robot_comm_ready(robot, timeout=15.0):
        print("警告: 通信未在 15s 内稳定，仍尝试初始化")

    robot.set_speed_percent(max(1, min(args.speed, 100)))
    robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
    robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
    time.sleep(0.3)

    # 第一步：先以 move_j 到指定绝对关节角（手臂抬起）
    init_joints = _parse_init_joints(args.init_joints)
    print(f"初始关节姿态(move_j, absolute): {init_joints}")
    robot.move_j(init_joints)
    reached = wait_motion_done(robot, timeout=args.init_joint_timeout, target_joints=init_joints)
    if reached:
        print("初始关节姿态已到位")
    else:
        print("初始关节姿态等待超时，继续执行后续流程")

    qp = PiperPlacoQPIK(
        PiperPlacoConfig(
            urdf_path=urdf_path,
            ee_frame="link6",
            dt=args.qp_dt,
            position_weight=1.0,
            orientation_weight=0.1,
            manipulability_weight=1e-2,
            joints_regularization_weight=1e-4,
        )
    )

    joint = robot.get_joint_angles()
    if joint is None:
        raise RuntimeError("无法读取当前关节角。")
    q = np.array(list(joint.msg), dtype=float)
    qp.sync_state_from_joint_positions(q.tolist())
    current_pose = qp.current_pose6().tolist()

    # 启动初始姿态（全局绝对 XYZ）
    init_pose = current_pose.copy()
    if args.init_x is not None:
        init_pose[0] = args.init_x
    if args.init_y is not None:
        init_pose[1] = args.init_y
    if args.init_z is not None:
        init_pose[2] = args.init_z
    qp.set_target_pose6(init_pose)
    solved, q_sol = qp.solve()
    if solved:
        robot.move_j(q_sol.tolist())
        if viewer is not None:
            viewer.update(q, q_sol)
        if args.init_wait > 0:
            time.sleep(args.init_wait)
        joint = robot.get_joint_angles()
        if joint is not None:
            q = np.array(list(joint.msg), dtype=float)
            qp.sync_state_from_joint_positions(q.tolist())
            target_pose = qp.current_pose6().tolist()
        else:
            target_pose = init_pose
    else:
        print("[QP] 初始绝对位姿求解失败，保持当前姿态")
        target_pose = current_pose

    print("初始目标位姿:", _fmt_pose(target_pose))
    print(HELP_TEXT)

    key_to_axis = {
        "w": "+x", "s": "-x", "a": "+y", "d": "-y",
        "q": "+z", "e": "-z",
        "u": "+roll", "o": "-roll",
        "i": "+pitch", "k": "-pitch",
        "j": "+yaw", "l": "-yaw",
    }

    try:
        with RawKeyboard() as kb:
            while True:
                key = kb.read_key()
                if key is None:
                    time.sleep(0.01)
                    continue

                if key in ("c", "\x03"):
                    print("退出遥操作")
                    break
                if key == "h":
                    print(HELP_TEXT)
                    continue
                if key == "[":
                    step.position_m = max(0.001, step.position_m * 0.8)
                    print(f"平移步长 = {step.position_m:.4f} m")
                    continue
                if key == "]":
                    step.position_m = min(0.05, step.position_m * 1.25)
                    print(f"平移步长 = {step.position_m:.4f} m")
                    continue
                if key == ";":
                    step.rotation_rad = max(math.radians(0.2), step.rotation_rad * 0.8)
                    print(f"旋转步长 = {math.degrees(step.rotation_rad):.2f} deg")
                    continue
                if key == "'":
                    step.rotation_rad = min(math.radians(15.0), step.rotation_rad * 1.25)
                    print(f"旋转步长 = {math.degrees(step.rotation_rad):.2f} deg")
                    continue
                if key == "b":
                    print("回到初始位姿...")
                    robot.move_j(init_joints)
                    wait_motion_done(robot, timeout=args.init_joint_timeout)
                    joint = robot.get_joint_angles()
                    if joint is None:
                        print("读取关节角失败")
                        continue
                    q_cur = np.array(list(joint.msg), dtype=float)
                    qp.sync_state_from_joint_positions(q_cur.tolist())
                    qp.set_target_pose6(init_pose)
                    solved, q_sol = qp.solve()
                    if not solved:
                        print("[QP] 回初始位姿求解失败")
                        continue
                    robot.move_j(q_sol.tolist())
                    if viewer is not None:
                        viewer.update(q_cur, q_sol)
                    if args.wait_motion:
                        wait_motion_done(robot, timeout=3.0)
                    joint = robot.get_joint_angles()
                    if joint is not None:
                        q_now = np.array(list(joint.msg), dtype=float)
                        qp.sync_state_from_joint_positions(q_now.tolist())
                        target_pose = qp.current_pose6().tolist()
                    else:
                        target_pose = init_pose.copy()
                    print("已回到初始位姿:", _fmt_pose(target_pose))
                    continue
                if key == "z":
                    joint = robot.get_joint_angles()
                    if joint is None:
                        print("读取关节角失败")
                        continue
                    q = np.array(list(joint.msg), dtype=float)
                    qp.sync_state_from_joint_positions(q.tolist())
                    target_pose = qp.current_pose6().tolist()
                    print("已重置目标位姿:", _fmt_pose(target_pose))
                    continue
                if key not in key_to_axis:
                    continue

                candidate = _apply_delta(target_pose, key_to_axis[key], step)
                joint = robot.get_joint_angles()
                if joint is None:
                    print("读取关节角失败")
                    continue
                q_cur = np.array(list(joint.msg), dtype=float)
                qp.sync_state_from_joint_positions(q_cur.tolist())

                qp.set_target_pose6(candidate)
                solved, q_sol = qp.solve()
                if not solved:
                    print(f"[QP] 未收敛: {_fmt_pose(candidate)}")
                    continue

                dq = np.clip(q_sol - q_cur, -args.max_joint_step, args.max_joint_step)
                q_cmd = q_cur + dq
                robot.move_j(q_cmd.tolist())
                if viewer is not None:
                    viewer.update(q_cur, q_cmd)
                if args.wait_motion:
                    wait_motion_done(robot, timeout=3.0)

                target_pose = candidate
                print(f"  [{key_to_axis[key]:>6}] -> {_fmt_pose(target_pose)}")
    finally:
        if args.disable_on_exit:
            robot.disable()
        robot.disconnect()
        if args.disable_on_exit:
            print("已断开连接（已失能）")
        else:
            print("已断开连接（未失能）")


if __name__ == "__main__":
    main()
