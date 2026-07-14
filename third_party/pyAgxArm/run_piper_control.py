#!/usr/bin/env python3
"""Piper 机械臂基础控制：连接、使能、回零并读取关节角度。"""

import argparse
import sys
import time
from platform import system

from pyAgxArm import AgxArmFactory, PiperFW, create_agx_arm_config


def resolve_can_backend():
    """根据操作系统返回默认 CAN 接口类型与通道。"""
    platform_system = system()
    if platform_system == "Windows":
        return "agx_cando", "0"
    if platform_system == "Linux":
        return "socketcan", "can0"
    if platform_system == "Darwin":
        return "slcan", "/dev/ttyACM0"
    raise RuntimeError("仅支持 Linux socketcan、Windows agx_cando、macOS slcan。")


def wait_robot_comm_ready(robot, timeout: float = 15.0, poll_interval: float = 0.1) -> bool:
    """等待 CAN 通信稳定、关节角与状态可读（冷启动后 enable 完成不代表立即可控）。"""
    start_t = time.monotonic()
    while time.monotonic() - start_t < timeout:
        if robot.is_ok() and robot.get_fps() > 0:
            joints = robot.get_joint_angles()
            status = robot.get_arm_status()
            if joints is not None and status is not None:
                return True
        time.sleep(poll_interval)
    return False


def wait_motion_done(
    robot,
    timeout: float = 10.0,
    poll_interval: float = 0.1,
    target_joints=None,
    motion_start_grace: float = 1.5,
) -> bool:
    """
    等待机械臂运动完成。

    冷启动时 move_j 下发后控制器可能仍报告 motion_status==0（空闲），
    若不做「先观察到运动再判到位」会误判为已到达，导致首次启动不归位。
    """
    start_t = time.monotonic()
    initial_q = None
    joints_msg = robot.get_joint_angles()
    if joints_msg is not None:
        initial_q = list(joints_msg.msg)

    motion_seen = False
    deadline = start_t + timeout
    while time.monotonic() < deadline:
        elapsed = time.monotonic() - start_t
        joints_msg = robot.get_joint_angles()
        status = robot.get_arm_status()

        if joints_msg is not None and initial_q is not None:
            delta = max(abs(a - b) for a, b in zip(joints_msg.msg, initial_q))
            if delta > 0.02:
                motion_seen = True

        if joints_msg is not None and target_joints is not None:
            err = max(abs(a - b) for a, b in zip(joints_msg.msg, target_joints))
            if err < 0.05 and (motion_seen or elapsed >= motion_start_grace):
                if status is None or getattr(status.msg, "motion_status", None) == 0:
                    return True
        elif status is not None and getattr(status.msg, "motion_status", None) == 0:
            if motion_seen or elapsed >= motion_start_grace:
                return True

        time.sleep(poll_interval)
    return False


def detect_firmware_version(robot, can_port: str, interface: str, robot_model: str):
    """探测固件版本并返回匹配的 PiperFW 配置。"""
    startup_deadline = time.time() + 15.0
    while robot.get_firmware() is None:
        if time.time() >= startup_deadline:
            raise TimeoutError(f"在 {can_port} 上等待固件信息超时，请检查 CAN 连接与机械臂上电状态。")
        print("等待机械臂连接...")
        time.sleep(1)

    sv = robot.get_firmware()["software_version"]
    fw = PiperFW.DEFAULT
    if sv >= "S-V1.8-9":
        fw = PiperFW.V189
    elif sv >= "S-V1.8-8":
        fw = PiperFW.V188
    elif sv >= "S-V1.8-3":
        fw = PiperFW.V183

    robot.disconnect()
    return create_agx_arm_config(
        robot=robot_model,
        firmeware_version=fw,
        interface=interface,
        channel=can_port,
    ), sv


def main():
    parser = argparse.ArgumentParser(description="Piper 机械臂基础控制")
    parser.add_argument("--robot", default="piper_h", help="机械臂型号，默认 piper_h")
    parser.add_argument("--can_port", default=None, help="CAN 通道，Linux 默认 can0")
    parser.add_argument("--speed", type=int, default=30, help="速度百分比 1-100，默认 30")
    parser.add_argument("--no-home", action="store_true", help="跳过回零动作")
    parser.add_argument("--monitor", type=float, default=5.0, help="回零后持续读取关节角度秒数，0 表示不读取")
    args = parser.parse_args()

    interface, default_port = resolve_can_backend()
    can_port = args.can_port or default_port

    print(f"连接 Piper: interface={interface}, channel={can_port}")
    probe_cfg = create_agx_arm_config(
        robot=args.robot,
        interface=interface,
        channel=can_port,
    )
    robot = AgxArmFactory.create_arm(probe_cfg)
    robot.connect()

    robot_cfg, firmware_version = detect_firmware_version(
        robot, can_port, interface, args.robot
    )
    robot = AgxArmFactory.create_arm(robot_cfg)
    robot.connect()
    print(f"已连接，固件版本: {firmware_version}")

    print("使能机械臂...")
    while not robot.enable():
        time.sleep(0.01)
    print("使能成功")

    if not wait_robot_comm_ready(robot, timeout=15.0):
        print("警告: 通信未在 15s 内稳定，仍尝试回零", file=sys.stderr)

    robot.set_speed_percent(max(1, min(args.speed, 100)))
    robot.set_installation_pos(robot.OPTIONS.INSTALLATION_POS.HORIZONTAL)
    robot.set_motion_mode(robot.OPTIONS.MOTION_MODE.J)
    time.sleep(0.3)

    if not args.no_home:
        target = [0.0] * robot.joint_nums
        print(f"执行回零 move_j: {target}")
        robot.move_j(target)
        if wait_motion_done(robot, target_joints=target):
            print("回零完成")
        else:
            print("回零超时，请检查机械臂状态", file=sys.stderr)

    if args.monitor > 0:
        print(f"读取关节角度 {args.monitor:.1f}s ...")
        end_t = time.time() + args.monitor
        while time.time() < end_t:
            joint = robot.get_joint_angles()
            if joint is not None:
                values = [round(v, 4) for v in joint.msg]
                print(f"joint(rad): {values}")
            time.sleep(0.5)

    robot.disable()
    robot.disconnect()
    print("已断开连接")


if __name__ == "__main__":
    main()
