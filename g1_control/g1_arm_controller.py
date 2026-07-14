"""G1 双臂 DDS 控制器（基于 unitree_sdk2_python / xr_teleoperate）。"""

from __future__ import annotations

import threading
import time
from typing import Protocol

import numpy as np

from g1_joints import G1_29_NUM_MOTORS, G1_29_JointArmIndex, G1_29_JointIndex

TOPIC_LOWCMD_DEBUG = "rt/lowcmd"
TOPIC_LOWCMD_MOTION = "rt/arm_sdk"
TOPIC_LOWSTATE = "rt/lowstate"


class DataBuffer:
    def __init__(self) -> None:
        self._data = None
        self._lock = threading.Lock()

    def get(self):
        with self._lock:
            return self._data

    def set(self, data) -> None:
        with self._lock:
            self._data = data


class MotorState:
    def __init__(self) -> None:
        self.q = 0.0
        self.dq = 0.0


class G1LowState:
    def __init__(self) -> None:
        self.motor_state = [MotorState() for _ in range(G1_29_NUM_MOTORS)]


class G1ArmControllerBase(Protocol):
    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None: ...

    def get_current_dual_arm_q(self) -> np.ndarray: ...

    def get_current_dual_arm_dq(self) -> np.ndarray: ...

    def ctrl_dual_arm_go_home(self) -> None: ...

    def release_arm_control(self) -> None: ...

    def close(self) -> None: ...


def init_dds_channel(simulation_mode: bool = False, network_interface: str | None = None) -> None:
    """
    /**
     * @brief 初始化 CycloneDDS ChannelFactory（进程内只需调用一次）
     */
    """
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize

    domain = 1 if simulation_mode else 0
    if network_interface:
        ChannelFactoryInitialize(domain, network_interface)
    else:
        ChannelFactoryInitialize(domain)


class MockG1ArmController:
    """无真机时的空跑控制器，便于联调 WebXR / IK。"""

    def __init__(self) -> None:
        self._q = np.zeros(14, dtype=float)
        self._dq = np.zeros(14, dtype=float)
        self._lock = threading.Lock()
        print("[G1] MockG1ArmController 已启用（--dry-run）")

    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None:
        with self._lock:
            self._q = np.asarray(q_target, dtype=float).reshape(14).copy()

    def get_current_dual_arm_q(self) -> np.ndarray:
        with self._lock:
            return self._q.copy()

    def get_current_dual_arm_dq(self) -> np.ndarray:
        with self._lock:
            return self._dq.copy()

    def ctrl_dual_arm_go_home(self) -> None:
        with self._lock:
            self._q = np.zeros(14, dtype=float)

    def release_arm_control(self) -> None:
        return

    def close(self) -> None:
        return


class G1_29_ArmController:
    """
    /**
     * @brief G1 29DoF 双臂低层 PD 控制器
     *
     * - motion_mode=False: 发布到 rt/lowcmd（Debug，锁住非臂关节）
     * - motion_mode=True:  发布到 rt/arm_sdk，并用关节 29 的 q 作为 weight
     *
     * 调用前需先执行 init_dds_channel()。
     */
    """

    def __init__(
        self,
        motion_mode: bool = False,
        simulation_mode: bool = False,
        arm_velocity_limit: float = 20.0,
        control_hz: float = 250.0,
    ):
        from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_ as HgLowCmd
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_ as HgLowState
        from unitree_sdk2py.utils.crc import CRC

        print("[G1] 初始化 G1_29_ArmController ...")
        self.q_target = np.zeros(14, dtype=float)
        self.tauff_target = np.zeros(14, dtype=float)
        self.motion_mode = bool(motion_mode)
        self.simulation_mode = bool(simulation_mode)
        self.kp_high = 300.0
        self.kd_high = 3.0
        self.kp_low = 80.0
        self.kd_low = 3.0
        self.kp_wrist = 40.0
        self.kd_wrist = 1.5
        self.arm_velocity_limit = float(arm_velocity_limit)
        self.control_dt = 1.0 / max(1.0, float(control_hz))
        self._stop = False
        self._weight = 1.0 if self.motion_mode else 0.0

        topic = TOPIC_LOWCMD_MOTION if self.motion_mode else TOPIC_LOWCMD_DEBUG
        self.lowcmd_publisher = ChannelPublisher(topic, HgLowCmd)
        self.lowcmd_publisher.Init()
        self.lowstate_subscriber = ChannelSubscriber(TOPIC_LOWSTATE, HgLowState)
        self.lowstate_subscriber.Init()
        self.lowstate_buffer = DataBuffer()

        self._subscribe_thread = threading.Thread(target=self._subscribe_motor_state, daemon=True)
        self._subscribe_thread.start()

        while self.lowstate_buffer.get() is None:
            time.sleep(0.1)
            print("[G1] 等待 rt/lowstate ...")
        print("[G1] rt/lowstate 已连接")

        self.crc = CRC()
        self.msg = unitree_hg_msg_dds__LowCmd_()
        self.msg.mode_pr = 0
        self.msg.mode_machine = self.get_mode_machine()

        all_q = self.get_current_motor_q()
        self.q_target = self.get_current_dual_arm_q()
        arm_indices = {m.value for m in G1_29_JointArmIndex}
        for jid in G1_29_JointIndex:
            self.msg.motor_cmd[jid].mode = 1
            if jid.value in arm_indices:
                if self._is_wrist_motor(jid):
                    self.msg.motor_cmd[jid].kp = self.kp_wrist
                    self.msg.motor_cmd[jid].kd = self.kd_wrist
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low
            else:
                if self._is_weak_motor(jid):
                    self.msg.motor_cmd[jid].kp = self.kp_low
                    self.msg.motor_cmd[jid].kd = self.kd_low
                else:
                    self.msg.motor_cmd[jid].kp = self.kp_high
                    self.msg.motor_cmd[jid].kd = self.kd_high
            self.msg.motor_cmd[jid].q = float(all_q[jid])

        self.ctrl_lock = threading.Lock()
        if self.motion_mode:
            self._ramp_weight(0.0, 1.0, duration_s=1.0)

        self._publish_thread = threading.Thread(target=self._ctrl_motor_state, daemon=True)
        self._publish_thread.start()
        print(f"[G1] G1_29_ArmController 就绪 (topic={topic}, motion={self.motion_mode})")

    def _subscribe_motor_state(self) -> None:
        while not self._stop:
            msg = self.lowstate_subscriber.Read()
            if msg is not None:
                lowstate = G1LowState()
                for idx in range(G1_29_NUM_MOTORS):
                    lowstate.motor_state[idx].q = msg.motor_state[idx].q
                    lowstate.motor_state[idx].dq = msg.motor_state[idx].dq
                self.lowstate_buffer.set(lowstate)
            time.sleep(0.002)

    def clip_arm_q_target(self, target_q: np.ndarray, velocity_limit: float) -> np.ndarray:
        current_q = self.get_current_dual_arm_q()
        delta = target_q - current_q
        motion_scale = float(np.max(np.abs(delta)) / (velocity_limit * self.control_dt))
        return current_q + delta / max(motion_scale, 1.0)

    def _ctrl_motor_state(self) -> None:
        while not self._stop:
            start = time.time()
            with self.ctrl_lock:
                arm_q_target = self.q_target.copy()
                arm_tauff_target = self.tauff_target.copy()
                weight = self._weight

            if self.simulation_mode:
                clipped = arm_q_target
            else:
                clipped = self.clip_arm_q_target(arm_q_target, self.arm_velocity_limit)

            for idx, jid in enumerate(G1_29_JointArmIndex):
                self.msg.motor_cmd[jid].q = float(clipped[idx])
                self.msg.motor_cmd[jid].dq = 0.0
                self.msg.motor_cmd[jid].tau = float(arm_tauff_target[idx])

            if self.motion_mode:
                self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(weight)

            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)

            elapsed = time.time() - start
            time.sleep(max(0.0, self.control_dt - elapsed))

    def ctrl_dual_arm(self, q_target: np.ndarray, tauff_target: np.ndarray) -> None:
        with self.ctrl_lock:
            self.q_target = np.asarray(q_target, dtype=float).reshape(14).copy()
            self.tauff_target = np.asarray(tauff_target, dtype=float).reshape(14).copy()

    def get_mode_machine(self) -> int:
        msg = self.lowstate_subscriber.Read()
        return int(msg.mode_machine) if msg is not None else 0

    def get_current_motor_q(self) -> np.ndarray:
        state = self.lowstate_buffer.get()
        return np.array([state.motor_state[jid].q for jid in G1_29_JointIndex], dtype=float)

    def get_current_dual_arm_q(self) -> np.ndarray:
        state = self.lowstate_buffer.get()
        return np.array([state.motor_state[jid].q for jid in G1_29_JointArmIndex], dtype=float)

    def get_current_dual_arm_dq(self) -> np.ndarray:
        state = self.lowstate_buffer.get()
        return np.array([state.motor_state[jid].dq for jid in G1_29_JointArmIndex], dtype=float)

    def ctrl_dual_arm_go_home(self) -> None:
        print("[G1] 双臂回零位 ...")
        with self.ctrl_lock:
            self.q_target = np.zeros(14, dtype=float)
        for _ in range(100):
            if np.all(np.abs(self.get_current_dual_arm_q()) < 0.05):
                print("[G1] 双臂已回零")
                break
            time.sleep(0.05)

    def _ramp_weight(self, start: float, end: float, duration_s: float = 1.0) -> None:
        steps = max(1, int(duration_s / 0.02))
        for w in np.linspace(start, end, num=steps + 1):
            if hasattr(self, "ctrl_lock"):
                with self.ctrl_lock:
                    self._weight = float(w)
            else:
                self._weight = float(w)
            # 尚未启动 publish 线程时直接写一次
            self.msg.motor_cmd[G1_29_JointIndex.kNotUsedJoint0].q = float(w)
            self.msg.crc = self.crc.Crc(self.msg)
            self.lowcmd_publisher.Write(self.msg)
            time.sleep(0.02)

    def release_arm_control(self) -> None:
        """
        /**
         * @brief motion 模式下将 weight 从 1 降到 0，交还运控
         */
        """
        if not self.motion_mode:
            return
        print("[G1] 释放臂控 weight 1 -> 0 ...")
        self._ramp_weight(self._weight, 0.0, duration_s=1.0)

    def close(self) -> None:
        try:
            self.release_arm_control()
        except Exception:
            pass
        self._stop = True

    @staticmethod
    def _is_weak_motor(motor_index: G1_29_JointIndex) -> bool:
        weak = {
            G1_29_JointIndex.kLeftAnklePitch.value,
            G1_29_JointIndex.kRightAnklePitch.value,
            G1_29_JointIndex.kLeftShoulderPitch.value,
            G1_29_JointIndex.kLeftShoulderRoll.value,
            G1_29_JointIndex.kLeftShoulderYaw.value,
            G1_29_JointIndex.kLeftElbow.value,
            G1_29_JointIndex.kRightShoulderPitch.value,
            G1_29_JointIndex.kRightShoulderRoll.value,
            G1_29_JointIndex.kRightShoulderYaw.value,
            G1_29_JointIndex.kRightElbow.value,
        }
        return motor_index.value in weak

    @staticmethod
    def _is_wrist_motor(motor_index: G1_29_JointIndex) -> bool:
        wrist = {
            G1_29_JointIndex.kLeftWristRoll.value,
            G1_29_JointIndex.kLeftWristPitch.value,
            G1_29_JointIndex.kLeftWristYaw.value,
            G1_29_JointIndex.kRightWristRoll.value,
            G1_29_JointIndex.kRightWristPitch.value,
            G1_29_JointIndex.kRightWristYaw.value,
        }
        return motor_index.value in wrist


def create_arm_controller(
    *,
    dry_run: bool = False,
    motion_mode: bool = False,
    simulation_mode: bool = False,
    network_interface: str | None = None,
    arm_velocity_limit: float = 20.0,
) -> G1ArmControllerBase:
    """
    /**
     * @brief 创建臂控制器；非 dry-run 时会初始化 DDS
     */
    """
    if dry_run:
        return MockG1ArmController()
    try:
        init_dds_channel(simulation_mode=simulation_mode, network_interface=network_interface)
    except Exception as exc:
        iface_hint = network_interface or "(默认网卡)"
        raise RuntimeError(
            "初始化 unitree_sdk2_python DDS 失败。\n"
            f"当前 --network-interface={iface_hint}\n"
            "请用 `ip -br addr` 确认有线网卡名（本机 G1 网段一般为 enp12s0，不是 eth0）。\n"
            "若尚未安装 SDK:\n"
            "  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git\n"
            "  CYCLONEDDS_HOME=... pip install -e unitree_sdk2_python\n"
            f"原始错误: {exc}"
        ) from exc
    return G1_29_ArmController(
        motion_mode=motion_mode,
        simulation_mode=simulation_mode,
        arm_velocity_limit=arm_velocity_limit,
    )
