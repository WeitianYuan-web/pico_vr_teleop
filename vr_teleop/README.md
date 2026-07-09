# VR Teleop Entrypoints

该目录只放 **VR 遥操作入口脚本**，用于统一启动与管理。

- `piper_dual_webxr.py`：Piper 双臂/双手 VR 入口（调用 `control/dual_arm_dual_hand_webxr.py`）
- `jaka_dual_webxr.py`：JAKA 双臂 VR 入口（调用 `jaka_control/sdk/vr_teleop_dual.py`）

原则：

- 各机械臂 SDK 依赖与实现逻辑留在各自目录（如 `pyAgxArm/`、`jaka_control/sdk/`）
- `vr_teleop/` 仅做入口聚合，便于统一维护与一键启动脚本对接
