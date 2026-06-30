# pico_vr_teleop

独立的 PICO WebXR + Piper 遥操作项目，已迁移：

- `pyAgxArm` SDK 与运动学解析（Placo QP IK）
- `webxr_test` 的 WebXR 前端、WSS 服务、VR 遥操作脚本
- 两个控制示例：键盘控制、VR 控制

## 目录结构

- `pyAgxArm/`：SDK 与 IK 相关代码
- `webxr_test/`：WebXR 网页、服务、VR 遥操作脚本
- `scripts/setup_env.sh`：创建并安装项目虚拟环境
- `scripts/run_keyboard_teleop.sh`：键盘控制示例
- `scripts/run_vr_teleop.sh`：VR 控制示例

## 一次性初始化

```bash
cd /home/b0106/pico_test/pico_vr_teleop
bash scripts/setup_env.sh
```

## 示例1：键盘控制机械臂

```bash
cd /home/b0106/pico_test/pico_vr_teleop
./scripts/run_keyboard_teleop.sh --can_port can0
```

常用参数：

- `--can_port can0`
- `--speed 40`
- `--disable-on-exit`
- `--no-can-activate`（跳过启动脚本中的 CAN 自动激活）

## 示例2：VR 控制机械臂

先启动 WebXR 服务：

```bash
cd /home/b0106/pico_test/pico_vr_teleop/webxr_test
/home/b0106/pico_test/pico_vr_teleop/.venv/bin/python server.py
```

再启动 VR 遥操作（支持单臂/双臂）：

```bash
cd /home/b0106/pico_test/pico_vr_teleop
./scripts/run_vr_teleop.sh --hands right --right-can-port can0
```

双臂示例：

```bash
./scripts/run_vr_teleop.sh --hands both --left-can-port can0 --right-can-port can1
```

## 说明

- 启动脚本默认会尝试自动激活 CAN 口（需要 `sudo`）。
- 若你的系统使用不同波特率，可设置环境变量：

```bash
CAN_BITRATE=500000 ./scripts/run_vr_teleop.sh --hands right --right-can-port can0
```
