# pico_vr_teleop

PICO WebXR 双臂遥操作：一套 WebXR/WSS 管线，可切换 **Piper / JAKA / Unitree G1** 后端；可选 ROS2 状态发布。

数据流：`PICO 浏览器 → webxr/server.py (WSS) → entrypoints/* → backends/<robot>/`

## 目录

```
pico_vr_teleop/
├── webxr/           # 网页 + HTTPS/WSS
├── entrypoints/     # 薄入口（按机型分发）
├── backends/        # piper | jaka | g1
├── common/          # 共用数学 / clutch / 滤波 / WSS
├── publisher/       # ROS2 + UDP 状态桥
├── third_party/     # pyAgxArm、InspireHandSDK_Y
└── scripts/         # setup / 一键启动
```

细节见各子目录 `README.md`。旧路径（`vr_teleop/`、`webxr_test/`、`g1_control/` 等）仅保留 `MOVED.md` 或转发 shim。

## 初始化

```bash
cd pico_vr_teleop
./scripts/setup_env.sh
source .venv/bin/activate
```

可选：

- Piper 灵巧手：编译 `third_party/InspireHandSDK_Y` 的 Python 绑定（见该目录说明）
- G1：安装 `unitree_sdk2_python` + CycloneDDS（见 [backends/g1/README.md](backends/g1/README.md)）
- WebXR 证书：需 `webxr/cert.pem`、`webxr/key.pem`

## 一键启动（推荐）

```bash
source /opt/ros/jazzy/setup.bash   # 按本机 ROS 调整；不需要 ROS 可加 --no-publisher
source .venv/bin/activate

./scripts/run_full_stack.sh --backend piper
./scripts/run_full_stack.sh --backend jaka
./scripts/run_full_stack.sh --backend g1 -- --motion --network-interface enp12s0
```

常用开关：`--no-can-activate`、`--no-publisher`；`--` 之后参数传给遥操作脚本。

启动后看 `logs/vr_server.log` 里的 HTTPS 地址，用 PICO 浏览器打开并进入透视 AR。

| 后端 | 控制方式 | 专用说明 |
|------|----------|----------|
| piper | CAN + Placo IK + 可选 Inspire 手 | [backends/piper](backends/piper/README.md) |
| jaka | SDK `servo_p` | [backends/jaka](backends/jaka/README.md) |
| g1 | DDS + Placo IK | [backends/g1](backends/g1/README.md) |

## VR 操作

| 输入 | 作用 |
|------|------|
| Grip | 按住接合该侧臂；松开保持 |
| Trigger | Piper 灵巧手张合（松开≈张，按满默认约六成握） |
| B | 回初始 / 偏好姿态 |

## ROS

遥操作经 UDP `127.0.0.1:17981` 上报，由 `publisher/` 发布 `/puppet/*` 与可选 RealSense 图像。详见 [publisher/README.md](publisher/README.md)。

## 更多文档

- [webxr/WEBXR_PIPER_TELEOP.md](webxr/WEBXR_PIPER_TELEOP.md) — Piper VR 参数与坐标系
- [common/README.md](common/README.md) — 三后端共用模块
- [third_party/README.md](third_party/README.md) — 厂商 SDK 安装
