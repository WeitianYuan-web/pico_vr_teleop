# pico_vr_teleop

PICO WebXR 双臂遥操作：一套 WebXR/WSS 管线，可切换 **Piper / JAKA / Unitree G1 / 天轶(tianyee) / Noetix M1** 后端；可选 ROS2 状态发布与 RealSense；灵巧手经**统一手控**执行（VR / IO 只发指令）。

数据流：`PICO 浏览器 → webxr/server.py (WSS) → entrypoints/* → backends/<robot>/`

## 目录

```
pico_vr_teleop/
├── webxr/              # 网页 + HTTPS/WSS
├── entrypoints/        # 薄入口（按机型分发）
├── backends/           # piper | jaka | g1 | tianyee | noetix
├── common/             # 共用数学 / clutch / 滤波 / WSS
├── controllers/io_hand/ # 统一手控 RH56F2 / RH5DG2（唯一占串口）
├── publisher/          # ROS2 + UDP 臂状态 + RealSense
├── third_party/        # pyAgxArm、InspireHandSDK_Y、io_exotrans2hand、cartesian_min_ws（厂商 SDK，不入库）
└── scripts/            # setup / 一键启动 / 手控包装 / 天轶桥入口（薄包装）
```
## 初始化

```bash
cd pico_vr_teleop
./scripts/setup_env.sh
source .venv/bin/activate
```

详见 **[DEPENDENCIES.md](DEPENDENCIES.md)**。手控需编译 `InspireHandSDK_Y`；IO 手套另需 `sync_io_exotrans2hand.sh` 与 `protobuf>=5.28`。

## 一键启动（臂）

```bash
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

./scripts/run_full_stack.sh --backend piper
./scripts/run_full_stack.sh --backend jaka
./scripts/run_full_stack.sh --backend g1 -- --motion --network-interface enp12s0
./scripts/run_full_stack.sh --backend tianyee
./scripts/run_full_stack.sh --backend noetix
```

Tianyee 与其它后端一键启动时，本机 ROS（publisher）默认都在 **Domain 42**（`LOCAL_ROS_DOMAIN_ID`），
避免与机上/实验室常见 Domain 0 串网。天轶额外使用 loopback/SHM Fast DDS 配置。
**Noetix 遥操作例外**：子进程用 `rmw_cyclonedds_cpp` + `cartesian_min_ws` 的 `cyclonedds.xml` 直连机器人，不套 Domain 42/FastDDS；publisher 仍走本机隔离。
可用 `LOCAL_ROS_DOMAIN_ID` 修改；仅在明确需要进 Domain 0 时设 `LOCAL_ROS_ISOLATION=0`。
旧变量名 `TIANYEE_LOCAL_ROS_DOMAIN_ID` / `TIANYEE_ROS_ISOLATION` 仍可用。

天轶机器人侧建议先持久安装 UDP 桥（开机自启 + 状态监控）：

```bash
./scripts/run_tianyee_bridge_install.sh
python3 scripts/query_tianyee_bridge_status.py
```

常用开关：`--no-can-activate`、`--no-publisher`；`--` 之后参数传给遥操作。

| 后端 | 说明 |
|------|------|
| piper | CAN + Placo；Trigger 默认发 `/hand_cmd`（不占手串口） |
| jaka | SDK `servo_p` |
| g1 | DDS + Placo IK |
| tianyee | WebXR → UDP → 机器人 `/endposetarget_*`（见 `backends/tianyee/README.md`） |
| noetix | WebXR → CycloneDDS → `/Cartesian_Cmd_Topic`（见 `backends/noetix/README.md`） |

## 统一手控（指令源与执行解耦）

```text
VR Trigger 或 IO 手套  →  只发 JointState 指令
rh56f2_controller     →  唯一写 RS485，并发布 /puppet/hand_*
```

### VR 扳机 + Piper

```bash
./scripts/run_hand_controller.sh
./scripts/run_full_stack.sh --backend piper
```

### IO 手套 + 臂

```bash
./scripts/run_io_gateway.sh
./scripts/run_io_zenoh2ros.sh
./scripts/run_hand_controller.sh
./scripts/run_full_stack.sh --backend piper -- --disable-hands
# 或 JAKA：
# ./scripts/run_full_stack.sh --backend jaka --no-publisher --no-can-activate
```

RH5DG2（G2，13 DOF）把三个脚本都加上 `--model rh5dg2`，或先 `export IO_HAND_MODEL=rh5dg2`。只接一只手时加 `-p enable_left_hand:=false`。

完整说明：[controllers/io_hand/README.md](controllers/io_hand/README.md)。

## VR 操作

| 输入 | 作用 |
|------|------|
| Grip | 按住接合该侧臂；松开保持 |
| Trigger | 发手指令到 `/hand_cmd`（需手控节点）；`--disable-hands` 时不发 |
| B | 回初始 / 偏好姿态 |

## ROS / 相机

- 臂状态：遥操作 UDP → `publisher` → `/puppet/joint_*`、`/puppet/end_pose_*`
- 手状态：默认由 `run_hand_controller.sh` → `/puppet/hand_*`（publisher 的 `publish_hands_from_udp` 默认关闭）
- 相机：随 `run_full_stack`；不要相机加 `--no-publisher`

详见 [publisher/README.md](publisher/README.md)。

## 更多文档

- [DEPENDENCIES.md](DEPENDENCIES.md)
- [controllers/io_hand/README.md](controllers/io_hand/README.md)
- [backends/piper](backends/piper/README.md) / [jaka](backends/jaka/README.md) / [g1](backends/g1/README.md) / [tianyee](backends/tianyee/README.md) / [noetix](backends/noetix/README.md)
- [third_party/README.md](third_party/README.md)
