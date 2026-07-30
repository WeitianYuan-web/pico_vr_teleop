# backends/jaka

JAKA 双臂 WebXR 遥操作（SDK `servo_p`）。

## 布局

```
backends/jaka/
├── README.md
├── sdk/                        # 运行时代码（入库）
│   ├── vr_teleop_dual.py       # WebXR 主循环
│   ├── jaka_sdk_client.py      # SDK 薄封装
│   ├── motion_utils.py         # 回零 / 跟踪误差 / 位姿打印
│   └── config.py               # IP、SDK 路径、初始关节
├── 20260104145805A007/         # 厂商 SDK 动态库（本机放置，不入库）
└── pdf-docs-jaka-md/           # 厂商文档（可选，不入库）
```

## 依赖（换机必做）

本仓库不含 JAKA 官方 Python SDK 二进制。放到：

```text
backends/jaka/20260104145805A007/
```

需能解析到（与 `sdk/config.py` / 启动脚本一致）：

```text
.../SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu/
  ├── jkrc*.so / libjakaAPI.so 等
```

```bash
rsync -a backends/jaka/20260104145805A007/ user@newhost:pico_vr_teleop/backends/jaka/20260104145805A007/
```

## 启动

仅 VR 双臂（含相机 publisher）：

```bash
./scripts/run_full_stack.sh --backend jaka
./scripts/run_vr_teleop_jaka_dual.sh
```

不要相机 / 不要 CAN（JAKA 走网口）：

```bash
./scripts/run_full_stack.sh --backend jaka --no-publisher --no-can-activate
```

默认臂 IP：左 `192.168.10.21`、右 `192.168.10.11`。可覆盖：

```bash
./scripts/run_full_stack.sh --backend jaka --no-publisher --no-can-activate -- \
  --left-ip 192.168.10.21 --right-ip 192.168.10.11
```

入口：`entrypoints/jaka_dual_webxr.py` → `sdk/vr_teleop_dual.py`。  
启动脚本会把厂商库路径加入 `LD_LIBRARY_PATH`。

## 与 IO / 统一手控共存

JAKA **不占用** Inspire 手串口。手套或其它指令源 + RH56F2：

```bash
./scripts/run_io_gateway.sh
./scripts/run_io_zenoh2ros.sh
./scripts/run_hand_controller.sh -p right_serial_port:=/dev/ttyUSB0 -p left_serial_port:=/dev/ttyUSB1
./scripts/run_full_stack.sh --backend jaka --no-publisher --no-can-activate
```

详见 [controllers/io_hand/README.md](../../controllers/io_hand/README.md)。
