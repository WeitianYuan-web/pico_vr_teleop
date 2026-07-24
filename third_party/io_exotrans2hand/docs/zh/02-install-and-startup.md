# 02 · 安装与启动

> 适用读者：部署工程师（终端用户可直接看 [3.5 快速开始](#25-快速开始清单)）

## 2.1 系统前置要求

| 项目 | 要求 |
|------|------|
| 操作系统 | **Ubuntu 22.04**，架构 **x86_64** |
| Python | 使用 bundle 内置 **Python 3.10**（`bundle/opt/python/bin/python3`），无需系统 Python |
| ROS | 运行时依赖 bundle 内预编译的 ROS2（Humble）组件；硬件桥接脚本需系统 ROS Humble |
| 桌面环境（head 模式） | 需 `xdg-open` / `sensible-browser` 之一，用于自动打开浏览器 |
| 串口权限 | 用户须在 **dialout** 组；外骨骼/灵巧手 USB 串口需 udev 规则 |
| 网络 | HTTP 监听 `0.0.0.0`；无线 UDP 探测需本机存在 `10.42.x` 网段地址 |
| 权限 | 安装 udev 规则需 **sudo** |

构建元信息见 `bundle/BUILD_INFO`：`ubuntu=22.04 / ros_distro=humble / python=3.10 / arch=x86_64-linux-gnu`。

## 2.2 顶层目录结构

```text
{root}/
├── io-gateway.desktop      # 桌面快捷方式模板（由 install-desktop.sh 展开）
├── bundle/                 # 预编译自包含运行时
│   ├── BUILD_INFO          # 构建元信息
│   ├── opt/python/         # 内置 Python 3.10
│   ├── opt/io-deps/        # C++ 依赖 + protoc 等
│   ├── opt/zenoh/          # Zenoh 库
│   ├── python/             # pip site-packages
│   └── install/bin/        # 预编译节点：exo_tf_comm / exo_tf_udp_comm / tf_transform_comm
├── src/                    # 应用源码（大量已 Cython 编译为 .so）
│   ├── io_gateway/         # 网关主程序（FastAPI + 编排 + Zenoh 桥 + Web UI）
│   ├── io_unicontroller/   # 灵巧手手指重定向控制器
│   └── io_bus_proto/       # Protobuf 编解码
├── configs/
│   ├── config/
│   │   ├── gateway.yaml    # 网关主配置（端口、手型、子进程命令、bundle 路径）
│   │   ├── zenoh.json5     # Zenoh 组网（仅 loopback）
│   │   └── topics.yaml     # 话题映射
│   ├── end_tools/          # 各手型配置（Inspire_RH56F2、Inspire_RH5DG2…）
│   ├── exoskeleton_urdf/   # 外骨骼 URDF（3D 可视化）
│   ├── udev/               # 串口 udev 规则模板
│   └── IO.png              # 应用图标
├── scripts/                # 运维与开发脚本
│   ├── install-desktop.sh  # 桌面安装
│   ├── run_gateway.sh      # 网关启动
│   ├── bundle-env.sh       # 环境加载
│   └── Inspire_Hardware_Bridge/  # 灵巧手 RS485 桥接脚本
├── logs/YYYY-MM-DD/        # 运行日志（按日期分目录）
└── tools/                  # 可选调试工具（zenoh2ros_bridge.py 等）
```

## 2.3 安装：`scripts/install-desktop.sh`

将桌面快捷方式模板展开，并可选安装串口 udev 规则、把当前用户加入 dialout 组。

```bash
cd {root}

./scripts/install-desktop.sh                # 默认：桌面 + 应用菜单 + udev
./scripts/install-desktop.sh --no-app-menu  # 不装应用菜单
./scripts/install-desktop.sh --no-udev      # 不装 udev / dialout
IO_EXOTRANS2HAND_ROOT=/opt/io_project ./scripts/install-desktop.sh  # 自定义项目根
```

脚本做了三件事：

1. **生成桌面快捷方式**：读取 `io-gateway.desktop` 模板，将 `@IO_ROOT@` 替换为项目绝对路径，写入桌面目录并赋可执行权限。启动器中可搜索「IO Gateway」或「IO Gesture」。
2. **安装应用菜单入口**（默认）：写入 `~/.local/share/applications/io-gateway.desktop` 并刷新数据库。
3. **安装 udev 串口规则**（默认，需 sudo）：复制 `configs/udev/99-io-exo-serial.rules` 到 `/etc/udev/rules.d/`，reload + trigger，并 `usermod -aG dialout $USER`。

udev 规则要点（`configs/udev/99-io-exo-serial.rules`）：

```text
# STM32 Virtual ComPort（本项目常见外骨骼）
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0660", GROUP="dialout"
# 兜底：所有 ACM / USB 转串口
KERNEL=="ttyACM[0-9]*", MODE="0660", GROUP="dialout"
KERNEL=="ttyUSB[0-9]*", MODE="0660", GROUP="dialout"
```

> **重要**：加入 dialout 组后，须 **注销并重新登录（或重启）** 才会生效，否则打开串口会报权限不足。

## 2.4 启动：`scripts/run_gateway.sh`

### head vs headless

| 模式 | 命令 | 行为 |
|------|------|------|
| **head**（默认） | `./scripts/run_gateway.sh` | 挂载 Web 控制台与 3D 资源；就绪后自动打开浏览器 |
| **headless** | `./scripts/run_gateway.sh --headless` | 仅 REST API + WebSocket；根路径返回 JSON 指引；适合 SSH / systemd |
| head 但不开浏览器 | `./scripts/run_gateway.sh --no-browser` | 或设 `GATEWAY_NO_BROWSER=1` |

启动机制：`run_gateway.sh` 先 `source scripts/bundle-env.sh` 加载 bundle 环境（自动设置 `IO_PYTHON`、`PYTHONPATH`、`LD_LIBRARY_PATH`、`PATH`），再以 bundle Python 启动网关。因 `main` 已 Cython 编译为 `.so`，入口通过 import 调用而非 `python -m`。

### 环境变量

| 变量 | 作用 | 说明 |
|------|------|------|
| `IO_EXOTRANS2HAND_ROOT` | 项目根目录，路径解析基准 | 默认自动推断，可覆盖 |
| `GATEWAY_PORT` | **仅影响** `run_gateway.sh` 的浏览器 URL 与就绪检测地址 | **不覆盖** 实际监听端口 |
| `GATEWAY_NO_BROWSER` | head 模式禁止自动开浏览器 | `=1` 等价 `--no-browser` |
| `IO_PYTHON` / `PREFIX` / `PYTHONPATH` / `LD_LIBRARY_PATH` | bundle 运行时路径 | 由 `bundle-env.sh` 自动设置 |

> **修改实际监听端口** 须编辑 `configs/config/gateway.yaml` 的 `listen_port`，而非只设 `GATEWAY_PORT`。若用脚本自动开浏览器，两者需一致。

## 2.5 启动后的监听地址与端口

配置来源 `configs/config/gateway.yaml`：`listen_host: 0.0.0.0`、`listen_port: 8080`。

| 服务 | 地址 | 说明 |
|------|------|------|
| Web 控制台（head） | `http://<主机IP>:8080/` | 本机 `http://127.0.0.1:8080/` |
| REST API | `http://<主机IP>:8080/api/v1/` | 见 [07 配置参考](./07-configuration-reference.md) |
| API 文档 | `http://<主机IP>:8080/docs` | FastAPI 自动生成 |
| WebSocket | `ws://<主机IP>:8080/ws` | 数据流订阅/发布 |
| 无线 UDP 探测 | `10.42.0.2:8888` | `udp_probe` 段 |
| 无线心跳 | `0.0.0.0:8889` | `wifi_heartbeat` 段 |
| Zenoh | `tcp/127.0.0.1:0`（动态端口，仅 loopback） | `zenoh.json5` |

## 2.6 快速开始清单

1. 确认系统为 **Ubuntu 22.04 x86_64**
2. 放置项目到目标目录
3. 执行 `./scripts/install-desktop.sh`（安装快捷方式 + udev + dialout）
4. **注销重新登录**（使 dialout 生效）
5. 连接外骨骼 USB 设备
6. 执行 `./scripts/run_gateway.sh`（或双击桌面「IO Gesture」图标）
7. 浏览器访问 `http://127.0.0.1:8080/`

## 2.7 常用运维命令

```bash
# 启动（有界面 / 无界面）
./scripts/run_gateway.sh
./scripts/run_gateway.sh --headless

# 自定义项目根
IO_EXOTRANS2HAND_ROOT=/opt/io_project ./scripts/run_gateway.sh

# 查看今日主日志
tail -f logs/$(date +%Y-%m-%d)/io_gateway.log

# 健康检查
curl -s http://127.0.0.1:8080/api/v1/status | python3 -m json.tool
```

## 2.8 开发构建脚本（运维无需执行）

发行包已预编译，以下脚本仅用于构建流水线：`scripts/cython_build.sh`（Cython 编译）、`scripts/gen_protobuf.sh`（protobuf 生成）、`scripts/install_protobuf_bundle.sh`（protobuf 编译安装）。详见 [09 二次开发指南](./09-developer-guide.md)。

---

下一步：[03 Web 控制台](./03-web-console.md)
