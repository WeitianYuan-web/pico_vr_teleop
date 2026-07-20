# backends/jaka

JAKA 双臂 WebXR 遥操作（SDK `servo_p`）。

## 布局

```
backends/jaka/
├── README.md
├── sdk/                    # 本仓库追踪：VR / 键盘 / SDK 薄封装
├── tcp_ip/                 # 本仓库追踪：备用 TCP/IP 示例
├── 20260104145805A007/     # 不入库：厂商 SDK 动态库（需本机放置）
└── pdf-docs-jaka-md/       # 不入库：厂商文档（可选）
```

## 换机依赖（必做）

本仓库**不包含** JAKA 官方 Python SDK 二进制。请将厂商包放到：

```text
backends/jaka/20260104145805A007/
```

目录内需能解析到（与 `sdk/config.py` / 启动脚本一致）：

```text
.../SDK V2.3.1_beta3/Linux/x86_64-linux-gnu/Linux/python3/x86_64-linux-gnu/
  ├── jkrc*.so / libjakaAPI.so 等
```

可从当前可用机器整目录拷贝：

```bash
# 在已配好的机器上
rsync -a backends/jaka/20260104145805A007/ user@newhost:pico_vr_teleop/backends/jaka/20260104145805A007/
```

放置后验证：

```bash
./scripts/run_vr_teleop_jaka_dual.sh --hands both --print-status
# 或
./scripts/run_full_stack.sh --backend jaka --no-publisher
```

启动脚本会把上述路径加入 `LD_LIBRARY_PATH`。

## 启动

```bash
./scripts/run_full_stack.sh --backend jaka
# 或
./scripts/run_vr_teleop_jaka_dual.sh
```

入口：`entrypoints/jaka_dual_webxr.py` → `sdk/vr_teleop_dual.py`。
