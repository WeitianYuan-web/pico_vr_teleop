# backends/tianyee/scripts

天轶运维脚本（安装/启动 bridge、查状态等）。本机常用入口仍保留在仓库根 `scripts/` 的薄包装里：

| 包装 | 实际脚本 |
|------|----------|
| `scripts/run_tianyee_bridge_install.sh` | `install_tianyee_bridge_on_robot.py` |
| `scripts/run_tianyee_udp_bridge.sh` | `start_tianyee_udp_bridge.py` |
| `scripts/run_tianyee_robot_stack.sh` | `start_tianyee_robot_stack.py` |
| `scripts/query_tianyee_bridge_status.py` | `query_tianyee_bridge_status.py` |
| `scripts/run_vr_teleop_tianyee.sh` | → `entrypoints/tianyee_dual_webxr.py` |

这些是本仓库一等脚本，**不要**放进 `third_party/`（那里只放厂商 SDK）。
