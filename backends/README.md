# backends/

三机械臂后端。统一约定：

- 入口：`entrypoints/<robot>_dual_webxr.py`
- 主循环：各后端的 `vr_teleop_dual.py`（Piper 为 `teleop/dual_arm_dual_hand_webxr.py`）
- 共用逻辑：`common/`
- 厂商大包：本机放置，不入库

| 后端 | 运行时代码 | 本机另放 |
|------|------------|----------|
| piper | `piper/teleop/` | Inspire（可选）→ `third_party/InspireHandSDK_Y/` |
| jaka | `jaka/sdk/` | 厂商包 → `jaka/20260104145805A007/` |
| g1 | `g1/` | `unitree_sdk2_python` + CycloneDDS |
| tianyee | `tianyee/` | 机器人 XARM；UDP 桥见 `tianyee/README.md` |

跨设备清单见 [DEPENDENCIES.md](../DEPENDENCIES.md)。

IO 外骨骼 / VR 扳机控手（统一执行节点）：[controllers/io_hand/README.md](../controllers/io_hand/README.md)。
