# entrypoints/

统一 VR 遥操作入口（薄 bootstrap）。按机型选择后端，不放业务逻辑。

| 入口 | 后端主循环 |
|------|------------|
| `piper_dual_webxr.py` | `backends/piper/teleop/dual_arm_dual_hand_webxr.py` |
| `jaka_dual_webxr.py` | `backends/jaka/sdk/vr_teleop_dual.py` |
| `g1_dual_webxr.py` | `backends/g1/vr_teleop_dual.py` |

约定：

- 入口只把项目根与对应 `backends/<robot>/...` 加入 `sys.path`，再调用 `main`
- 机型实现留在 `backends/<robot>/`
- 共用数学 / clutch / WSS 在 `common/`
- 厂商 SDK 在 `third_party/` 或本机厂商目录
- 一键启动：`./scripts/run_full_stack.sh --backend piper|jaka|g1`
