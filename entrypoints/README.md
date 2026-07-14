# entrypoints/

统一 VR 遥操作入口（薄 bootstrap）。按机型选择后端，不放业务逻辑。

- `piper_dual_webxr.py` → `backends/piper/teleop/dual_arm_dual_hand_webxr.py`
- `jaka_dual_webxr.py` → `backends/jaka/sdk/vr_teleop_dual.py`
- `g1_dual_webxr.py` → `backends/g1/vr_teleop_dual.py`

原则：

- 入口负责把**项目根目录**与对应 `backends/<robot>/` 加入 `sys.path`，再调用 `main`
- 机型依赖与实现留在 `backends/<robot>/`
- 共用数学 / clutch / WSS 在 `common/`
- 厂商 SDK 在 `third_party/`
- 一键启动：`./scripts/run_full_stack.sh --backend piper|jaka|g1`

旧路径 `vr_teleop/` 已迁移至此，仅保留兼容提示目录。
