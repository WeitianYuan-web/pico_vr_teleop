# backends/

三机械臂后端包。每个后端一个目录，对外由 `entrypoints/` 薄入口分发；共用逻辑在 `common/`；厂商 SDK 在 `third_party/`。

```
pico_vr_teleop/
├── common/
├── entrypoints/
├── webxr/
├── third_party/
│   ├── pyAgxArm/
│   └── InspireHandSDK_Y/
└── backends/
    ├── piper/
    ├── jaka/
    └── g1/
```

| 后端 | 遥操作实现 | 统一入口 | 坐标系预设 |
|------|-----------|----------|------------|
| piper | `piper/teleop/` | `entrypoints/piper_dual_webxr.py` | `x_forward` |
| jaka | `jaka/sdk/vr_teleop_dual.py` | `entrypoints/jaka_dual_webxr.py` | `y_forward` |
| g1 | `g1/vr_teleop_dual.py` | `entrypoints/g1_dual_webxr.py` | `x_forward` |
