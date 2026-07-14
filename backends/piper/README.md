# backends/piper

Piper 双臂（+ Inspire 手）遥操作后端。

## 布局

```
backends/piper/
├── README.md
└── teleop/
    ├── teleop_piper_webxr.py          # 单/双臂 WebXR + Placo IK
    └── dual_arm_dual_hand_webxr.py    # 双臂 + 双手（继承上一文件）
```

## 依赖（`third_party/`）

| 路径 | 作用 |
|------|------|
| `third_party/pyAgxArm/` | Piper CAN SDK + Placo IK 实现 |
| `third_party/InspireHandSDK_Y/` | 灵巧手 SDK |

## 启动

```bash
./scripts/run_full_stack.sh --backend piper
./scripts/run_dual_arm_dual_hand.sh
./scripts/run_vr_teleop.sh --hands both
```

入口：`entrypoints/piper_dual_webxr.py` → `teleop/dual_arm_dual_hand_webxr.py`。
