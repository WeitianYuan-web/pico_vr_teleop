# backends/piper

Piper 双臂（+ Inspire 手）WebXR 遥操作。

## 布局

```
backends/piper/
├── README.md
└── teleop/
    ├── teleop_piper_webxr.py          # 单/双臂 WebXR + Placo IK
    └── dual_arm_dual_hand_webxr.py    # 双臂 + 双手（继承上一文件）
```

## 依赖

| 路径 | 随仓？ | 作用 |
|------|--------|------|
| `third_party/pyAgxArm/` | 是 | Piper CAN + Placo IK |
| `third_party/InspireHandSDK_Y/` | 否 | 灵巧手（可选） |

## 启动

```bash
./scripts/run_full_stack.sh --backend piper
./scripts/run_dual_arm_dual_hand.sh
./scripts/run_vr_teleop.sh --hands both
```

入口：`entrypoints/piper_dual_webxr.py` → `teleop/dual_arm_dual_hand_webxr.py`。
