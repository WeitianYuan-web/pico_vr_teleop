# common/

三机械臂 VR 遥操作共用库（第二步抽取）。

| 模块 | 内容 |
|------|------|
| `constants.py` | HANDS / 按钮索引 / 默认 WSS |
| `coord_frames.py` | 头显→世界系预设（X 前 / Y 前） |
| `math_quat.py` | 四元数与旋转矩阵 |
| `math_se3.py` | XR 手柄变换、位姿增量 |
| `math_euler.py` | JAKA TCP 欧拉 ↔ 四元数 |
| `vr_input.py` | 按钮 / 旋转模式 |
| `filters.py` | EMA、alpha 滤波、WeightedMovingFilter |
| `clutch.py` | Grip 相对位姿内核 |
| `ws_client.py` | WebXR WSS 重连循环 |

使用前确保项目根目录在 `sys.path`（`entrypoints/*` 入口已处理）。

**不抽**：各机型 connect / IK / servo 下发、G1 `hold_q`、Piper 朝前保护等。
