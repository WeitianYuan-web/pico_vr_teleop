# backends/g1/assets

仅放置正式运动学资源。

## 已包含

- `g1_dual_arm.urdf`：上肢 14 DoF（来自 Unitree / XRoboToolkit）

Placo 运行时生成的无 mesh 临时 URDF 写入系统 `/tmp`（`g1_placo_*.urdf`），**不要**提交到本目录。

IK 使用 Pinocchio / Placo，不需要 mesh 文件。

## 可选可视化

完整 meshes 可从：

- https://github.com/unitreerobotics/xr_teleoperate/tree/main/assets/g1
- https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description
