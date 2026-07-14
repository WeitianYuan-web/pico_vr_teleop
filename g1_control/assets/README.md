# g1_control assets

本目录放置 G1 双臂运动学 URDF。

## 已包含

- `g1_dual_arm.urdf`：仅上肢 14 DoF（来自 Unitree / XRoboToolkit 资源）

IK 使用 `pinocchio.buildModelFromUrdf`，**不需要** mesh 文件。

## 可选：完整可视化资源

若需要 MeshCat / MuJoCo 可视化，可从以下仓库拷贝 meshes：

- https://github.com/unitreerobotics/xr_teleoperate/tree/main/assets/g1
- https://github.com/unitreerobotics/unitree_ros/tree/master/robots/g1_description

## 官方整机 URDF（xr_teleoperate 风格）

若改用 `g1_body29_hand14.urdf`，需自行锁下肢/腰/手指关节后再做 IK；
当前默认 `g1_dual_arm.urdf` 已是锁死后的双臂模型，更适合本仓库接入。
