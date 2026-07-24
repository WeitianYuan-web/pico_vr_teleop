# 05 · 手型配置

> 适用读者：终端用户 / 二次开发者

「手型（hand）」是一款灵巧手的完整配置包，位于 `configs/end_tools/<HandName>/`。选定手型后，网关会为其启动坐标对齐（transform）与手指重定向（controller）子进程。

## 5.1 已交付手型

| 手型目录 | 说明 | 自由度 |
|----------|------|--------|
| `Inspire_RH56F2` | Inspire RH56F2 六电缸手 | 6 |
| `Inspire_RH5DG2` | Inspire RH5DG2 十三自由度手 | 13 |

> `scripts/Inspire_Hardware_Bridge/` 下另有 `inspire_rh56e2_teleop_bridge.py`（对应 `Inspire_RH56E2`），但 `configs/end_tools/` 中 **暂无 RH56E2 手型包**，仅有桥接脚本模板。

## 5.2 手型目录结构

```text
configs/end_tools/<HandName>/
├── tf_transform_v2.yml          # 外骨骼 TF -> 手坐标系 TF（对齐）
├── controller_v2_3_left.yml     # 左手指重定向控制器
├── controller_v2_3_right.yml    # 右手指重定向控制器
├── urdf/                        # 手 URDF（至少一个）
└── meshes/                      # 可视化网格 STL（上传校验要求）
```

前端上传时会硬性校验 **3 个 yml 文件** 是否齐全：`tf_transform_v2.yml`、`controller_v2_3_left.yml`、`controller_v2_3_right.yml`；压缩包/文件夹还须含 `urdf/`、`meshes/`。

> 注：`Inspire_RH5DG2` 当前 URDF 内以 `../meshes/*.STL` 引用网格，目录内可能未单独附 `meshes/`；自行打包上传新手型时请保证 `meshes/` 存在以通过校验。

## 5.3 配置文件说明

### A. `tf_transform_v2.yml` — 坐标对齐
`tf_transform_comm` 子进程读取外骨骼 TF，按 `tf_list` 变换后发布「对齐后的手 TF」。

```yaml
sub_topic:
  tf: /io_fusion/tf_exoskeleton      # 订阅外骨骼融合 TF（逻辑名）
pub_topic:
  tf: /io_align/tf_hand              # 发布对齐后的手 TF
  pose: /io_align/poses              # 可视化 Pose（按 frame 拆分）
rate: 100                            # 发布频率 Hz
tf_list:
# [exo_parent, exo_child, hand_parent, hand_child, parent_RPY, child_RPY]
- [right_hand, right_thumb_tip, right_hand_ee_link, right_thumb_tip, [-1.5708,0,1.5708], [0,3.1416,0]]
```

运行时逻辑名会加手型命名空间，实际 Zenoh key 形如 `io_align/<Hand>/tf_hand`、`io_align/<Hand>/poses_<frame>`。

### B. `controller_v2_3_left.yml` / `controller_v2_3_right.yml` — 手指重定向
`io_unicontroller`（`control_v2_3_zenoh`）根据对齐 TF 做逆运动学/优化，输出手指关节角 `JointState`。

```yaml
enable: True
ros_interface:
  node_name: Finger_Retarget_left
  rate: 100
  sub_topic:
    tf_target: "/io_align/tf_hand"
    joint_state: "/io_teleop/joint_states"     # 可选反馈，当前版本可不依赖
  pub_topic:
    joint_target: "/io_teleop/joint_cmd_finger_left"
model:
  urdf: urdf/RH56F2_dual.urdf                   # 相对手型目录
  free_joints: [ left_thumb_1_joint, ... ]      # 参与优化的关节
task:
  pose:   [ [["left_hand_ee_link","left_thumb_tip"], 1, 0, 1.0], ... ]  # [ [base,tip], 位置权重, 姿态权重, 位置scale ]
  vector: [ [["left_hand_ee_link","left_thumb_tip","left_index_tip"], 0.03, 400, 0], ... ]  # 指间约束
  smooth: 0.1
```

| 字段 | 含义 |
|------|------|
| `enable` | 是否启用该侧 |
| `ros_interface.rate` | 控制循环频率（通常 100 Hz） |
| `sub_topic.tf_target` | 订阅对齐后的手 TF |
| `pub_topic.joint_target` | 发布手指关节指令 |
| `model.urdf` | 相对手型目录的 URDF 路径 |
| `model.free_joints` | 参与优化的 URDF 关节名 |
| `task.pose` | 指尖位置跟踪：`[[base_link,tip_link], pos_weight, rot_weight, pos_scale]` |
| `task.vector` | 指间距离约束：`[[base,tip1,tip2], work_thr, max_weight, adsorption_thr]` |
| `task.smooth` | 输出平滑系数 |

运行时节点名会带手型后缀，如 `Finger_Retarget_left_Inspire_RH56F2`；关节指令实际 key 为 `io_teleop/<Hand>/joint_cmd_finger_left`。

### C. `urdf/` — 运动学模型
- RH56F2：控制用 `RH56F2_dual.urdf`（双手）；另有 `RH56F2_L.urdf` / `RH56F2_R.urdf`。
- RH5DG2：`Inspire_RH5DG2.urdf`（双手合一，18 个 revolute，其中 13 主动 + 5 mimic 从动）。

URDF 中必须含 task 引用的 link：各指 `*_tip`、`*_hand_ee_link` 等。

## 5.4 上传、应用、多手型

### 上传（Web）
1. 拖放 zip/tar.gz，或 **Shift+点击** 选型号根文件夹。
2. 前端校验目录结构（`型号名/urdf/`、`meshes/`、3 个 yml）。
3. 撞名时确认覆盖。
4. 点「上传配置」（`POST /api/v1/hands/configs/upload`）。

手型名规则：`^[A-Za-z0-9_]{1,128}$`。

### 应用
在型号选择区勾选（可多选）→ 点「应用」（`POST /api/v1/hands/select`，body `{hands:[...]}`）。空数组表示清空并停止 transform/controller。应用结果也会写入 `gateway.yaml` 的 `hand_choose`。

### 多手型
`hand_choose` 可含多个手型；每个手型各启一套 `transform@Hand`，并按当前拓扑启 left/right controller。例如双手 + 两手型并行时：
```
desired = [exo_tf_udp,
           transform@Inspire_RH56F2, controller_left@..., controller_right@...,
           transform@Inspire_RH5DG2, controller_left@..., controller_right@...]
```

## 5.5 编排与拓扑

| 拓扑 | 外骨骼进程 | transform | controller |
|------|-----------|-----------|------------|
| `left`（有线左） | `exo_tf` | `transform@<Hand>` ×N | `controller_left@<Hand>` ×N |
| `right`（无线右） | `exo_tf_udp` | 同上 | `controller_right@<Hand>` ×N |
| `both` | `exo_tf_udp` | 同上 | left + right ×N |
| `none` | 无 | 无 | 无 |

> 网关 **不会** 自动启动灵巧手 RS485 硬件桥接——它只负责「外骨骼 → 对齐 → 关节指令」的软件链路。硬件下发见 [06 遥操作与硬件桥接](./06-teleop-and-bridge.md)。

## 5.6 手型相关日志
```text
logs/<YYYY-MM-DD>/transform_<Hand>.log
logs/<YYYY-MM-DD>/controller_left_<Hand>.log
logs/<YYYY-MM-DD>/controller_right_<Hand>.log
```

---

下一步：[06 遥操作与硬件桥接](./06-teleop-and-bridge.md)
