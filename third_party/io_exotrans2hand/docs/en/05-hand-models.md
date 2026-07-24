# 05 · Hand Models

> Audience: End users / Developers

A "hand" is the full config package of one dexterous hand under `configs/end_tools/<HandName>/`. Once a hand is selected, the gateway starts its coordinate-alignment (transform) and finger-retargeting (controller) subprocesses.

## 5.1 Shipped hands

| Directory | Description | DOF |
|-----------|-------------|-----|
| `Inspire_RH56F2` | Inspire RH56F2 six-cylinder hand | 6 |
| `Inspire_RH5DG2` | Inspire RH5DG2 13-DOF hand | 13 |

> `scripts/Inspire_Hardware_Bridge/` also has `inspire_rh56e2_teleop_bridge.py` (for `Inspire_RH56E2`), but `configs/end_tools/` has **no RH56E2 package yet** — only the bridge script template.

## 5.2 Hand directory layout

```text
configs/end_tools/<HandName>/
├── tf_transform_v2.yml          # Exoskeleton TF -> hand-frame TF (alignment)
├── controller_v2_3_left.yml     # Left finger retargeting controller
├── controller_v2_3_right.yml    # Right finger retargeting controller
├── urdf/                        # Hand URDF (at least one)
└── meshes/                      # Visualization STL (required by upload validation)
```

The web uploader strictly checks that the **three yml files** exist: `tf_transform_v2.yml`, `controller_v2_3_left.yml`, `controller_v2_3_right.yml`; the archive/folder must also contain `urdf/` and `meshes/`.

> Note: `Inspire_RH5DG2` currently references meshes via `../meshes/*.STL` inside its URDF and may not ship a local `meshes/`. When packaging a new hand for upload, ensure `meshes/` exists to pass validation.

## 5.3 Config files

### A. `tf_transform_v2.yml` — alignment
The `tf_transform_comm` subprocess reads the exoskeleton TF, transforms it per `tf_list`, and publishes the aligned hand TF.

```yaml
sub_topic:
  tf: /io_fusion/tf_exoskeleton      # subscribe exoskeleton fused TF (logical)
pub_topic:
  tf: /io_align/tf_hand              # publish aligned hand TF
  pose: /io_align/poses              # viz Pose (split by frame)
rate: 100                            # publish rate Hz
tf_list:
# [exo_parent, exo_child, hand_parent, hand_child, parent_RPY, child_RPY]
- [right_hand, right_thumb_tip, right_hand_ee_link, right_thumb_tip, [-1.5708,0,1.5708], [0,3.1416,0]]
```

At runtime, logical names are namespaced by hand: actual Zenoh keys look like `io_align/<Hand>/tf_hand`, `io_align/<Hand>/poses_<frame>`.

### B. `controller_v2_3_left.yml` / `controller_v2_3_right.yml` — retargeting
`io_unicontroller` (`control_v2_3_zenoh`) runs IK/optimization on the aligned TF and outputs finger joint angles as `JointState`.

```yaml
enable: True
ros_interface:
  node_name: Finger_Retarget_left
  rate: 100
  sub_topic:
    tf_target: "/io_align/tf_hand"
    joint_state: "/io_teleop/joint_states"     # optional feedback, not required in this version
  pub_topic:
    joint_target: "/io_teleop/joint_cmd_finger_left"
model:
  urdf: urdf/RH56F2_dual.urdf                   # relative to hand dir
  free_joints: [ left_thumb_1_joint, ... ]      # optimized joints
task:
  pose:   [ [["left_hand_ee_link","left_thumb_tip"], 1, 0, 1.0], ... ]  # [ [base,tip], pos_weight, rot_weight, pos_scale ]
  vector: [ [["left_hand_ee_link","left_thumb_tip","left_index_tip"], 0.03, 400, 0], ... ]  # inter-finger constraints
  smooth: 0.1
```

| Field | Meaning |
|-------|---------|
| `enable` | Whether this side is enabled |
| `ros_interface.rate` | Control loop rate (usually 100 Hz) |
| `sub_topic.tf_target` | Subscribe aligned hand TF |
| `pub_topic.joint_target` | Publish finger joint commands |
| `model.urdf` | URDF path relative to hand dir |
| `model.free_joints` | URDF joints included in optimization |
| `task.pose` | Fingertip position tracking: `[[base_link,tip_link], pos_weight, rot_weight, pos_scale]` |
| `task.vector` | Inter-finger distance constraint: `[[base,tip1,tip2], work_thr, max_weight, adsorption_thr]` |
| `task.smooth` | Output smoothing factor |

At runtime the node name gets a hand suffix, e.g. `Finger_Retarget_left_Inspire_RH56F2`; the joint-command key is `io_teleop/<Hand>/joint_cmd_finger_left`.

### C. `urdf/` — kinematic model
- RH56F2: control uses `RH56F2_dual.urdf` (dual); also `RH56F2_L.urdf` / `RH56F2_R.urdf`.
- RH5DG2: `Inspire_RH5DG2.urdf` (combined; 18 revolute = 13 active + 5 mimic).

The URDF must contain the links referenced by tasks: `*_tip`, `*_hand_ee_link`, etc.

## 5.4 Upload, apply, multi-hand

### Upload (web)
1. Drag-drop a zip/tar.gz, or **Shift+click** to pick the model root folder.
2. The frontend validates layout (`Name/urdf/`, `meshes/`, three yml).
3. Confirm overwrite on name clash.
4. Click "Upload config" (`POST /api/v1/hands/configs/upload`).

Hand-name rule: `^[A-Za-z0-9_]{1,128}$`.

### Apply
Select model(s) then Apply (`POST /api/v1/hands/select` body `{hands:[...]}`). An empty array clears and stops transform/controller. The result is also written to `hand_choose` in `gateway.yaml`.

### Multi-hand
`hand_choose` may contain several hands; each starts its own `transform@Hand` plus left/right controllers per topology. For dual + two hands:
```
desired = [exo_tf_udp,
           transform@Inspire_RH56F2, controller_left@..., controller_right@...,
           transform@Inspire_RH5DG2, controller_left@..., controller_right@...]
```

## 5.5 Orchestration by topology

| Topology | Exo process | transform | controller |
|----------|-------------|-----------|------------|
| `left` (wired) | `exo_tf` | `transform@<Hand>` xN | `controller_left@<Hand>` xN |
| `right` (wireless) | `exo_tf_udp` | same | `controller_right@<Hand>` xN |
| `both` | `exo_tf_udp` | same | left + right xN |
| `none` | none | none | none |

> The gateway does **not** auto-start the RS485 hardware bridge — it only handles the software chain (exoskeleton to alignment to joint commands). Hardware output: [06 Teleop & Hardware Bridge](./06-teleop-and-bridge.md).

## 5.6 Hand logs
```text
logs/<YYYY-MM-DD>/transform_<Hand>.log
logs/<YYYY-MM-DD>/controller_left_<Hand>.log
logs/<YYYY-MM-DD>/controller_right_<Hand>.log
```

---

Next: [06 Teleop & Hardware Bridge](./06-teleop-and-bridge.md)
