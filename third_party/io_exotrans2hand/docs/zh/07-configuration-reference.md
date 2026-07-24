# 07 · 配置参考

> 适用读者：部署工程师 / 二次开发者

本章汇总 `configs/config/` 下三个核心配置文件与 REST API 端点。修改配置后需 **重启网关** 生效（部分运行时项可经 API 热改，见文中标注）。

## 7.1 `gateway.yaml`（网关主配置）

### 基础
| 字段 | 默认 | 说明 |
|------|------|------|
| `version` | `"V2.0.2"` | Web 控制台标题后显示的版本号 |
| `listen_host` | `0.0.0.0` | HTTP 监听地址 |
| `listen_port` | `8080` | HTTP 监听端口（改端口在此） |
| `end_tools_dir` | `configs/end_tools` | 手型包根目录 |
| `logs_dir` | `logs` | 日志目录 |

### 串口探测
| 字段 | 默认 | 说明 |
|------|------|------|
| `probe_interval_sec` | `2.0` | 串口轮询间隔（秒） |
| `layout_confirm_count` | `2` | 布局变化防抖确认次数 |
| `serial_probe_fail_threshold` | `2` | 失败 N 次前保留已绑定拓扑 |
| `probe_exclude_ports` | `[]` | 永不探测的串口列表（灵巧手 RS485 口写这里） |
| `startup_delay_sec` | `2.0` | 启动延迟 |
| `exo_recovery_interval_sec` | `5.0` | exo 掉线后自动补启最小间隔 |

### 手型与设备
| 字段 | 默认 | 说明 |
|------|------|------|
| `hand_choose` | `[Inspire_RH56F2]` | 待自动应用的手型列表（Web「应用」会写入） |
| `device_ids.left` | `8` | 左手外骨骼设备 ID |
| `device_ids.right` | `12` | 右手外骨骼设备 ID |
| `udp_allowed_ips` | `[]` | 无线 IP 白名单（最多 2 个） |

### WebSocket
| 字段 | 说明 |
|------|------|
| `websocket.max_fps` | 推送节流；0 = 不节流（收到即转发） |
| `websocket.default_streams` | 无手型时默认订阅（global 流） |
| `websocket.default_streams_by_side` | 按外骨骼侧（left/right/both）默认订阅 |

### bundle（运行时路径）
`bundle.prefix / python / python_lib / python_site / pythonpath / src / end_tools / install_lib / zenoh_lib / zenoh_config / exo_tf_bin / exo_tf_udp_bin / transform_bin`：均以 `{root}` 展开，供 `bundle-env.sh` 与子进程命令使用。一般无需改动。

### commands（子进程命令模板）
占位符由 `config_loader` 展开：
| 命令 | 目标 |
|------|------|
| `exo_tf` | `bundle/install/bin/exo_tf_comm`（有线） |
| `exo_tf_udp` | `exo_tf_udp_comm {udp_bind_ip} {udp_port}`（无线） |
| `transform` | `tf_transform_comm {end_tools} {hand}` |
| `controller_left` / `controller_right` | `run_control_v2_3.py {end_tools} {hand} controller_v2_3_{side}.yml` |

### streams / publish_streams（Zenoh 数据面）
| 字段 | 说明 |
|------|------|
| `streams_config.hand_stream_id_format` | `"{id}.{hand}"`，`scope: hand` 流的 ID 展开规则 |
| `streams[]` | 订阅流：`id / scope(global\|hand) / topic / type`；`hand` 流按手型加命名空间 |
| `publish_streams[]` | 发布流：如 `io_esk.vibration_feedback`（Float64MultiArray，长度 10） |

scope 展开示例（手型 `Inspire_RH56F2`）：
| scope | 展开 | 实际 Zenoh key |
|-------|------|----------------|
| `global` | 不展开 | `io_esk/joint_data` |
| `hand` | `{id}.{hand}` | `io_align/Inspire_RH56F2/tf_hand` |
| `hand`+`pose_frame` | namespaced + frame 后缀 | `io_align/Inspire_RH56F2/poses_left_hand_ee_link` |

### 无线（配网 / UDP / 心跳）
```yaml
wifi_provision:            # 配网默认参数（Web「保存网络」写入）
  ssid: IO_2.4G_LSFWN
  password: minnanoIO
  return_ip: 10.42.0.1     # 回调 IP = 路由器/网关地址
udp_probe:
  bind_ip: 10.42.0.2
  port: 8888
  listen_ms: 1000
  probe_retries: 2
  retry_gap_sec: 1.0
  fail_threshold: 3
  require_local_network: true   # 要求本机存在 bind_ip 所在 /24 网段
wifi_heartbeat:
  port: 8889
  bind_ip: ""             # 空 = 0.0.0.0
  alive_timeout_ms: 3000
  scan_listen_ms: 2000
  log_max_entries: 200
```

## 7.2 `topics.yaml`（外骨骼话题映射）
```yaml
exo_tf_publisher:
  tf_topic: io_fusion/tf_exoskeleton
  joint_state_topic: io_esk/joint_data
  timer_frequency: 120                 # 外骨骼数据发布频率 Hz
  urdf_path: configs/exoskeleton_urdf/description/blender_human_skeleton_v5.urdf
```
`timer_frequency` 可经 `POST /api/v1/runtime/frequency` 热改（会写 yaml 并重启 exo_tf）。

## 7.3 `zenoh.json5`（Zenoh 组网）
```json5
{
  mode: "peer",
  scouting: { multicast: { enabled: true, interface: "lo" } },
  listen: { endpoints: ["tcp/127.0.0.1:0"] }   // 仅 loopback，动态端口
}
```
默认仅本机 loopback 隔离通信；跨机组网需调整此文件（谨慎）。

## 7.4 REST API 端点（`/api/v1` 前缀）

| 方法 | 路径 | 作用 |
|------|------|------|
| GET | `/bootstrap` | headless 启动报告（手型、拓扑、端口扫描） |
| GET | `/probe` | 立即重探测（串口或 UDP） |
| GET | `/status` | 运行时快照（exo、手型、无线、子进程、Zenoh 桥） |
| GET | `/hands/configs` | 可用手型列表 |
| GET | `/hands/dirs` | `end_tools` 目录名（上传撞名检测） |
| POST | `/hands/configs/upload` | 上传手型包（multipart） |
| POST | `/hands/select` | 选手型（空数组=清空并停子进程） |
| POST | `/runtime/frequency` | 修改 exo_tf 发布频率并重启 |
| GET | `/streams` | 数据流目录（订阅 + 发布） |
| GET | `/wifi/heartbeat/devices` | 无线心跳在线 IP |
| GET | `/wifi/config` | 读配网默认参数 |
| PUT | `/wifi/config` | 保存配网默认参数到 gateway.yaml |
| POST | `/wifi/provision` | ESP-Touch 配网 |
| GET | `/visualization/config` | 可视化页 URDF 资源 URL |
| GET | `/visualization/urdf/exo` | 外骨骼 URDF |
| GET | `/visualization/urdf/{hand}?side=left\|right` | 灵巧手 URDF |

> 数据面（订阅/发布实时数据）走 `WS /ws`，不走 REST。
>
> 说明：测试脚本中出现的 `/api/v1/control/sync/start|stop`、`/udp/start|stop` 等在当前后端 **未实现**，属遗留/计划接口；无线 UDP 由编排器自动管理。

在线交互文档：`http://<主机IP>:8080/docs`。

---

下一步：[08 故障排查](./08-troubleshooting.md)
