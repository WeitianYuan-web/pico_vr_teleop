# 07 · Configuration Reference

> Audience: Deployers / Developers

This chapter summarizes the three core files under `configs/config/` and the REST API endpoints. Changes require a **gateway restart** to take effect (some runtime items are hot-changeable via API, as noted).

## 7.1 `gateway.yaml` (main config)

### Basics
| Field | Default | Meaning |
|-------|---------|---------|
| `version` | `"V2.0.2"` | Version shown after the console title |
| `listen_host` | `0.0.0.0` | HTTP listen address |
| `listen_port` | `8080` | HTTP listen port (change the port here) |
| `end_tools_dir` | `configs/end_tools` | Hand package root |
| `logs_dir` | `logs` | Log directory |

### Serial probe
| Field | Default | Meaning |
|-------|---------|---------|
| `probe_interval_sec` | `2.0` | Serial poll interval (s) |
| `layout_confirm_count` | `2` | Debounce confirm count for layout changes |
| `serial_probe_fail_threshold` | `2` | Keep bound topology until N failures |
| `probe_exclude_ports` | `[]` | Ports never probed (put dexterous-hand RS485 ports here) |
| `startup_delay_sec` | `2.0` | Startup delay |
| `exo_recovery_interval_sec` | `5.0` | Min interval for auto-restart after exo drop |

### Hands & devices
| Field | Default | Meaning |
|-------|---------|---------|
| `hand_choose` | `[Inspire_RH56F2]` | Hands to auto-apply (Web Apply writes this) |
| `device_ids.left` | `8` | Left exoskeleton device ID |
| `device_ids.right` | `12` | Right exoskeleton device ID |
| `udp_allowed_ips` | `[]` | Wireless IP whitelist (up to 2) |

### WebSocket
| Field | Meaning |
|-------|---------|
| `websocket.max_fps` | Push throttle; 0 = no throttle (forward on receive) |
| `websocket.default_streams` | Default subscriptions with no hand (global streams) |
| `websocket.default_streams_by_side` | Default subscriptions by exo side (left/right/both) |

### bundle (runtime paths)
`bundle.prefix / python / python_lib / python_site / pythonpath / src / end_tools / install_lib / zenoh_lib / zenoh_config / exo_tf_bin / exo_tf_udp_bin / transform_bin`: all expanded from `{root}`, used by `bundle-env.sh` and subprocess commands. Usually no need to edit.

### commands (subprocess templates)
Placeholders expanded by `config_loader`:
| Command | Target |
|---------|--------|
| `exo_tf` | `bundle/install/bin/exo_tf_comm` (wired) |
| `exo_tf_udp` | `exo_tf_udp_comm {udp_bind_ip} {udp_port}` (wireless) |
| `transform` | `tf_transform_comm {end_tools} {hand}` |
| `controller_left` / `controller_right` | `run_control_v2_3.py {end_tools} {hand} controller_v2_3_{side}.yml` |

### streams / publish_streams (Zenoh data plane)
| Field | Meaning |
|-------|---------|
| `streams_config.hand_stream_id_format` | `"{id}.{hand}"`, ID expansion for `scope: hand` streams |
| `streams[]` | Subscribed streams: `id / scope(global\|hand) / topic / type`; hand streams get a namespace |
| `publish_streams[]` | Published streams: e.g. `io_esk.vibration_feedback` (Float64MultiArray, len 10) |

Scope expansion example (hand `Inspire_RH56F2`):
| scope | Expansion | Actual Zenoh key |
|-------|-----------|------------------|
| `global` | none | `io_esk/joint_data` |
| `hand` | `{id}.{hand}` | `io_align/Inspire_RH56F2/tf_hand` |
| `hand`+`pose_frame` | namespaced + frame suffix | `io_align/Inspire_RH56F2/poses_left_hand_ee_link` |

### Wireless (provision / UDP / heartbeat)
```yaml
wifi_provision:            # provisioning defaults (Web "Save network" writes this)
  ssid: IO_2.4G_LSFWN
  password: minnanoIO
  return_ip: 10.42.0.1     # return IP = router/gateway address
udp_probe:
  bind_ip: 10.42.0.2
  port: 8888
  listen_ms: 1000
  probe_retries: 2
  retry_gap_sec: 1.0
  fail_threshold: 3
  require_local_network: true   # require a local address in bind_ip's /24
wifi_heartbeat:
  port: 8889
  bind_ip: ""             # empty = 0.0.0.0
  alive_timeout_ms: 3000
  scan_listen_ms: 2000
  log_max_entries: 200
```

## 7.2 `topics.yaml` (exoskeleton topic mapping)
```yaml
exo_tf_publisher:
  tf_topic: io_fusion/tf_exoskeleton
  joint_state_topic: io_esk/joint_data
  timer_frequency: 120                 # exoskeleton publish rate Hz
  urdf_path: configs/exoskeleton_urdf/description/blender_human_skeleton_v5.urdf
```
`timer_frequency` can be hot-changed via `POST /api/v1/runtime/frequency` (writes yaml and restarts exo_tf).

## 7.3 `zenoh.json5` (Zenoh networking)
```json5
{
  mode: "peer",
  scouting: { multicast: { enabled: true, interface: "lo" } },
  listen: { endpoints: ["tcp/127.0.0.1:0"] }   // loopback only, dynamic port
}
```
Defaults to loopback-only local communication; cross-host networking requires editing this file (carefully).

## 7.4 REST API endpoints (`/api/v1` prefix)

| Method | Path | Purpose |
|--------|------|---------|
| GET | `/bootstrap` | Headless startup report (hands, topology, port scan) |
| GET | `/probe` | Re-probe now (serial or UDP) |
| GET | `/status` | Runtime snapshot (exo, hands, wireless, subprocesses, Zenoh bridge) |
| GET | `/hands/configs` | Available hands |
| GET | `/hands/dirs` | `end_tools` directory names (upload clash check) |
| POST | `/hands/configs/upload` | Upload a hand package (multipart) |
| POST | `/hands/select` | Select hands (empty array clears and stops subprocesses) |
| POST | `/runtime/frequency` | Change exo_tf publish rate and restart |
| GET | `/streams` | Stream catalog (subscribe + publish) |
| GET | `/wifi/heartbeat/devices` | Wireless heartbeat online IPs |
| GET | `/wifi/config` | Read provisioning defaults |
| PUT | `/wifi/config` | Save provisioning defaults to gateway.yaml |
| POST | `/wifi/provision` | ESP-Touch provisioning |
| GET | `/visualization/config` | Viz page URDF resource URLs |
| GET | `/visualization/urdf/exo` | Exoskeleton URDF |
| GET | `/visualization/urdf/{hand}?side=left\|right` | Dexterous-hand URDF |

> The data plane (live subscribe/publish) uses `WS /ws`, not REST.
>
> Note: `/api/v1/control/sync/start|stop`, `/udp/start|stop` seen in test scripts are **not implemented** in the current backend (legacy/planned); wireless UDP is managed automatically by the orchestrator.

Interactive docs: `http://<host>:8080/docs`.

---

Next: [08 Troubleshooting](./08-troubleshooting.md)
