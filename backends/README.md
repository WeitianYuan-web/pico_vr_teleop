# backends/

三机械臂后端。入口在 `entrypoints/`，共用逻辑在 `common/`，厂商大包在 `third_party/` 或本机放置。

| 后端 | 随仓代码 | 需本机另放 |
|------|----------|------------|
| piper | `piper/teleop/` | Inspire（可选）→ `third_party/InspireHandSDK_Y/` |
| jaka | `jaka/sdk/`、`jaka/tcp_ip/` | 厂商包 → `jaka/20260104145805A007/` |
| g1 | `g1/` | `unitree_sdk2_python` + CycloneDDS |

跨设备清单见仓库根目录 [DEPENDENCIES.md](../DEPENDENCIES.md)。
