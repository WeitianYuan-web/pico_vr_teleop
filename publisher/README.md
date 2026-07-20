# teleop_realsense_publisher

ROS2 发布者节点，发布以下话题：

- `/camera_f/color/image_raw` (`sensor_msgs/Image`, `rgb8`) **默认**
- `/camera_l/color/image_raw` (`sensor_msgs/Image`, `rgb8`) **默认**
- `/camera_r/color/image_raw` (`sensor_msgs/Image`, `rgb8`) **默认**
- `/camera_*/color/image_raw/compressed` (`sensor_msgs/CompressedImage`, jpeg) 可选
- `/puppet/joint_left` (`sensor_msgs/JointState`)
- `/puppet/joint_right` (`sensor_msgs/JointState`)
- `/puppet/end_pose_left` (`geometry_msgs/PoseStamped`)
- `/puppet/end_pose_right` (`geometry_msgs/PoseStamped`)
- `/puppet/hand_left` (`sensor_msgs/JointState`)
- `/puppet/hand_right` (`sensor_msgs/JointState`)

其中 3 路相机话题发布 RealSense 实时彩色图像，每路相机使用独立采集线程发布；`/puppet/*` 话题默认监听本机 UDP `17981`，
接收 `backends` 或 `webxr` 遥操作脚本上报的臂/手状态后发布真实数据。
若超过 `state_stale_timeout_s` 未收到新数据，则发布零位占位。

## 依赖

- ROS2 Python 环境（`rclpy`, `sensor_msgs`, `geometry_msgs`）
- `pyrealsense2`
- `numpy`；若开启 compressed 还需 `opencv-python`

## 运行

```bash
cd /home/b0106/pico_test/pico_vr_teleop
source /opt/ros/jazzy/setup.bash
source .venv/bin/activate

python3 publisher/teleop_realsense_publisher.py
```

可选参数（ROS 参数）：

```bash
python3 publisher/teleop_realsense_publisher.py --ros-args \
  -p camera_width:=640 \
  -p camera_height:=480 \
  -p camera_fps:=30 \
  -p placeholder_hz:=30.0 \
  -p publish_raw_image:=true \
  -p publish_compressed_image:=false \
  -p jpeg_quality:=80 \
  -p state_udp_host:=127.0.0.1 \
  -p state_udp_port:=17981 \
  -p state_stale_timeout_s:=1.0 \
  -p camera_f_serial:=<front_serial> \
  -p camera_l_serial:=<left_serial> \
  -p camera_r_serial:=<right_serial>
```

若不手动指定序列号，节点会自动按设备序列号排序后依次分配到 `f/l/r`。

### 帧率相关（三路 640×480 目标约 30Hz）

| 参数 / 行为 | 说明 |
|---|---|
| `camera_fps` | 驱动目标帧率，默认 30 |
| `publish_raw_image` | 默认 **true**，话题不变，无损 `rgb8` |
| `publish_compressed_image` | 默认 **false**；可选 JPEG 小包备份话题 |
| `placeholder_hz` | `/puppet/*` 状态定时器频率，默认 **30** |
| 采集侧 | `frames_queue_size=1`、`poll_for_frames` 取最新、AE 优先帧率 |
| 执行器 | `MultiThreadedExecutor(num_threads=4)` |
| 本机传输 | Fast DDS 保留内置发现 + 加大 SHM（默认 SHM≈512KB < 单帧≈900KB） |

**为何订阅端会只有十几 Hz？**  
发布端内部一直是 ~30Hz。640×480 `rgb8` 单帧约 900KB，默认 Fast DDS SHM segment 只有约 512KB，大图会被迫走 UDP 分片并丢帧。`fastdds_local_image.xml` 加大 SHM，并保留内置传输，因此普通 `ros2 topic list` 仍能发现话题；话题名、消息类型、QoS 和图像参数均不改变。

测订阅端真实 FPS（QoS 匹配）：

```bash
python3 publisher/topic_hz_best_effort.py /camera_f/color/image_raw
```

发布端内部统计：

```bash
tail -f logs/publisher.log | grep 发布帧率
```

可选开启 JPEG（有损、另开话题，不替换 raw）：

```bash
python3 publisher/teleop_realsense_publisher.py --ros-args \
  -p publish_compressed_image:=true
```

## 与遥操作联动

终端 1：启动 ROS 发布者

```bash
python3 publisher/teleop_realsense_publisher.py
```

终端 2：启动双臂双手遥操作（默认已开启状态上报）

```bash
./scripts/run_dual_arm_dual_hand.sh
```

仅双臂（无灵巧手）时：

```bash
./scripts/run_vr_teleop.sh --publish-state
```

上报协议见 `publisher/teleop_state_bridge.py`（UDP JSON）。
