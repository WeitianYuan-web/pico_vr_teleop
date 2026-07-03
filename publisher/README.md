# teleop_realsense_publisher

ROS2 发布者节点，发布以下话题：

- `/camera_f/color/image_raw` (`sensor_msgs/Image`, `rgb8`)
- `/camera_l/color/image_raw` (`sensor_msgs/Image`, `rgb8`)
- `/camera_r/color/image_raw` (`sensor_msgs/Image`, `rgb8`)
- `/puppet/joint_left` (`sensor_msgs/JointState`)
- `/puppet/joint_right` (`sensor_msgs/JointState`)
- `/puppet/end_pose_left` (`geometry_msgs/PoseStamped`)
- `/puppet/end_pose_right` (`geometry_msgs/PoseStamped`)
- `/puppet/hand_left` (`sensor_msgs/JointState`)
- `/puppet/hand_right` (`sensor_msgs/JointState`)

其中 3 路相机话题发布 RealSense 实时彩色图像；`/puppet/*` 话题默认监听本机 UDP `17981`，
接收 `control` 或 `webxr_test` 遥操作脚本上报的臂/手状态后发布真实数据。
若超过 `state_stale_timeout_s` 未收到新数据，则发布零位占位。

## 依赖

- ROS2 Python 环境（`rclpy`, `sensor_msgs`, `geometry_msgs`）
- `pyrealsense2`

## 运行

```bash
cd /home/b0106/pico_test/pico_vr_teleop
python3 publisher/teleop_realsense_publisher.py
```

可选参数（ROS 参数）：

```bash
python3 publisher/teleop_realsense_publisher.py --ros-args \
  -p camera_width:=640 \
  -p camera_height:=480 \
  -p camera_fps:=30 \
  -p placeholder_hz:=120.0 \
  -p state_udp_host:=127.0.0.1 \
  -p state_udp_port:=17981 \
  -p state_stale_timeout_s:=1.0 \
  -p camera_f_serial:=<front_serial> \
  -p camera_l_serial:=<left_serial> \
  -p camera_r_serial:=<right_serial>
```

若不手动指定序列号，节点会自动按设备序列号排序后依次分配到 `f/l/r`。

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
