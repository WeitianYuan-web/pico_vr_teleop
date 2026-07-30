#!/usr/bin/env bash
# 从本机一键控天轶左臂（推荐：在机器人 Humble 上执行点动；可选 --local-dds 试本机发 Topic）
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROBOT_HOST="${TIANYEE_HOST:-192.168.41.1}"
ROBOT_USER="${TIANYEE_USER:-ubuntu}"
PASS="${TIANYEE_SSH_PASS:-123}"
MODE="robot" # robot | local
DELTA_DEG=3
HOLD=3

while [[ $# -gt 0 ]]; do
  case "$1" in
    --local-dds) MODE=local; shift ;;
    --delta-deg) DELTA_DEG="$2"; shift 2 ;;
    --hold) HOLD="$2"; shift 2 ;;
    *) echo "未知参数: $1"; exit 1 ;;
  esac
done

prep_and_jog_robot() {
  local remote_script
  remote_script=$(mktemp)
  cat > "${remote_script}" <<EOF
#!/bin/bash
set +u
export ROS_HOME=/tmp/xarm_run/ros_home
mkdir -p "\$ROS_HOME" /tmp/xarm_run
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2ws/install/setup.bash
source /home/ubuntu/XARM/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/data/param/dds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
if ! pgrep -f "tianyi2.launch.py" >/dev/null; then
  echo "[错误] XARM 未运行"; exit 2
fi
if ! pgrep -f "body_control body.launch" >/dev/null; then
  echo "[错误] body_control 未运行（需遥控器 A 键或手动启动）"; exit 3
fi
ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"
ros2 control auto_switch_mode --enable || true
python3 - <<'PY'
import math, time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
NAMES=["shoulder_pitch_l_joint","shoulder_roll_l_joint","shoulder_yaw_l_joint","elbow_pitch_l_joint","elbow_yaw_l_joint","wrist_pitch_l_joint","wrist_roll_l_joint"]
DELTA=math.radians(float("${DELTA_DEG}"))
HOLD=float("${HOLD}")
rclpy.init(); n=Node("robot_jog"); got={}
def cb(m):
  for a,b in zip(m.name,m.position):
    if a in NAMES: got[a]=float(b)
n.create_subscription(JointState,"/joint_states",cb,50)
pub=n.create_publisher(Float64MultiArray,"/jointspace_commands_L",10)
t0=time.time()
while time.time()-t0<5 and len(got)<7:
  rclpy.spin_once(n,timeout_sec=0.1)
q0=[got[k] for k in NAMES]
q1=list(q0); q1[3]=q0[3]+DELTA
print("q0", [round(x,4) for x in q0])
print("jog elbow deg", round(math.degrees(q0[3]),2), "->", round(math.degrees(q1[3]),2))
def stream(q, sec):
  msg=Float64MultiArray(); msg.data=q
  end=time.time()+sec
  while time.time()<end:
    pub.publish(msg); rclpy.spin_once(n,timeout_sec=0.05); time.sleep(0.05)
stream(q1, HOLD)
got.clear(); t0=time.time()
while time.time()-t0<2: rclpy.spin_once(n,timeout_sec=0.1)
print("MID_delta_deg", round(math.degrees(got.get(NAMES[3],q0[3])-q0[3]),3))
stream(q0, HOLD)
got.clear(); t0=time.time()
while time.time()-t0<2: rclpy.spin_once(n,timeout_sec=0.1)
print("BACK_delta_deg", round(math.degrees(got.get(NAMES[3],q0[3])-q0[3]),3))
n.destroy_node(); rclpy.shutdown()
PY
ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: false}"
EOF
  python3 - "${ROBOT_USER}@${ROBOT_HOST}" "${PASS}" "${remote_script}" <<'PY'
import pexpect, sys
host, password, local_script = sys.argv[1], sys.argv[2], sys.argv[3]
remote = "/tmp/xarm_run/jog_from_pc.sh"
child = pexpect.spawn(
    f"scp -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
    f"-o PubkeyAuthentication=no {local_script} {host}:{remote}",
    encoding="utf-8", timeout=60,
)
child.expect(["password:", "Password:"])
child.sendline(password)
child.expect(pexpect.EOF, timeout=60)
child = pexpect.spawn(
    f"ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password "
    f"-o PubkeyAuthentication=no {host} bash {remote}",
    encoding="utf-8", timeout=180, maxread=400000,
)
child.expect(["password:", "Password:"])
child.sendline(password)
child.expect(pexpect.EOF, timeout=180)
print(child.before)
sys.exit(0 if child.exitstatus in (0, None) else (child.exitstatus or 1))
PY
  rm -f "${remote_script}"
}

jog_local_dds() {
  MSG_PREFIX="${PROJECT_DIR}/third_party/tianyee_ros_ws/install/eai_manipulator_msgs"
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  export AMENT_PREFIX_PATH="${MSG_PREFIX}:${AMENT_PREFIX_PATH}"
  export PYTHONPATH="${MSG_PREFIX}/lib/python3.12/site-packages:${PYTHONPATH:-}"
  export LD_LIBRARY_PATH="${MSG_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export ROS_DOMAIN_ID=0
  unset FASTRTPS_DEFAULT_PROFILES_FILE
  export ROS_HOME="${PROJECT_DIR}/logs/ros_home_local"
  mkdir -p "${ROS_HOME}"
  # enable on robot first
  prep_and_jog_robot() { :; }
  python3 - "${ROBOT_USER}@${ROBOT_HOST}" "${PASS}" <<'PY'
import pexpect, sys
host, password = sys.argv[1], sys.argv[2]
cmd = r'''bash -lc 'set +u; export ROS_HOME=/tmp/xarm_run/ros_home; mkdir -p $ROS_HOME; source /opt/ros/humble/setup.bash; source /home/ubuntu/ros2ws/install/setup.bash; source /home/ubuntu/XARM/install/setup.bash; export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/data/param/dds_profile.xml; export RMW_IMPLEMENTATION=rmw_fastrtps_cpp; ros2 service call /EAIHardware/set_arm_enable std_srvs/srv/SetBool "{data: true}"; ros2 service call /EAIHardware/set_arm_mode eai_manipulator_msgs/srv/Mode "{mode: 3}"; ros2 control auto_switch_mode --enable' '''
child = pexpect.spawn(f"ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no {host} {cmd}", encoding="utf-8", timeout=90)
child.expect(["password:", "Password:"])
child.sendline(password)
child.expect(pexpect.EOF, timeout=90)
print(child.before)
PY
  python3 "${PROJECT_DIR}/scripts/local_tianyee_arm_jog.py" --no-enable --delta-deg "${DELTA_DEG}" --hold "${HOLD}"
  python3 - "${ROBOT_USER}@${ROBOT_HOST}" "${PASS}" <<'PY'
import pexpect, sys
host, password = sys.argv[1], sys.argv[2]
child = pexpect.spawn(f"ssh -o StrictHostKeyChecking=no -o PreferredAuthentications=password -o PubkeyAuthentication=no {host} bash /tmp/xarm_run/disable_arm.sh", encoding="utf-8", timeout=60)
child.expect(["password:", "Password:"])
child.sendline(password)
child.expect(pexpect.EOF, timeout=60)
print(child.before)
PY
}

echo "[mode=${MODE}] delta=${DELTA_DEG}deg hold=${HOLD}s"
if [[ "${MODE}" == "local" ]]; then
  echo "[警告] Jazzy↔Humble DDS 可能不稳定；优先用默认 robot 模式"
  jog_local_dds
else
  prep_and_jog_robot
fi
