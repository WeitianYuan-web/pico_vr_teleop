/**
 * 可视化页：外骨骼 / 灵巧手 URDF（本地 three + urdf-loader）与关节折线图。
 */
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import URDFLoader from "urdf-loader";

const CHART_LEN = 120;
/** 外骨骼视口：正上方俯视略带前倾，模型尽量铺满画面（网格可超出视口） */
const EXO_VIEW_FILL = 1.15;
const EXO_VIEW_OFFSET = { x: 0, y: 1, z: 0.34 };
/** 灵巧手视口取景距离倍数（大于外骨骼，模型在画面中更小） */
const HAND_VIEW_DIST_MUL = 3.6;
/** 左右分离单手 URDF 在场景中的横向间距（米） */
const HAND_SPLIT_OFFSET = 0.22;
const chartBuffers = {
  exoLeft: new Map(),
  exoRight: new Map(),
  exoRate: new Map(),
  handLeft: new Map(),
  handRight: new Map(),
  handLeftRate: new Map(),
  handRightRate: new Map(),
};
/** io_esk.vibration_feedback Float64MultiArray 固定 10 个末端，数值 0–10 */
const EXO_VIBRATION_CHANNELS = 10;
const EXO_VIBRATION_Y_MIN = 0;
const EXO_VIBRATION_Y_MAX = 10;
const FREQ_WINDOW_MS = 1000;
/** 3D URDF 固定输出频率（图表与频率统计仍用满格 WS 原始数据） */
const VIZ_OUTPUT_HZ = 30;
const VIZ_BUCKET_MS = 1000 / VIZ_OUTPUT_HZ;
const VIZ_RESAMPLE_BUFFER_MS = 1000;
const streamTickTimes = new Map();
/** 灵巧手关节图：缓冲全部关节以便图例完整着色标识 */
const HAND_SIDE_CHART_MAX_SERIES = 64;
/** 外骨骼每侧最多 21 个关节（right_* / left_*） */
const EXO_SIDE_CHART_MAX_SERIES = 21;
/** WS stream id → Ros key（与 gateway.yaml streams 一致） */
const STREAM_EXO_JOINT = "io_esk.joint_data";
const STREAM_EXO_VIBRATION = "io_esk.vibration_feedback";
const STREAM_HAND_CMD_LEFT = "io_teleop.joint_cmd_left";
const STREAM_HAND_CMD_RIGHT = "io_teleop.joint_cmd_right";
const STREAM_TOPIC = {
  [STREAM_EXO_JOINT]: "/io_esk/joint_data",
  [STREAM_EXO_VIBRATION]: "/io_esk/vibration_feedback",
  [STREAM_HAND_CMD_LEFT]: "/io_teleop/joint_cmd_finger_left",
  [STREAM_HAND_CMD_RIGHT]: "/io_teleop/joint_cmd_finger_right",
};
/** 最近一次振动反馈原始数组（meta 展示用） */
let lastExoVibrationData = null;
/** 最近一次收到 vibration_feedback 的时间（断流后清空图表） */
let exoVibrationLastRx = 0;
/** 超过该毫秒未收到振动流则清空柱状图，避免最后一帧残留 */
const VIBRATION_STALE_MS = 1500;
/** 频率图固定 Y 轴 0–150 Hz；auto=随数据动态缩放 */
const RATE_Y_FIXED = { min: 0, max: 150 };
/** 频率图 Y 轴刻度（含 120 Hz 档位，与 timer_frequency 默认一致） */
const RATE_Y_TICK_VALUES = [0, 30, 60, 90, 120, 150];
const rateYAxisModes = { exo: "fixed", handLeft: "fixed", handRight: "fixed" };
const RATE_YAXIS_RADIO_GROUPS = [
  { name: "exo-rate-yaxis", modeKey: "exo", stableKey: "exoRate" },
  { name: "hand-left-rate-yaxis", modeKey: "handLeft", stableKey: "handLeftRate" },
  { name: "hand-right-rate-yaxis", modeKey: "handRight", stableKey: "handRightRate" },
];
/** 灵巧手频率图例：左右分图，固定显示 joint_cmd_left / joint_cmd_right */
const HAND_RATE_LABEL = { left: STREAM_HAND_CMD_LEFT, right: STREAM_HAND_CMD_RIGHT };
/** io_esk.joint_data 按关节名分侧缓存（图表标注用） */
const exoJointCache = { left: null, right: null };
/** joint_cmd_left / joint_cmd_right 分侧缓存，合并后写入双手 URDF */
const handJointCmdCache = { left: null, right: null };
/** 每侧最近一次收到的 cmd 时间（用于断流后清空） */
const handCmdLastRx = { left: 0, right: 0 };
/** 清空缓存后仍用于把对应侧关节置 0 */
const lastHandJointNames = { left: [], right: [] };
/** 超过该毫秒未收到该侧 cmd 则视为断流 */
const HAND_CMD_STALE_MS = 1500;
/** 超过该毫秒未收到 joint_data 且双侧断连后清空外骨骼图表 */
const EXO_JOINT_STALE_MS = 2000;
/** 外骨骼是否仍连接（串口 endpoint 或 UDP 模式；用于断流判断与清空可视化） */
const exoPortConnected = { left: false, right: false };
/** 最近一次 gateway-status 快照（UDP 模式判断用） */
let lastGatewayStatus = null;
/** 灵巧手 3D：combined=单文件双手；split=左右各载一个 URDF */
let handVizState = {
  handId: "",
  mode: "",
  group: null,
  combined: null,
  left: null,
  right: null,
};
/** 灵巧手图表 EMA 平滑（仅显示，减轻 IK 指令高频抖动） */
const HAND_CHART_EMA = 0.22;
const chartYStable = {
  exoLeft: null,
  exoRight: null,
  exoRate: null,
  handLeft: null,
  handRight: null,
  handLeftRate: null,
  handRightRate: null,
};
/** 图表 Y 轴参考范围（由 URDF 关节限位汇总，与 3D 模型加载解耦） */
const urdfChartYLim = {
  exoLeft: null,
  exoRight: null,
  handLeft: null,
  handRight: null,
};

let vizReady = false;
let exoPreloaded = false;
let exoScene = null;
let handScene = null;
let exoRobot = null;
let vizConfig = null;

class UrdfViewport {
  constructor(container, options = {}) {
    this.container = container;
    this.floorGrid = !!options.floorGrid;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0xf1f2f1);
    const w = Math.max(container.clientWidth, 240);
    const h = Math.max(container.clientHeight, 200);
    this.camera = new THREE.PerspectiveCamera(45, w / h, 0.01, 50);
    this.camera.position.set(0.8, 0.6, 0.9);
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.setSize(w, h);
    container.innerHTML = "";
    container.appendChild(this.renderer.domElement);
    const amb = new THREE.AmbientLight(0xffffff, 0.65);
    const dir = new THREE.DirectionalLight(0xffffff, 0.85);
    dir.position.set(2, 3, 4);
    this.scene.add(amb, dir);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this._gridBaseSize = 1;
    this.grid = new THREE.GridHelper(this._gridBaseSize, 16, 0xa9b4b3, 0xd8dedd);
    if (!this.floorGrid) {
      this.grid.scale.setScalar(0.6);
    }
    this.scene.add(this.grid);
    this.robot = null;
    /** 递增后丢弃过期的 loadUrdf，避免快速切换手型时多个 URDF 叠在场景里 */
    this._urdfLoadGen = 0;
    this._raf = 0;
    this._tick = this._tick.bind(this);
    this._tick();
    this._ro = new ResizeObserver(() => this.resize());
    this._ro.observe(container);
  }

  resize() {
    const w = Math.max(this.container.clientWidth, 120);
    const h = Math.max(this.container.clientHeight, 120);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h);
  }

  _tick() {
    this._raf = requestAnimationFrame(this._tick);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  _clearRobotFromScene() {
    if (this.robot) {
      this.scene.remove(this.robot);
      this.robot = null;
    }
    for (const child of [...this.scene.children]) {
      if (child instanceof THREE.GridHelper || child.isLight) continue;
      if (child.joints) this.scene.remove(child);
    }
  }

  /** 取消进行中的 URDF 加载（切换手型时调用） */
  invalidateUrdfLoads() {
    this._urdfLoadGen += 1;
    this._clearRobotFromScene();
  }

  async _parseUrdfRobot(urdfUrl, packagePrefix, loadGen) {
    if (!urdfUrl) return null;
    const loader = new URDFLoader();
    loader.packages = { "": packagePrefix || "/" };
    const res = await fetch(urdfUrl);
    if (loadGen !== this._urdfLoadGen) return null;
    if (!res.ok) throw new Error(`URDF HTTP ${res.status}`);
    const robot = loader.parse(await res.text());
    if (loadGen !== this._urdfLoadGen) return null;
    if (!robot) throw new Error("URDF parse failed");
    robot.rotation.x = -Math.PI / 2;
    return robot;
  }

  async loadUrdf(urdfUrl, packagePrefix, statusEl, distMul = 2.2, viewOffset = null, fillRatio = null) {
    const loadGen = ++this._urdfLoadGen;
    this._clearRobotFromScene();
    if (!urdfUrl) {
      if (statusEl) statusEl.textContent = t("viz.noUrdf");
      return null;
    }
    if (statusEl) statusEl.textContent = t("viz.loading");

    try {
      const robot = await this._parseUrdfRobot(urdfUrl, packagePrefix, loadGen);
      if (loadGen !== this._urdfLoadGen) return null;
      if (!robot) throw new Error("URDF parse failed");
      this.scene.add(robot);
      this.robot = robot;
      this._frameModel(robot, distMul, viewOffset, fillRatio);
      if (statusEl) statusEl.textContent = "";
      return robot;
    } catch (e) {
      if (loadGen !== this._urdfLoadGen) return null;
      console.warn("URDF load failed", urdfUrl, e);
      if (statusEl) {
        statusEl.textContent = `${t("viz.loadFailed")}: ${e.message || e}`;
      }
      return null;
    }
  }

  /** 分离单手 URDF：模型底面贴到网格平面（y=0） */
  _alignRobotOntoGrid(robot) {
    robot.updateMatrixWorld(true);
    const box = new THREE.Box3().setFromObject(robot);
    if (!box.isEmpty()) robot.position.y -= box.min.y;
  }

  /** 分离单手 URDF 右手：与左手手心相对，且保持掌面朝上 */
  _orientSplitHandRight(robot) {
    robot.rotation.x = -Math.PI / 2;
    robot.scale.x = -1;
  }

  /** 灵巧手：双手合一单文件，或左右分离各载一个 URDF */
  async loadHandUrdfDisplay(spec, distMul = HAND_VIEW_DIST_MUL) {
    const loadGen = ++this._urdfLoadGen;
    this._clearRobotFromScene();
    if (!spec) return null;

    const group = new THREE.Group();
    group.name = "hand-urdf-group";
    const out = { mode: spec.mode, group, combined: null, left: null, right: null };

    try {
      if (spec.mode === "combined") {
        const url = spec.combinedUrl || spec.leftUrl || spec.rightUrl;
        const pkg = spec.combinedPkg || spec.leftPkg || spec.rightPkg;
        const robot = await this._parseUrdfRobot(url, pkg, loadGen);
        if (!robot) return null;
        group.add(robot);
        out.combined = robot;
      } else {
        if (spec.leftUrl) {
          const robot = await this._parseUrdfRobot(spec.leftUrl, spec.leftPkg, loadGen);
          if (loadGen !== this._urdfLoadGen) return null;
          if (robot) {
            robot.position.x = -HAND_SPLIT_OFFSET;
            this._alignRobotOntoGrid(robot);
            group.add(robot);
            out.left = robot;
          }
        }
        if (spec.rightUrl) {
          const robot = await this._parseUrdfRobot(spec.rightUrl, spec.rightPkg, loadGen);
          if (loadGen !== this._urdfLoadGen) return null;
          if (robot) {
            this._orientSplitHandRight(robot);
            robot.position.x = HAND_SPLIT_OFFSET;
            this._alignRobotOntoGrid(robot);
            group.add(robot);
            out.right = robot;
          }
        }
      }
      if (loadGen !== this._urdfLoadGen) return null;
      if (!group.children.length) return null;
      this.scene.add(group);
      this.robot = group;
      this._frameModel(group, distMul);
      return out;
    } catch (e) {
      if (loadGen !== this._urdfLoadGen) return null;
      console.warn("hand URDF load failed", e);
      return null;
    }
  }

  _frameModel(obj, distMul = 2.2, viewOffset = null, fillRatio = null) {
    const view = viewOffset || { x: 0.6, y: 0.5, z: 0.7 };
    const box = new THREE.Box3().setFromObject(obj);
    if (box.isEmpty()) {
      this.controls.target.set(0, 0, 0);
      this.camera.position.set(view.x, view.y, view.z);
      return;
    }
    const size = box.getSize(new THREE.Vector3());
    const center = box.getCenter(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z, 0.05);
    const dist =
      fillRatio != null && viewOffset
        ? this._distForViewportFill(box, view, fillRatio)
        : maxDim * distMul;
    this.controls.target.copy(center);
    this.camera.position.set(
      center.x + dist * view.x,
      center.y + dist * view.y,
      center.z + dist * view.z
    );
    this.controls.update();
    if (this.floorGrid && viewOffset) {
      this._alignFloorGrid(box, maxDim);
    }
  }

  /** 按包围盒与 FOV 计算相机距离，使模型在画面中达到目标占比 */
  _distForViewportFill(box, view, fillRatio) {
    const center = new THREE.Vector3();
    box.getCenter(center);
    const viewDir = new THREE.Vector3(view.x, view.y, view.z).normalize();
    const corners = [
      new THREE.Vector3(box.min.x, box.min.y, box.min.z),
      new THREE.Vector3(box.min.x, box.min.y, box.max.z),
      new THREE.Vector3(box.min.x, box.max.y, box.min.z),
      new THREE.Vector3(box.min.x, box.max.y, box.max.z),
      new THREE.Vector3(box.max.x, box.min.y, box.min.z),
      new THREE.Vector3(box.max.x, box.min.y, box.max.z),
      new THREE.Vector3(box.max.x, box.max.y, box.min.z),
      new THREE.Vector3(box.max.x, box.max.y, box.max.z),
    ];
    const rel = new THREE.Vector3();
    const along = new THREE.Vector3();
    let maxPerp = 0;
    for (const c of corners) {
      rel.copy(c).sub(center);
      along.copy(viewDir).multiplyScalar(rel.dot(viewDir));
      rel.sub(along);
      maxPerp = Math.max(maxPerp, rel.length());
    }
    maxPerp = Math.max(maxPerp, 0.05);
    const f = Math.min(Math.max(fillRatio, 0.55), 1.25);
    const vFov = (this.camera.fov * Math.PI) / 180;
    const hFov = 2 * Math.atan(Math.tan(vFov / 2) * this.camera.aspect);
    const distV = maxPerp / Math.tan((vFov / 2) * f);
    const distH = maxPerp / Math.tan((hFov / 2) * f);
    return Math.max(distV, distH, 0.05);
  }

  /** 地面网格与模型共面（XZ 平面），避免视线法向网格产生歪斜参考线 */
  _alignFloorGrid(box, maxDim) {
    if (!this.grid) return;
    const center = new THREE.Vector3();
    box.getCenter(center);
    const spanX = box.max.x - box.min.x;
    const spanZ = box.max.z - box.min.z;
    const size = Math.max(Math.max(spanX, spanZ) * 1.05, maxDim * 1.1, 0.35);
    this.grid.position.set(center.x, box.min.y - maxDim * 0.015, center.z);
    this.grid.rotation.set(0, 0, 0);
    this.grid.quaternion.identity();
    this.grid.scale.setScalar(size / this._gridBaseSize);
  }

  setJointPositions(names, positions) {
    if (!this.robot || !names?.length) return;
    for (let i = 0; i < names.length; i++) {
      const v = positions[i];
      if (v == null || Number.isNaN(v)) continue;
      try {
        this.robot.setJointValue(names[i], v);
      } catch (_) {
        /* 忽略未知关节 */
      }
    }
  }
}

function knownHandIds() {
  const pick = getCurrentHandId();
  const fromCfg = [
    ...(vizConfig?.selected_hands || []),
    ...((vizConfig?.hands || []).map((h) => h.id)),
  ];
  return [...new Set([pick, ...fromCfg].filter(Boolean))];
}

/** 手型流 id 格式 {base_id}.{hand}，base_id 本身可含点号，故用已知 hand 后缀匹配 */
function splitStreamId(stream) {
  if (!stream) return { base: "", hand: null };
  for (const hand of knownHandIds()) {
    const suffix = `.${hand}`;
    if (stream.endsWith(suffix) && stream.length > suffix.length) {
      return { base: stream.slice(0, -suffix.length), hand };
    }
  }
  return { base: stream, hand: null };
}

function streamBaseId(stream) {
  return splitStreamId(stream).base;
}

function streamHandId(stream) {
  return splitStreamId(stream).hand;
}

function namespacedTopic(hand, topic) {
  if (!hand || !topic?.startsWith("/")) return topic;
  const slash = topic.indexOf("/", 1);
  if (slash < 0) return `/${hand}${topic}`;
  return `${topic.slice(0, slash)}/${hand}${topic.slice(slash)}`;
}

function formatStreamSource(stream) {
  if (!stream) return "—";
  const base = streamBaseId(stream);
  const hand = streamHandId(stream);
  const tpl = STREAM_TOPIC[base];
  const topic = tpl ? namespacedTopic(hand, tpl) : stream;
  return hand ? `${stream} → ${topic}` : `${stream} → ${topic}`;
}

function truncateLabel(s, max = 22) {
  const str = String(s);
  return str.length <= max ? str : `${str.slice(0, max - 1)}…`;
}

function getCurrentHandId() {
  const pickVal = document.getElementById("viz-hand-pick")?.value;
  if (pickVal) return pickVal;
  const selected = vizConfig?.selected_hands || [];
  return selected[0] || null;
}

function resolveHandIdForViz(cfg = vizConfig) {
  const pickVal = document.getElementById("viz-hand-pick")?.value;
  if (pickVal) return pickVal;
  const selected = cfg?.selected_hands || [];
  return selected[0] || null;
}

/** 当前手型的 joint_cmd 流：io_teleop.joint_cmd_*.{hand} */
function isHandCmdStream(stream) {
  const handId = getCurrentHandId();
  if (!handId || !stream.endsWith(`.${handId}`)) return false;
  const base = streamBaseId(stream);
  return base === STREAM_HAND_CMD_LEFT || base === STREAM_HAND_CMD_RIGHT;
}

function rememberHandJointNames(side, names) {
  if (names?.length) lastHandJointNames[side] = [...names];
}

function setRobotJointPositions(robot, names, positions) {
  if (!robot || !names?.length) return;
  for (let i = 0; i < names.length; i++) {
    const v = positions[i];
    if (v == null || Number.isNaN(v)) continue;
    try {
      robot.setJointValue(names[i], v);
    } catch (_) {
      /* 忽略未知关节 */
    }
  }
}

function resetHandJointsOnRobot(side) {
  const names = lastHandJointNames[side] || [];
  if (!names.length) return;
  if (handVizState.mode === "split") {
    const robot = side === "right" ? handVizState.right : handVizState.left;
    setRobotJointPositions(robot, names, names.map(() => 0));
    return;
  }
  if (handVizState.combined) {
    for (const name of names) {
      try {
        handVizState.combined.setJointValue(name, 0);
      } catch (_) {
        /* 忽略未知关节 */
      }
    }
  }
}

/** 某一侧 cmd 断流 / 控制器停 / 拓扑无该侧：清图表、清缓存、URDF 该侧关节置 0 */
function clearHandSide(side) {
  const buf = side === "right" ? chartBuffers.handRight : chartBuffers.handLeft;
  const rateBuf = side === "right" ? chartBuffers.handRightRate : chartBuffers.handLeftRate;
  const yKey = side === "right" ? "handRight" : "handLeft";
  const yRateKey = side === "right" ? "handRightRate" : "handLeftRate";

  rememberHandJointNames(side, handJointCmdCache[side]?.names);
  handJointCmdCache[side] = null;
  handCmdLastRx[side] = 0;
  buf.clear();
  rateBuf.clear();
  chartYStable[yKey] = null;
  chartYStable[yRateKey] = null;

  const streamId = handSideStreamId(side);
  if (streamId) streamTickTimes.delete(streamId);

  handVizResampler[side].reset();
  resetHandJointsOnRobot(side);
  applyHandResampledToRobot(handVizResampler.left.lastOutput, handVizResampler.right.lastOutput);
  updateHandSideRateLabels(side, 0);

  const metaEl = document.getElementById(`chart-hand-${side}-meta`);
  if (metaEl) {
    metaEl.hidden = false;
    metaEl.textContent = t("viz.chartHandSideMetaEmpty");
  }
}

function checkHandCmdStale() {
  const handId = getCurrentHandId();
  if (!handId) return;
  const now = Date.now();
  for (const side of ["left", "right"]) {
    const last = handCmdLastRx[side];
    if (!last) continue;
    if (now - last > HAND_CMD_STALE_MS) clearHandSide(side);
  }
}

function statusExo(detail) {
  return detail?.exo || {};
}

function statusExoBindings(detail) {
  return statusExo(detail).bindings || {};
}

function statusExoTopology(detail) {
  return statusExo(detail).topology || "none";
}

function isUdpExoActive(detail) {
  if (!detail) return false;
  const exo = statusExo(detail);
  return exo.transport === "udp" && Boolean(exo.running);
}

function exoSideConnected(detail, side) {
  if (!detail || (side !== "left" && side !== "right")) return false;
  const devices = statusExoBindings(detail);
  if (!String(devices[side] || "").trim()) return false;
  return Boolean(statusExo(detail).running);
}

function isExoVisualizationActive(detail = lastGatewayStatus) {
  if (!detail) return false;
  if (!Boolean(statusExo(detail).running)) return false;
  if (isUdpExoActive(detail)) {
    const topo = statusExoTopology(detail);
    return topo !== "none";
  }
  return exoPortConnected.left || exoPortConnected.right;
}

function hasRecentExoJointStream(maxAgeMs = EXO_JOINT_STALE_MS) {
  const arr = streamTickTimes.get(STREAM_EXO_JOINT);
  if (!arr?.length) return false;
  return performance.now() - arr[arr.length - 1] < maxAgeMs;
}

function syncExoPortConnected(detail) {
  if (!detail) return;
  for (const side of ["left", "right"]) {
    exoPortConnected[side] = exoSideConnected(detail, side);
  }
}

function onGatewayPortDisconnect(ev) {
  const side = ev.detail?.side;
  if (side !== "left" && side !== "right") return;
  if (isUdpExoActive(lastGatewayStatus)) {
    clearHandSide(side);
    syncExoPortConnected(lastGatewayStatus);
    maybeClearExoWhenAllPortsGone();
    return;
  }
  exoPortConnected[side] = false;
  clearHandSide(side);
  maybeClearExoWhenAllPortsGone();
}

function onGatewayStatus(detail) {
  lastGatewayStatus = detail || null;
  syncExoPortConnected(detail);
  maybeClearExoWhenAllPortsGone();
}

function handSideStreamId(side) {
  const handId = getCurrentHandId();
  const base = side === "right" ? STREAM_HAND_CMD_RIGHT : STREAM_HAND_CMD_LEFT;
  return handId ? `${base}.${handId}` : "";
}

function jointFrameSnapshot(frame) {
  if (!frame?.names?.length) return null;
  return {
    names: [...frame.names],
    positions: [...frame.positions],
  };
}

class JointFrameResampler {
  constructor() {
    this._buffer = [];
    this._lastOutput = null;
  }

  ingest(data) {
    const names = data.names || [];
    const positions = data.position || [];
    if (!names.length) return;
    const now = performance.now();
    this._buffer.push({ t: now, names, positions });
    const cutoff = now - VIZ_RESAMPLE_BUFFER_MS;
    while (this._buffer.length && this._buffer[0].t < cutoff) this._buffer.shift();
  }

  /** 取刚结束时隙内最后一条样本；空时隙保持上一帧（补到 30Hz） */
  sample() {
    const now = performance.now();
    const bucketStart = now - VIZ_BUCKET_MS;
    const inBucket = this._buffer.filter((s) => s.t > bucketStart && s.t <= now);
    if (inBucket.length) {
      this._lastOutput = jointFrameSnapshot(inBucket[inBucket.length - 1]);
    }
    return this._lastOutput;
  }

  reset() {
    this._buffer = [];
    this._lastOutput = null;
  }

  get lastOutput() {
    return this._lastOutput;
  }
}

const exoVizResampler = new JointFrameResampler();
const handVizResampler = { left: new JointFrameResampler(), right: new JointFrameResampler() };

function resetAllVizResamplers() {
  exoVizResampler.reset();
  handVizResampler.left.reset();
  handVizResampler.right.reset();
}

function mergeHandVizFrames(leftFrame, rightFrame) {
  const names = [];
  const positions = [];
  for (const chunk of [leftFrame, rightFrame]) {
    if (!chunk?.names?.length) continue;
    const n = Math.min(chunk.names.length, chunk.positions.length);
    for (let i = 0; i < n; i++) {
      names.push(chunk.names[i]);
      positions.push(chunk.positions[i]);
    }
  }
  return names.length ? { names, positions } : null;
}

function applyHandResampledToRobot(leftFrame, rightFrame) {
  if (!handVizState.handId) return;
  if (handVizState.mode === "split") {
    if (leftFrame?.names?.length) {
      setRobotJointPositions(handVizState.left, leftFrame.names, leftFrame.positions);
    }
    if (rightFrame?.names?.length) {
      setRobotJointPositions(handVizState.right, rightFrame.names, rightFrame.positions);
    }
    return;
  }
  const merged = mergeHandVizFrames(leftFrame, rightFrame);
  if (merged) setRobotJointPositions(handVizState.combined, merged.names, merged.positions);
}

function tickVizUrdf() {
  const exo = exoVizResampler.sample();
  if (exo?.names?.length && exoScene) {
    exoScene.setJointPositions(exo.names, exo.positions);
  }
  applyHandResampledToRobot(handVizResampler.left.sample(), handVizResampler.right.sample());
}

function startVizUrdfTimer() {
  if (window.__vizUrdfTimer) return;
  window.__vizUrdfTimer = setInterval(tickVizUrdf, 1000 / VIZ_OUTPUT_HZ);
}

/** 识别 io_esk.joint_data 关节所属侧（right_0 / left_0 或 URDF 名 joint_Right* / joint_Left*） */
function exoJointSide(name) {
  const n = String(name);
  if (/^right_/i.test(n)) return "right";
  if (/^left_/i.test(n)) return "left";
  if (/joint_Right/i.test(n) || /RightSkeleton/i.test(n)) return "right";
  if (/joint_Left/i.test(n) || /LeftSkeleton/i.test(n)) return "left";
  return null;
}

function splitExoJoints(names, positions) {
  const left = { names: [], positions: [] };
  const right = { names: [], positions: [] };
  const n = Math.min(names?.length || 0, positions?.length || 0);
  for (let i = 0; i < n; i++) {
    const side = exoJointSide(names[i]);
    if (side === "right") {
      right.names.push(names[i]);
      right.positions.push(positions[i]);
    } else if (side === "left") {
      left.names.push(names[i]);
      left.positions.push(positions[i]);
    }
  }
  return { left, right };
}

function handleExoJointWs(stream, data) {
  exoAllPortsDisconnected = false;
  const names = data.names || [];
  const positions = data.position || [];
  const { left, right } = splitExoJoints(names, positions);

  if (left.names.length) {
    exoJointCache.left = left;
    pushChart(chartBuffers.exoLeft, left.names, left.positions, stream, EXO_SIDE_CHART_MAX_SERIES);
  }
  if (right.names.length) {
    exoJointCache.right = right;
    pushChart(chartBuffers.exoRight, right.names, right.positions, stream, EXO_SIDE_CHART_MAX_SERIES);
  }

  exoVizResampler.ingest(data);
}

function clearHandStreamTicks() {
  for (const side of ["left", "right"]) {
    const id = handSideStreamId(side);
    if (id) streamTickTimes.delete(id);
  }
}

function updateHandSideRateLabels(side, hz) {
  const streamId = handSideStreamId(side);
  const metaEl = document.getElementById(`chart-hand-${side}-rate-meta`);
  if (!metaEl) return;
  const ticks = streamId ? streamTickTimes.get(streamId) : null;
  if (ticks?.length) {
    metaEl.textContent = t("viz.chartRateMeta", {
      hz: Number.isFinite(hz) ? hz.toFixed(1) : "0",
      unit: "Hz",
    });
  } else {
    metaEl.textContent =
      side === "right" ? t("viz.chartHandRateMetaEmptyRight") : t("viz.chartHandRateMetaEmptyLeft");
  }
}

function handleHandCmdWs(stream, data) {
  const side = streamBaseId(stream) === STREAM_HAND_CMD_RIGHT ? "right" : "left";
  const names = data.names || [];
  const positions = data.position || [];
  handJointCmdCache[side] = { names, positions };
  rememberHandJointNames(side, names);
  handCmdLastRx[side] = Date.now();

  const metaEl = document.getElementById(`chart-hand-${side}-meta`);
  if (metaEl) metaEl.hidden = true;

  handVizResampler[side].ingest(data);

  const sideBuf = side === "right" ? chartBuffers.handRight : chartBuffers.handLeft;
  pushChart(sideBuf, names, positions, stream, HAND_SIDE_CHART_MAX_SERIES, true);

  recordStreamTick(stream);
  const rateBuf = side === "right" ? chartBuffers.handRightRate : chartBuffers.handLeftRate;
  const hz = pushRateSample(rateBuf, stream, HAND_RATE_LABEL[side]);
  updateHandSideRateLabels(side, hz);
}

function clearHandChartData() {
  clearHandSide("left");
  clearHandSide("right");
  lastHandJointNames.left = [];
  lastHandJointNames.right = [];
}

const MOVABLE_JOINT_TYPES = new Set(["revolute", "continuous", "prismatic"]);

function jointHasMovableLimits(joint) {
  return MOVABLE_JOINT_TYPES.has(joint?.jointType);
}

/** 从已加载 URDF 机器人汇总关节限位，供图表 Y 轴参考 */
function collectUrdfYRange(robot, opts = {}) {
  const { jointNames, sideFilter } = opts;
  if (!robot?.joints) return null;
  let names;
  if (jointNames?.length) {
    names = jointNames;
  } else {
    names = Object.keys(robot.joints).filter((n) => {
      if (sideFilter && !sideFilter(n)) return false;
      return jointHasMovableLimits(robot.joints[n]);
    });
  }
  let ymin = Infinity;
  let ymax = -Infinity;
  let count = 0;
  for (const name of names) {
    const joint = robot.joints[name];
    if (!joint || !jointHasMovableLimits(joint)) continue;
    if (sideFilter && !sideFilter(name)) continue;
    const lo = Number(joint.limit?.lower);
    const hi = Number(joint.limit?.upper);
    if (!Number.isFinite(lo) || !Number.isFinite(hi)) continue;
    if (lo < ymin) ymin = lo;
    if (hi > ymax) ymax = hi;
    count += 1;
  }
  if (!count || !Number.isFinite(ymin)) return null;
  return { min: ymin, max: ymax };
}

function handJointSidePrefix(name) {
  const n = String(name);
  if (/^left_/i.test(n) || /^L_/i.test(n)) return "left";
  if (/^right_/i.test(n) || /^R_/i.test(n)) return "right";
  return null;
}

/** 直接解析 URDF XML 中的关节限位（双手型可分别拉取 left/right URDF） */
async function fetchUrdfJointLimits(urdfUrl, sideFilter) {
  if (!urdfUrl) return null;
  try {
    const res = await fetch(urdfUrl);
    if (!res.ok) return null;
    const doc = new DOMParser().parseFromString(await res.text(), "application/xml");
    let ymin = Infinity;
    let ymax = -Infinity;
    let count = 0;
    for (const joint of doc.querySelectorAll("joint")) {
      const type = joint.getAttribute("type");
      if (!jointHasMovableLimits({ jointType: type })) continue;
      const jname = joint.getAttribute("name") || "";
      if (sideFilter && !sideFilter(jname)) continue;
      const limit = joint.querySelector("limit");
      if (!limit) continue;
      const lo = Number(limit.getAttribute("lower"));
      const hi = Number(limit.getAttribute("upper"));
      if (!Number.isFinite(lo) || !Number.isFinite(hi)) continue;
      if (lo < ymin) ymin = lo;
      if (hi > ymax) ymax = hi;
      count += 1;
    }
    if (!count || !Number.isFinite(ymin)) return null;
    return { min: ymin, max: ymax };
  } catch (e) {
    console.warn("URDF limit parse failed", urdfUrl, e);
    return null;
  }
}

function refreshExoUrdfChartYLims() {
  urdfChartYLim.exoLeft = collectUrdfYRange(exoRobot, {
    sideFilter: (n) => exoJointSide(n) === "left",
  });
  urdfChartYLim.exoRight = collectUrdfYRange(exoRobot, {
    sideFilter: (n) => exoJointSide(n) === "right",
  });
}

async function refreshExoUrdfChartYLimsFromUrl() {
  refreshExoUrdfChartYLims();
  if (urdfChartYLim.exoLeft && urdfChartYLim.exoRight) return;
  const url = vizConfig?.exo?.urdf;
  if (!url) return;
  const [left, right] = await Promise.all([
    fetchUrdfJointLimits(url, (n) => exoJointSide(n) === "left"),
    fetchUrdfJointLimits(url, (n) => exoJointSide(n) === "right"),
  ]);
  if (left) urdfChartYLim.exoLeft = left;
  if (right) urdfChartYLim.exoRight = right;
}

async function refreshHandUrdfChartYLims(handId) {
  const entry = handEntryForPick(vizConfig, handId);
  if (!entry) {
    urdfChartYLim.handLeft = null;
    urdfChartYLim.handRight = null;
    return;
  }
  const [left, right] = await Promise.all([
    fetchUrdfJointLimits(entry.urdf_left, (n) => handJointSidePrefix(n) === "left"),
    fetchUrdfJointLimits(entry.urdf_right, (n) => handJointSidePrefix(n) === "right"),
  ]);
  urdfChartYLim.handLeft = left;
  urdfChartYLim.handRight = right;
}

function defaultJointYLim(yUnit) {
  if (yUnit === "Hz") return { min: 0, max: 10 };
  return { min: -3.14, max: 3.14 };
}

function resolveChartYLim(explicitLim, yUnit) {
  if (explicitLim && Number.isFinite(explicitLim.min) && Number.isFinite(explicitLim.max)) {
    return explicitLim;
  }
  return defaultJointYLim(yUnit);
}

function stableYRange(key, ymin, ymax) {
  const span = ymax - ymin || 1;
  const pad = Math.max(span * 0.08, 0.02);
  const tmin = ymin - pad;
  const tmax = ymax + pad;
  let r = chartYStable[key];
  if (!r) {
    r = { min: tmin, max: tmax };
    chartYStable[key] = r;
    return r;
  }
  const a = 0.12;
  r.min += (tmin - r.min) * a;
  r.max += (tmax - r.max) * a;
  if (tmin < r.min) r.min = tmin;
  if (tmax > r.max) r.max = tmax;
  return r;
}

const CHART_Y_AXIS_DECIMALS = 2;

function formatChartAxisValue(v, _unit) {
  if (!Number.isFinite(v)) return "—";
  return v.toFixed(CHART_Y_AXIS_DECIMALS);
}

function measureYAxisPad(ctx, ymin, ymax, yUnit, tickCount, font = "11px monospace", tickValues = null) {
  const range = ymax - ymin;
  if (range <= 0) return 44;
  ctx.save();
  ctx.font = font;
  let maxW = 30;
  const values = resolvePlotYTickValues(ymin, ymax, tickCount, tickValues);
  const maxVal = Math.max(...values);
  for (const val of values) {
    const label =
      val === maxVal && yUnit
        ? `${formatChartAxisValue(val, yUnit)} ${yUnit}`
        : formatChartAxisValue(val, yUnit);
    maxW = Math.max(maxW, ctx.measureText(label).width);
  }
  ctx.restore();
  return Math.ceil(maxW) + 12;
}

function resolvePlotYTickValues(ymin, ymax, tickCount, tickValues) {
  if (tickValues?.length) {
    return tickValues.filter((v) => v >= ymin && v <= ymax);
  }
  const ticks = Math.max(2, tickCount);
  const range = ymax - ymin;
  return Array.from({ length: ticks }, (_, i) => {
    const frac = ticks === 1 ? 0 : i / (ticks - 1);
    return ymax - frac * range;
  });
}

function rateYTickValuesForRange(ymin, ymax) {
  const clipped = RATE_Y_TICK_VALUES.filter((v) => v >= ymin && v <= ymax);
  if (clipped.includes(120) && clipped.length >= 3) return clipped;
  return null;
}

function drawPlotYAxis(ctx, pad, plotW, plotH, ymin, ymax, yUnit, tickCount = 5, tickValues = null) {
  const range = ymax - ymin;
  if (range <= 0 || plotW <= 0 || plotH <= 0) return;
  const values = resolvePlotYTickValues(ymin, ymax, tickCount, tickValues);
  if (!values.length) return;
  const minVal = Math.min(...values);
  const maxVal = Math.max(...values);
  const labelX = pad.l - 8;
  ctx.save();
  ctx.strokeStyle = "rgba(30, 27, 23, 0.14)";
  ctx.fillStyle = VIZ_BRAND.black;
  ctx.font = "11px monospace";
  ctx.textAlign = "right";
  for (const val of values) {
    const y = pad.t + plotH - ((val - ymin) / range) * plotH;
    ctx.beginPath();
    ctx.moveTo(pad.l, y);
    ctx.lineTo(pad.l + plotW, y);
    ctx.stroke();
    const label =
      val === maxVal && yUnit
        ? `${formatChartAxisValue(val, yUnit)} ${yUnit}`
        : formatChartAxisValue(val, yUnit);
    if (val === maxVal) ctx.textBaseline = "top";
    else if (val === minVal) ctx.textBaseline = "bottom";
    else ctx.textBaseline = "middle";
    ctx.fillText(label, labelX, y);
  }
  ctx.textAlign = "left";
  ctx.textBaseline = "alphabetic";
  ctx.restore();
}

const VIZ_BRAND = {
  black: "#1E1B17",
  green: "#7FC42A",
  gray: "#A9B4B3",
  white: "#FFFFFF",
};

const CHART_PALETTE = [
  VIZ_BRAND.green,
  VIZ_BRAND.black,
  "#6aa822",
  "#5a5854",
  VIZ_BRAND.gray,
  "#9ed654",
  "#c9a227",
  "#c44a3a",
  "#4a7c59",
  "#8b7355",
];

const RATE_CHART_COLOR = VIZ_BRAND.green;

function chartSeriesColor(idx) {
  if (idx < CHART_PALETTE.length) return CHART_PALETTE[idx];
  const hue = (idx * 47) % 360;
  return `hsl(${hue} 72% 62%)`;
}

/** 图例面板 id → 用户隐藏的曲线名集合 */
const chartHiddenSeries = new Map();

/** 四角外扩放大图标（非 + 号） */
const CHART_EXPAND_ICON_SVG =
  '<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true" focusable="false"><path d="M8 3H3v5"/><path d="M16 3h5v5"/><path d="M8 21H3v-5"/><path d="M16 21h5v-5"/></svg>';

/** 关节折线图：canvas id → 绘制参数 */
const JOINT_CHART_DEFS = {
  "chart-exo-left-joint": {
    titleKey: "viz.chartExoLeft",
    buffer: "exoLeft",
    yRangeKey: "exoLeft",
    yLimKey: "exoLeft",
    legendId: "chart-exo-left-legend",
    metaId: "chart-exo-left-meta",
  },
  "chart-exo-right-joint": {
    titleKey: "viz.chartExoRight",
    buffer: "exoRight",
    yRangeKey: "exoRight",
    yLimKey: "exoRight",
    legendId: "chart-exo-right-legend",
    metaId: "chart-exo-right-meta",
  },
  "chart-hand-left-joint": {
    titleKey: "viz.chartHandLeft",
    buffer: "handLeft",
    yRangeKey: "handLeft",
    yLimKey: "handLeft",
    legendId: "chart-hand-left-legend",
    metaId: "chart-hand-left-meta",
  },
  "chart-hand-right-joint": {
    titleKey: "viz.chartHandRight",
    buffer: "handRight",
    yRangeKey: "handRight",
    yLimKey: "handRight",
    legendId: "chart-hand-right-legend",
    metaId: "chart-hand-right-meta",
  },
};

/** 当前打开的关节图放大层（canvas id） */
let activeJointChartLightboxId = null;

function legendStateId(legendEl) {
  return legendEl?.dataset?.legendStateId || legendEl?.id || null;
}

function chartLegendElementForCanvas(canvas, opts) {
  if (opts.legendEl) return opts.legendEl;
  if (!canvas?.id) return null;
  return document.getElementById(canvas.id.replace(/-joint$/, "-legend"));
}

function legendHiddenSet(legendEl) {
  const key = legendStateId(legendEl);
  if (!key) return new Set();
  if (!chartHiddenSeries.has(key)) chartHiddenSeries.set(key, new Set());
  return chartHiddenSeries.get(key);
}

function isSeriesHidden(legendEl, name) {
  return legendHiddenSet(legendEl).has(name);
}

function setSeriesHidden(legendEl, name, hidden) {
  const set = legendHiddenSet(legendEl);
  if (hidden) set.add(name);
  else set.delete(name);
}

function legendSeriesNames(legendEl) {
  if (!legendEl) return [];
  return [...legendEl.querySelectorAll(".viz-chart-legend-toggle[data-series]")]
    .map((cb) => cb.dataset.series)
    .filter(Boolean);
}

function setAllLegendSeriesHidden(legendEl, names, hidden) {
  const set = legendHiddenSet(legendEl);
  for (const name of names) {
    if (hidden) set.add(name);
    else set.delete(name);
  }
}

function syncLegendSelectAll(legendEl) {
  const checkbox = legendEl?.querySelector(".viz-chart-legend-select-all-toggle");
  if (!checkbox) return;
  const names = legendSeriesNames(legendEl);
  if (!names.length) {
    checkbox.checked = false;
    checkbox.indeterminate = false;
    return;
  }
  const hidden = legendHiddenSet(legendEl);
  const visibleCount = names.filter((name) => !hidden.has(name)).length;
  checkbox.checked = visibleCount === names.length;
  checkbox.indeterminate = visibleCount > 0 && visibleCount < names.length;
}

function buildLegendSelectAllRow(legendEl) {
  const row = document.createElement("div");
  row.className = "viz-chart-legend-item viz-chart-legend-select-all";
  row.setAttribute("role", "listitem");
  const checkbox = document.createElement("input");
  checkbox.type = "checkbox";
  checkbox.className = "viz-chart-legend-toggle viz-chart-legend-select-all-toggle";
  checkbox.setAttribute("aria-label", t("viz.chartLegendSelectAll"));
  const sw = document.createElement("span");
  sw.className = "viz-chart-legend-swatch viz-chart-legend-swatch--spacer";
  sw.setAttribute("aria-hidden", "true");
  const label = document.createElement("span");
  label.className = "viz-chart-legend-label viz-chart-legend-select-all-label";
  label.textContent = t("viz.chartLegendSelectAll");
  checkbox.addEventListener("change", () => {
    const names = legendSeriesNames(legendEl);
    setAllLegendSeriesHidden(legendEl, names, !checkbox.checked);
    checkbox.indeterminate = false;
    const yKey = chartYRangeKeyForLegend(legendEl);
    if (yKey) chartYStable[yKey] = null;
    redrawCharts();
  });
  row.addEventListener("click", (e) => {
    if (e.target === checkbox) return;
    checkbox.checked = !checkbox.checked;
    checkbox.dispatchEvent(new Event("change"));
  });
  row.append(checkbox, sw, label);
  return row;
}

function chartYRangeKeyForLegend(legendEl) {
  const map = {
    "chart-exo-left-legend": "exoLeft",
    "chart-exo-right-legend": "exoRight",
    "chart-hand-left-legend": "handLeft",
    "chart-hand-right-legend": "handRight",
  };
  const id = legendStateId(legendEl);
  return id ? map[id] || null : null;
}

function syncScrollableChartLegend(legendEl) {
  if (!legendEl || legendEl.classList.contains("is-empty")) return;
  legendEl.querySelectorAll(".viz-chart-legend-item:not(.viz-chart-legend-select-all)").forEach((row) => {
    const checkbox = row.querySelector(".viz-chart-legend-toggle");
    const swatch = row.querySelector(".viz-chart-legend-swatch");
    const name = checkbox?.dataset.series;
    if (!name) return;
    const visible = !isSeriesHidden(legendEl, name);
    checkbox.checked = visible;
    row.classList.toggle("is-muted", !visible);
    if (swatch) swatch.style.opacity = visible ? "1" : "0.35";
  });
  syncLegendSelectAll(legendEl);
}

function renderScrollableChartLegend(legendEl, entries) {
  if (!legendEl) return;
  if (!entries?.length) {
    legendEl.dataset.legendNamesKey = "";
    legendEl.innerHTML = "";
    legendEl.classList.add("is-empty");
    return;
  }
  const namesKey = entries.map((e) => e.name).join("\0");
  if (legendEl.dataset.legendNamesKey === namesKey && legendEl.childElementCount > 0) {
    legendEl.classList.remove("is-empty");
    syncScrollableChartLegend(legendEl);
    return;
  }
  legendEl.dataset.legendNamesKey = namesKey;
  legendEl.innerHTML = "";
  legendEl.classList.remove("is-empty");
  const frag = document.createDocumentFragment();
  frag.appendChild(buildLegendSelectAllRow(legendEl));
  for (let idx = 0; idx < entries.length; idx++) {
    const entry = entries[idx];
    const name = String(entry.name);
    const visible = !isSeriesHidden(legendEl, name);
    const row = document.createElement("div");
    row.className = "viz-chart-legend-item";
    if (!visible) row.classList.add("is-muted");
    row.setAttribute("role", "listitem");
    const checkbox = document.createElement("input");
    checkbox.type = "checkbox";
    checkbox.className = "viz-chart-legend-toggle";
    checkbox.checked = visible;
    checkbox.dataset.series = name;
    checkbox.setAttribute("aria-label", t("viz.chartLegendToggle", { name }));
    const sw = document.createElement("span");
    sw.className = "viz-chart-legend-swatch";
    sw.style.background = chartSeriesColor(idx);
    if (!visible) sw.style.opacity = "0.35";
    const label = document.createElement("span");
    label.className = "viz-chart-legend-label";
    label.textContent = name;
    checkbox.addEventListener("change", () => {
      setSeriesHidden(legendEl, name, !checkbox.checked);
      const yKey = chartYRangeKeyForLegend(legendEl);
      if (yKey) chartYStable[yKey] = null;
      redrawCharts();
    });
    row.addEventListener("click", (e) => {
      if (e.target === checkbox) return;
      checkbox.checked = !checkbox.checked;
      checkbox.dispatchEvent(new Event("change"));
    });
    row.append(checkbox, sw, label);
    frag.appendChild(row);
  }
  legendEl.appendChild(frag);
  syncLegendSelectAll(legendEl);
}

function rateChartLegendElementForCanvas(canvas) {
  if (!canvas?.id) return null;
  return document.getElementById(`${canvas.id}-legend`);
}

function renderRateChartHeaderLegend(legendEl, entries) {
  if (!legendEl) return;
  const namesKey = entries.map((e) => e.name).join("\0");
  if (legendEl.dataset.legendNamesKey === namesKey && legendEl.childElementCount > 0) {
    legendEl.classList.toggle("is-empty", entries.length === 0);
    return;
  }
  legendEl.dataset.legendNamesKey = namesKey;
  legendEl.innerHTML = "";
  if (!entries.length) {
    legendEl.classList.add("is-empty");
    return;
  }
  legendEl.classList.remove("is-empty");
  const frag = document.createDocumentFragment();
  entries.forEach(({ name }, idx) => {
    const row = document.createElement("span");
    row.className = "viz-chart-rate-legend-item";
    const sw = document.createElement("span");
    sw.className = "viz-chart-rate-legend-swatch";
    sw.style.background = RATE_CHART_COLOR;
    const label = document.createElement("span");
    label.className = "viz-chart-rate-legend-label";
    label.textContent = name;
    row.append(sw, label);
    frag.appendChild(row);
  });
  legendEl.appendChild(frag);
}

const CHART_URDF_FOOTER_H = 16;

function chartBottomPad(showYAxis, legendNamesOnly, urdfLim) {
  const base = legendNamesOnly ? 12 : 18;
  if (showYAxis && urdfLim) return base + CHART_URDF_FOOTER_H;
  return base;
}

function formatUrdfRangeHint(urdfLim, yUnit) {
  return t("viz.chartUrdfRange", {
    min: formatChartAxisValue(urdfLim.min, yUnit),
    max: formatChartAxisValue(urdfLim.max, yUnit),
    unit: yUnit || "",
  });
}

/** 在 canvas 底部预留区绘制 URDF 限位提示（白底，避免被曲线遮挡） */
function drawUrdfRangeFooter(ctx, pad, cw, ch, urdfLim, yUnit) {
  if (!urdfLim) return;
  const text = formatUrdfRangeHint(urdfLim, yUnit);
  ctx.font = "10px monospace";
  const textW = ctx.measureText(text).width;
  const x = pad.l;
  const footerTop = ch - pad.b + 1;
  ctx.fillStyle = VIZ_BRAND.white;
  ctx.fillRect(
    x - 2,
    footerTop,
    Math.min(textW + 10, Math.max(cw - pad.l - pad.r, 40)),
    Math.max(pad.b - 2, 12)
  );
  ctx.fillStyle = VIZ_BRAND.gray;
  ctx.fillText(text, x, ch - 5);
}

function drawLineChart(canvas, seriesMap, title, yRangeKey = null, yUnit = "", opts = {}) {
  if (!canvas) return;
  const showYAxis = !!opts.showYAxis;
  const legendNamesOnly = !!opts.legendNamesOnly;
  const legendEl = legendNamesOnly ? chartLegendElementForCanvas(canvas, opts) : null;
  const useScrollLegend = legendNamesOnly && !!legendEl;
  const externalRateLegendEl = yUnit === "Hz" ? rateChartLegendElementForCanvas(canvas) : null;
  const useExternalRateLegend = yUnit === "Hz" && opts.externalRateLegend !== false;
  const yTicks = opts.yTicks ?? 5;
  const ctx = canvas.getContext("2d");
  const cw = canvas.clientWidth || 300;
  const ch = canvas.clientHeight || 140;
  canvas.width = cw * devicePixelRatio;
  canvas.height = ch * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  ctx.fillStyle = VIZ_BRAND.white;
  ctx.fillRect(0, 0, cw, ch);

  const allEntries = [...seriesMap.entries()]
    .map(([name, serie]) => ({
      name,
      values: serie.values || [],
      stream: serie.stream || "",
    }))
    .filter((e) => e.values.length > 0);
  const entries =
    useScrollLegend && legendEl
      ? allEntries.filter((e) => !isSeriesHidden(legendEl, e.name))
      : allEntries;

  const urdfLim = opts.yLim;
  const strictYLim = opts.strictYLim;
  if (allEntries.length === 0) {
    renderScrollableChartLegend(legendEl, []);
    if (useExternalRateLegend) renderRateChartHeaderLegend(externalRateLegendEl, []);
    const waitingLine = t("viz.chartWaiting");
    if (showYAxis) {
      const refLim = strictYLim || resolveChartYLim(urdfLim, yUnit);
      let ymin = refLim.min;
      let ymax = refLim.max;
      if (ymin === ymax) {
        ymin -= 0.5;
        ymax += 0.5;
      }
      const yTickValues = resolveChartYTickValues(opts, ymin, ymax);
      const pad = {
        l: measureYAxisPad(ctx, ymin, ymax, yUnit, yTicks, "11px monospace", yTickValues),
        r: useExternalRateLegend ? 56 : 10,
        t: useExternalRateLegend ? 22 : legendNamesOnly ? 12 : 28,
        b: chartBottomPad(showYAxis, legendNamesOnly, urdfLim),
      };
      const plotW = Math.max(cw - pad.l - pad.r, 1);
      const plotH = Math.max(ch - pad.t - pad.b, 1);
      ctx.fillStyle = VIZ_BRAND.black;
      ctx.font = "11px monospace";
      ctx.fillText(waitingLine, pad.l, 14);
      ctx.strokeStyle = "rgba(30, 27, 23, 0.14)";
      ctx.strokeRect(pad.l, pad.t, plotW, plotH);
      drawPlotYAxis(ctx, pad, plotW, plotH, ymin, ymax, yUnit, yTicks, yTickValues);
      drawUrdfRangeFooter(ctx, pad, cw, ch, urdfLim, yUnit);
    } else {
      ctx.fillStyle = VIZ_BRAND.black;
      ctx.font = "12px monospace";
      ctx.fillText(waitingLine, 8, 20);
    }
    return;
  }

  if (useScrollLegend) renderScrollableChartLegend(legendEl, allEntries);
  if (useExternalRateLegend) renderRateChartHeaderLegend(externalRateLegendEl, allEntries);

  let ymin;
  let ymax;
  let dataMin = Infinity;
  let dataMax = -Infinity;
  for (const { values } of entries) {
    for (const v of values) {
      if (v < dataMin) dataMin = v;
      if (v > dataMax) dataMax = v;
    }
  }
  if (
    strictYLim &&
    Number.isFinite(strictYLim.min) &&
    Number.isFinite(strictYLim.max)
  ) {
    ymin = strictYLim.min;
    ymax = strictYLim.max;
  } else if (entries.length === 0) {
    if (urdfLim && Number.isFinite(urdfLim.min) && Number.isFinite(urdfLim.max)) {
      ymin = urdfLim.min;
      ymax = urdfLim.max;
    } else {
      const fallback = defaultJointYLim(yUnit);
      ymin = fallback.min;
      ymax = fallback.max;
    }
  } else if (useScrollLegend) {
    ymin = dataMin;
    ymax = dataMax;
    if (!Number.isFinite(ymin)) {
      const fallback = urdfLim || defaultJointYLim(yUnit);
      ymin = fallback.min;
      ymax = fallback.max;
    } else {
      const span = ymax - ymin || 0.01;
      const pad = Math.max(span * 0.12, 0.02);
      ymin -= pad;
      ymax += pad;
      if (urdfLim && Number.isFinite(urdfLim.min) && Number.isFinite(urdfLim.max)) {
        ymin = Math.max(ymin, urdfLim.min);
        ymax = Math.min(ymax, urdfLim.max);
        if (ymin >= ymax) {
          ymin = urdfLim.min;
          ymax = urdfLim.max;
        }
      }
    }
  } else if (urdfLim && Number.isFinite(urdfLim.min) && Number.isFinite(urdfLim.max)) {
    ymin = urdfLim.min;
    ymax = urdfLim.max;
    if (Number.isFinite(dataMin)) ymin = Math.min(ymin, dataMin);
    if (Number.isFinite(dataMax)) ymax = Math.max(ymax, dataMax);
  } else {
    ymin = dataMin;
    ymax = dataMax;
    if (!Number.isFinite(ymin)) {
      const fallback = defaultJointYLim(yUnit);
      ymin = fallback.min;
      ymax = fallback.max;
    }
  }
  if (ymin === ymax) {
    ymin -= 0.5;
    ymax += 0.5;
  }
  if (yRangeKey && !strictYLim) {
    const yr = stableYRange(yRangeKey, ymin, ymax);
    ymin = yr.min;
    ymax = yr.max;
  }

  const yTickValues = resolveChartYTickValues(opts, ymin, ymax);
  const yAxisPad = showYAxis
    ? measureYAxisPad(ctx, ymin, ymax, yUnit, yTicks, "11px monospace", yTickValues)
    : 36;
  const pad = {
    l: yAxisPad,
    r: useExternalRateLegend ? 56 : 10,
    t: useExternalRateLegend ? 22 : legendNamesOnly ? 12 : 28,
    b: chartBottomPad(showYAxis, legendNamesOnly, urdfLim),
  };
  let legendW = showYAxis ? 72 : Math.min(200, Math.max(100, cw * 0.38));
  let legendRowH = 28;
  if (useScrollLegend || useExternalRateLegend) {
    pad.r = useExternalRateLegend ? 56 : 10;
  } else if (legendNamesOnly) {
    const maxNameLen = Math.max(...allEntries.map((e) => String(e.name).length), 4);
    legendW = Math.min(cw * 0.55, Math.max(96, maxNameLen * 6.4 + 20));
    legendRowH = Math.max(9, Math.min(13, Math.floor((ch - pad.t - 8) / entries.length)));
    pad.r = legendW + 10;
  } else if (yUnit === "Hz") {
    ctx.save();
    ctx.font = "11px monospace";
    let maxTextW = ctx.measureText(STREAM_EXO_JOINT).width;
    for (const { name } of allEntries) {
      maxTextW = Math.max(maxTextW, ctx.measureText(String(name)).width);
    }
    ctx.restore();
    legendW = Math.max(legendW, Math.ceil(maxTextW) + 24);
    pad.r = legendW + 10;
  } else {
    pad.r = legendW + 10;
  }

  const plotW = Math.max(cw - pad.l - pad.r, 1);
  const plotH = Math.max(ch - pad.t - pad.b, 1);
  ctx.strokeStyle = "rgba(30, 27, 23, 0.14)";
  ctx.strokeRect(pad.l, pad.t, plotW, plotH);
  if (showYAxis) drawPlotYAxis(ctx, pad, plotW, plotH, ymin, ymax, yUnit, yTicks, yTickValues);

  entries.forEach(({ name, values, stream }, idx) => {
    const colorIdx = allEntries.findIndex((e) => e.name === name);
    const color = yUnit === "Hz" ? RATE_CHART_COLOR : chartSeriesColor(colorIdx >= 0 ? colorIdx : 0);
    const pts = values.length === 1 ? [values[0], values[0]] : values;
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    pts.forEach((v, i) => {
      const x = pad.l + (i / (pts.length - 1 || 1)) * plotW;
      const y = pad.t + plotH - ((v - ymin) / (ymax - ymin)) * plotH;
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    });
    ctx.stroke();

    if (useScrollLegend || useExternalRateLegend) return;

    const lx = cw - legendW + 4;
    const ly = pad.t + 4 + idx * legendRowH;
    ctx.fillStyle = color;
    ctx.fillRect(lx, ly + 2, 10, 3);
    ctx.fillStyle = VIZ_BRAND.black;
    ctx.font = legendNamesOnly ? "10px monospace" : "11px monospace";
    if (legendNamesOnly || yUnit === "Hz") {
      ctx.fillText(String(name), lx + 14, ly + 9);
      return;
    }
    const lastV = values[values.length - 1];
    ctx.fillText(truncateLabel(name, 18), lx + 14, ly + 8);
    ctx.fillStyle = VIZ_BRAND.gray;
    ctx.font = "10px monospace";
    const tail = Number.isFinite(lastV) ? `=${lastV.toFixed(4)}` : "";
    if (stream) {
      ctx.fillText(
        truncateLabel(`${tail} · ${formatStreamSource(stream)}`, 30),
        lx + 14,
        ly + 20
      );
    } else if (tail) {
      ctx.fillText(tail, lx + 14, ly + 20);
    }
  });

  if (!useExternalRateLegend && !legendNamesOnly) {
    ctx.fillStyle = VIZ_BRAND.black;
    ctx.font = "11px monospace";
    ctx.fillText(title, pad.l, 14);
  }
  if (!legendNamesOnly && yUnit !== "Hz") {
    ctx.fillStyle = VIZ_BRAND.gray;
    ctx.font = "10px monospace";
    const unitSuffix = yUnit ? ` ${yUnit}` : "";
    let rangeHint;
    if (showYAxis) {
      rangeHint = t("viz.chartDataRange", {
        min: formatChartAxisValue(dataMin, yUnit),
        max: formatChartAxisValue(dataMax, yUnit),
        unit: yUnit || "",
      });
      if (urdfLim) {
        rangeHint += ` · ${t("viz.chartUrdfRange", {
          min: formatChartAxisValue(urdfLim.min, yUnit),
          max: formatChartAxisValue(urdfLim.max, yUnit),
          unit: yUnit || "",
        })}`;
      }
    } else {
      rangeHint = `${t("viz.chartLegendHint")} · Y[${formatChartAxisValue(ymin, yUnit)}, ${formatChartAxisValue(ymax, yUnit)}]${unitSuffix}`;
    }
    ctx.fillText(rangeHint, 8, ch - 6);
  } else if (showYAxis && urdfLim) {
    drawUrdfRangeFooter(ctx, pad, cw, ch, urdfLim, yUnit);
  }
}

function recordStreamTick(streamId) {
  if (!streamId) return;
  let arr = streamTickTimes.get(streamId);
  if (!arr) {
    arr = [];
    streamTickTimes.set(streamId, arr);
  }
  const now = performance.now();
  arr.push(now);
  const cutoff = now - FREQ_WINDOW_MS;
  while (arr.length && arr[0] < cutoff) arr.shift();
}

function computeStreamHz(streamId) {
  const arr = streamTickTimes.get(streamId);
  if (!arr?.length) return 0;
  if (arr.length === 1) return 1;
  const spanSec = (arr[arr.length - 1] - arr[0]) / 1000;
  if (spanSec <= 0) return arr.length;
  return (arr.length - 1) / spanSec;
}

function pushRateSample(buf, streamId, label) {
  const hz = computeStreamHz(streamId);
  let serie = buf.get(label || streamId);
  if (!serie) {
    serie = { values: [], stream: streamId };
    buf.set(label || streamId, serie);
  }
  if (streamId) serie.stream = streamId;
  serie.values.push(hz);
  if (serie.values.length > CHART_LEN) serie.values.shift();
  return hz;
}

/** 灵巧手单侧频率：无流时保持 cmd_* 序列并推入 0 */
function sampleHandRate(side) {
  const label = HAND_RATE_LABEL[side];
  const buf = side === "right" ? chartBuffers.handRightRate : chartBuffers.handLeftRate;
  const streamId = handSideStreamId(side);
  if (streamId) return pushRateSample(buf, streamId, label);
  let serie = buf.get(label);
  if (!serie) {
    serie = { values: [0], stream: label };
    buf.set(label, serie);
  } else {
    serie.stream = label;
    serie.values.push(0);
    if (serie.values.length > CHART_LEN) serie.values.shift();
  }
  return 0;
}

function updateExoRateLabels(hz) {
  const metaEl = document.getElementById("chart-exo-rate-meta");
  if (metaEl) {
    metaEl.textContent = t("viz.chartRateMeta", {
      hz: Number.isFinite(hz) ? hz.toFixed(1) : "0",
      unit: "Hz",
    });
  }
}

function clearExoVibrationChart() {
  lastExoVibrationData = null;
  exoVibrationLastRx = 0;
  streamTickTimes.delete(STREAM_EXO_VIBRATION);
}

function checkExoVibrationStale() {
  if (!exoVibrationLastRx) return;
  if (Date.now() - exoVibrationLastRx > VIBRATION_STALE_MS) clearExoVibrationChart();
}

function updateExoVibrationLabels(hz) {
  const metaEl = document.getElementById("chart-exo-vibration-meta");
  if (!metaEl) return;
  if (!lastExoVibrationData?.length) {
    metaEl.textContent = t("viz.chartExoVibrationMetaEmpty");
    return;
  }
  metaEl.textContent = t("viz.chartRateMeta", {
    hz: Number.isFinite(hz) ? hz.toFixed(1) : "0",
    unit: "Hz",
  });
}

/** 振动反馈：横轴 1–10 末端，纵轴 0–10 数值（柱状图，展示最近一帧） */
function drawExoVibrationChart(canvas) {
  if (!canvas) return;
  const ymin = EXO_VIBRATION_Y_MIN;
  const ymax = EXO_VIBRATION_Y_MAX;
  const yTicks = 6;
  const ctx = canvas.getContext("2d");
  const cw = canvas.clientWidth || 300;
  const ch = canvas.clientHeight || 140;
  canvas.width = cw * devicePixelRatio;
  canvas.height = ch * devicePixelRatio;
  ctx.setTransform(devicePixelRatio, 0, 0, devicePixelRatio, 0, 0);
  ctx.fillStyle = VIZ_BRAND.white;
  ctx.fillRect(0, 0, cw, ch);

  const yUnit = t("viz.chartExoVibrationAxisY");
  const xUnit = t("viz.chartExoVibrationAxisX");
  const barColor = chartSeriesColor(0);
  const valueLabelPad = 14;
  const pad = {
    l: measureYAxisPad(ctx, ymin, ymax, "", yTicks) + 6,
    r: 10,
    t: 10 + valueLabelPad,
    b: 34,
  };
  const plotW = Math.max(cw - pad.l - pad.r, 1);
  const plotH = Math.max(ch - pad.t - pad.b, 1);
  const slotW = plotW / EXO_VIBRATION_CHANNELS;
  const yRange = ymax - ymin;
  const barWidth = Math.max(slotW * 0.38, 4);

  drawPlotYAxis(ctx, pad, plotW, plotH, ymin, ymax, "", yTicks);
  ctx.strokeStyle = "rgba(30, 27, 23, 0.35)";
  ctx.beginPath();
  ctx.moveTo(pad.l, pad.t + plotH);
  ctx.lineTo(pad.l + plotW, pad.t + plotH);
  ctx.stroke();

  ctx.fillStyle = VIZ_BRAND.black;
  ctx.font = "10px monospace";
  ctx.textAlign = "center";
  ctx.textBaseline = "top";
  for (let i = 0; i < EXO_VIBRATION_CHANNELS; i++) {
    const xCenter = pad.l + slotW * i + slotW / 2;
    ctx.fillText(String(i + 1), xCenter, pad.t + plotH + 6);
  }
  ctx.fillText(xUnit, pad.l + plotW / 2, ch - 10);

  ctx.save();
  ctx.translate(12, pad.t + plotH / 2);
  ctx.rotate(-Math.PI / 2);
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";
  ctx.fillText(yUnit, 0, 0);
  ctx.restore();

  if (!lastExoVibrationData?.length) {
    ctx.textAlign = "left";
    ctx.textBaseline = "alphabetic";
    ctx.font = "11px monospace";
    ctx.fillText(t("viz.chartWaiting"), pad.l, 14);
    return;
  }

  for (let i = 0; i < EXO_VIBRATION_CHANNELS; i++) {
    const val = Number(lastExoVibrationData[i]);
    const clamped = Number.isFinite(val) ? Math.max(ymin, Math.min(ymax, val)) : ymin;
    const barH = ((clamped - ymin) / yRange) * plotH;
    const xCenter = pad.l + slotW * i + slotW / 2;
    const x = xCenter - barWidth / 2;
    const y = pad.t + plotH - barH;

    ctx.fillStyle = barColor;
    ctx.fillRect(x, y, barWidth, barH);

    if (clamped > 0) {
      ctx.fillStyle = VIZ_BRAND.black;
      ctx.font = "9px monospace";
      ctx.textAlign = "center";
      ctx.textBaseline = "bottom";
      ctx.fillText(formatChartAxisValue(clamped, ""), xCenter, y - 3);
    }
  }
}

/** 双手外骨骼口均已断开时置 true，避免 status 轮询重复清空 */
let exoAllPortsDisconnected = false;

function clearExoChartData() {
  chartBuffers.exoLeft.clear();
  chartBuffers.exoRight.clear();
  chartBuffers.exoRate.clear();
  chartYStable.exoLeft = null;
  chartYStable.exoRight = null;
  chartYStable.exoRate = null;
  exoJointCache.left = null;
  exoJointCache.right = null;
  streamTickTimes.delete(STREAM_EXO_JOINT);
  updateExoRateLabels(0);
}

/** io_esk.joint_data 单流：仅当左右串口都无连接时清空图表并将 URDF 关节置 0 */
function clearExoVisualization() {
  const namesToReset = new Set();
  for (const side of ["left", "right"]) {
    const c = exoJointCache[side];
    if (c?.names) for (const n of c.names) namesToReset.add(n);
  }
  clearExoChartData();
  exoVizResampler.reset();
  if (exoScene?.robot && namesToReset.size) {
    for (const name of namesToReset) {
      try {
        exoScene.robot.setJointValue(name, 0);
      } catch (_) {
        /* 忽略未知关节 */
      }
    }
  }
}

function maybeClearExoWhenAllPortsGone() {
  if (isExoVisualizationActive(lastGatewayStatus) || hasRecentExoJointStream()) {
    exoAllPortsDisconnected = false;
    return;
  }
  const allGone = !exoPortConnected.left && !exoPortConnected.right;
  if (!allGone) {
    exoAllPortsDisconnected = false;
    return;
  }
  if (exoAllPortsDisconnected) return;
  exoAllPortsDisconnected = true;
  clearExoVisualization();
}

function pushChart(buf, names, positions, stream, maxSeries = 6, smooth = false) {
  if (!names?.length || !positions?.length) return;
  const n = Math.min(names.length, positions.length, maxSeries);
  for (let i = 0; i < n; i++) {
    const key = names[i];
    let serie = buf.get(key);
    if (!serie) {
      serie = { values: [], stream: stream || "" };
      buf.set(key, serie);
    }
    if (stream) serie.stream = stream;
    const raw = Number(positions[i]) || 0;
    let v = raw;
    if (smooth && serie.values.length) {
      const last = serie.values[serie.values.length - 1];
      v = last * (1 - HAND_CHART_EMA) + raw * HAND_CHART_EMA;
    }
    serie.values.push(v);
    if (serie.values.length > CHART_LEN) serie.values.shift();
  }
}

function onWsPayload(raw) {
  let msg;
  try {
    msg = JSON.parse(raw);
  } catch (_) {
    return;
  }
  const stream = msg.stream || "";
  const data = msg.data;
  if (!data) return;

  if (stream === STREAM_EXO_JOINT) {
    recordStreamTick(STREAM_EXO_JOINT);
    handleExoJointWs(stream, data);
    const hz = pushRateSample(chartBuffers.exoRate, STREAM_EXO_JOINT, STREAM_EXO_JOINT);
    updateExoRateLabels(hz);
  } else if (stream === STREAM_EXO_VIBRATION || stream === "exo_vibration") {
    recordStreamTick(STREAM_EXO_VIBRATION);
    exoVibrationLastRx = Date.now();
    const raw = data.data;
    if (Array.isArray(raw)) {
      lastExoVibrationData = raw.slice(0, EXO_VIBRATION_CHANNELS);
    }
    updateExoVibrationLabels(computeStreamHz(STREAM_EXO_VIBRATION));
  } else if (isHandCmdStream(stream)) {
    handleHandCmdWs(stream, data);
  }
}

async function loadVizConfig() {
  const res = await fetch("/api/v1/visualization/config");
  if (!res.ok) throw new Error(`config HTTP ${res.status}`);
  return res.json();
}

function fillHandPicker(cfg) {
  const sel = document.getElementById("viz-hand-pick");
  if (!sel) return;
  sel.innerHTML = "";
  const selectedIds = cfg.selected_hands || [];
  const allHands = cfg.hands || [];
  const hands = allHands.filter((h) => selectedIds.includes(h.id));
  if (!hands.length) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = t("viz.noHandSelected");
    opt.disabled = true;
    opt.selected = true;
    sel.appendChild(opt);
    return;
  }
  for (const h of hands) {
    const opt = document.createElement("option");
    opt.value = h.id;
    opt.textContent = h.id;
    sel.appendChild(opt);
  }
  const primary = cfg.primary_hand || hands[0]?.id;
  if (primary && hands.some((h) => h.id === primary)) sel.value = primary;
  else sel.value = hands[0].id;
}

function handEntryForPick(cfg, handId) {
  return (cfg.hands || []).find((h) => h.id === handId);
}

function resetHandVizState() {
  handVizState = {
    handId: "",
    mode: "",
    group: null,
    combined: null,
    left: null,
    right: null,
  };
}

function handUrdfLayout(entry) {
  if (!entry) return null;
  if (entry.urdf_mode === "split") return "split";
  if (entry.urdf_mode === "combined") return "combined";
  if (entry.urdf_left_rel && entry.urdf_right_rel) {
    return entry.urdf_left_rel === entry.urdf_right_rel ? "combined" : "split";
  }
  return "combined";
}

function buildHandUrdfLoadSpec(entry) {
  if (!entry) return null;
  const mode = handUrdfLayout(entry);
  if (mode === "split") {
    return {
      mode: "split",
      leftUrl: entry.urdf_left,
      leftPkg: entry.package_left,
      rightUrl: entry.urdf_right,
      rightPkg: entry.package_right,
    };
  }
  return {
    mode: "combined",
    combinedUrl: entry.urdf_left || entry.urdf_right,
    combinedPkg: entry.package_left || entry.package_right,
    leftUrl: entry.urdf_left,
    leftPkg: entry.package_left,
    rightUrl: entry.urdf_right,
    rightPkg: entry.package_right,
  };
}

function handVizHasModel() {
  return !!(handVizState.combined || handVizState.left || handVizState.right);
}

async function loadHandUrdf(handId) {
  if (!vizConfig || !handScene) return;
  if (!handId) {
    if (!handVizState.handId && !handVizHasModel()) return;
    handScene.invalidateUrdfLoads();
    resetHandVizState();
    clearHandChartData();
    urdfChartYLim.handLeft = null;
    urdfChartYLim.handRight = null;
    await handScene.loadUrdf(null, null, null);
    return;
  }
  if (handId === handVizState.handId && handVizHasModel()) {
    if (!urdfChartYLim.handLeft && !urdfChartYLim.handRight) {
      await refreshHandUrdfChartYLims(handId);
    }
    return;
  }
  handScene.invalidateUrdfLoads();
  resetHandVizState();
  clearHandChartData();
  const entry = handEntryForPick(vizConfig, handId);
  const spec = buildHandUrdfLoadSpec(entry);
  const loaded = spec ? await handScene.loadHandUrdfDisplay(spec, HAND_VIEW_DIST_MUL) : null;
  if (loaded) {
    handVizState = { handId, ...loaded };
  }
  await refreshHandUrdfChartYLims(handId);
  if (loaded) tickVizUrdf();
}

async function refreshVizFromConfig() {
  vizConfig = await loadVizConfig();
  resetHandVizState();
  handScene?.invalidateUrdfLoads();
  resetAllVizResamplers();
  clearExoChartData();
  clearHandChartData();
  fillHandPicker(vizConfig);
  const exo = vizConfig.exo || {};
  if (exo.meshes && !exo.meshes.ready) {
    console.warn("exo meshes:", exo.meshes.message || t("viz.meshesMissing"));
    exoPreloaded = false;
  } else {
    exoRobot = await exoScene.loadUrdf(
      exo.urdf,
      exo.package,
      null,
      2.2,
      EXO_VIEW_OFFSET,
      EXO_VIEW_FILL
    );
    exoPreloaded = !!exoRobot;
    await refreshExoUrdfChartYLimsFromUrl();
  }
  await loadHandUrdf(resolveHandIdForViz(vizConfig));
}

window.resizeVizViewports = function resizeVizViewports() {
  exoScene?.resize();
  handScene?.resize();
  redrawCharts();
};

window.preloadExoVisualization = async function preloadExoVisualization() {
  const exoBox = document.getElementById("viz-exo-canvas");
  if (!exoBox) return;
  try {
    if (!exoScene) exoScene = new UrdfViewport(exoBox, { floorGrid: true });
    if (!vizConfig) vizConfig = await loadVizConfig();
    const exo = vizConfig.exo || {};
    if (exo.meshes && !exo.meshes.ready) {
      console.warn("exo meshes:", exo.meshes.message || t("viz.meshesMissing"));
      return;
    }
    exoRobot = await exoScene.loadUrdf(
      exo.urdf,
      exo.package,
      null,
      2.2,
      EXO_VIEW_OFFSET,
      EXO_VIEW_FILL
    );
    exoPreloaded = !!exoRobot;
    await refreshExoUrdfChartYLimsFromUrl();
  } catch (e) {
    console.warn("外骨骼 URDF 预加载失败", e);
  }
};

window.initVizPage = async function initVizPage() {
  const exoBox = document.getElementById("viz-exo-canvas");
  const handBox = document.getElementById("viz-hand-canvas");
  if (!exoBox || !handBox) return;

  if (!exoScene) exoScene = new UrdfViewport(exoBox, { floorGrid: true });
  else exoScene.resize();

  if (vizReady) {
    handScene?.resize();
    if (!exoPreloaded) await refreshVizFromConfig();
    else {
      vizConfig = await loadVizConfig();
      fillHandPicker(vizConfig);
      await loadHandUrdf(resolveHandIdForViz(vizConfig));
    }
    redrawCharts();
    startVizUrdfTimer();
    return;
  }

  handScene = new UrdfViewport(handBox);
  const pick = document.getElementById("viz-hand-pick");
  if (pick && !pick.dataset.bound) {
    pick.dataset.bound = "1";
    pick.addEventListener("change", (e) => loadHandUrdf(e.target.value));
  }
  window.addEventListener("gateway-ws", (ev) => onWsPayload(ev.detail));
  window.addEventListener("gateway-status", (ev) => onGatewayStatus(ev.detail));
  window.addEventListener("gateway-port-disconnect", onGatewayPortDisconnect);
  window.addEventListener("resize", () => window.resizeVizViewports?.());
  vizReady = true;

  if (exoPreloaded && vizConfig) {
    fillHandPicker(vizConfig);
    await loadHandUrdf(resolveHandIdForViz(vizConfig));
  } else {
    await refreshVizFromConfig();
  }
  redrawCharts();
  if (!window.__vizChartTimer) {
    window.__vizChartTimer = setInterval(redrawCharts, 200);
  }
  startVizUrdfTimer();
  initJointChartExpand();
};

function buildJointChartDrawOpts(def, legendEl) {
  return {
    legendNamesOnly: true,
    showYAxis: true,
    yTicks: 5,
    yLim: urdfChartYLim[def.yLimKey],
    legendEl,
  };
}

/** 灯箱图例面板高度与图表 canvas 底边对齐 */
function syncJointChartLightboxLayout() {
  if (!activeJointChartLightboxId) return;
  const lightboxCanvas = document.getElementById("chart-lightbox-joint");
  const legendEl = document.getElementById("chart-lightbox-legend");
  if (!lightboxCanvas || !legendEl) return;
  const h = lightboxCanvas.offsetHeight;
  if (h > 0) {
    legendEl.style.height = `${h}px`;
    legendEl.style.maxHeight = `${h}px`;
  }
}

function scheduleJointChartLightboxLayout() {
  requestAnimationFrame(() => {
    requestAnimationFrame(syncJointChartLightboxLayout);
  });
}

function redrawJointChartLightbox() {
  if (!activeJointChartLightboxId) return;
  const def = JOINT_CHART_DEFS[activeJointChartLightboxId];
  if (!def) return;
  const canvas = document.getElementById("chart-lightbox-joint");
  const legendEl = document.getElementById("chart-lightbox-legend");
  if (!canvas || !legendEl) return;
  legendEl.dataset.legendStateId = def.legendId;
  syncJointChartLightboxMeta();
  drawLineChart(
    canvas,
    chartBuffers[def.buffer],
    t(def.titleKey),
    def.yRangeKey,
    "rad",
    buildJointChartDrawOpts(def, legendEl)
  );
  scheduleJointChartLightboxLayout();
}

function syncJointChartLightboxMeta() {
  if (!activeJointChartLightboxId) return;
  const def = JOINT_CHART_DEFS[activeJointChartLightboxId];
  const metaEl = document.getElementById("viz-chart-lightbox-meta");
  if (!def || !metaEl) return;
  const srcMeta = def.metaId ? document.getElementById(def.metaId) : null;
  const text = srcMeta?.textContent?.trim();
  if (text && srcMeta && !srcMeta.hidden) {
    metaEl.textContent = text;
    metaEl.hidden = false;
  } else {
    metaEl.textContent = "";
    metaEl.hidden = true;
  }
}

function openJointChartLightbox(chartId) {
  const def = JOINT_CHART_DEFS[chartId];
  const root = document.getElementById("viz-chart-lightbox");
  if (!def || !root) return;
  activeJointChartLightboxId = chartId;
  const titleEl = document.getElementById("viz-chart-lightbox-title");
  const legendEl = document.getElementById("chart-lightbox-legend");
  if (titleEl) titleEl.textContent = t(def.titleKey);
  syncJointChartLightboxMeta();
  if (legendEl) legendEl.dataset.legendStateId = def.legendId;
  root.hidden = false;
  document.body.classList.add("viz-chart-lightbox-open");
  redrawJointChartLightbox();
  scheduleJointChartLightboxLayout();
  document.getElementById("viz-chart-lightbox-close")?.focus();
}

function closeJointChartLightbox() {
  const root = document.getElementById("viz-chart-lightbox");
  if (!root || root.hidden) return;
  const legendEl = document.getElementById("chart-lightbox-legend");
  if (legendEl) {
    legendEl.style.height = "";
    legendEl.style.maxHeight = "";
  }
  activeJointChartLightboxId = null;
  root.hidden = true;
  document.body.classList.remove("viz-chart-lightbox-open");
}

function initJointChartExpand() {
  if (window.__jointChartExpandReady) return;
  window.__jointChartExpandReady = true;

  for (const chartId of Object.keys(JOINT_CHART_DEFS)) {
    const canvas = document.getElementById(chartId);
    if (!canvas) continue;
    let wrap = canvas.closest(".viz-joint-chart-wrap");
    if (!wrap) {
      wrap = document.createElement("div");
      wrap.className = "viz-joint-chart-wrap";
      canvas.parentNode.insertBefore(wrap, canvas);
      wrap.appendChild(canvas);
    }
    if (wrap.querySelector(".viz-chart-expand-btn")) continue;
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "viz-chart-expand-btn";
    btn.setAttribute("aria-label", t("viz.chartExpand"));
    btn.innerHTML = CHART_EXPAND_ICON_SVG;
    btn.addEventListener("click", (e) => {
      e.stopPropagation();
      openJointChartLightbox(chartId);
    });
    wrap.appendChild(btn);
  }

  const root = document.getElementById("viz-chart-lightbox");
  if (!root) return;
  root.querySelectorAll("[data-lightbox-dismiss]").forEach((el) => {
    el.addEventListener("click", closeJointChartLightbox);
  });
  document.getElementById("viz-chart-lightbox-close")?.addEventListener("click", closeJointChartLightbox);
  if (!window.__jointChartLightboxKeyBound) {
    window.__jointChartLightboxKeyBound = true;
    window.addEventListener("keydown", (e) => {
      if (e.key === "Escape" && activeJointChartLightboxId) closeJointChartLightbox();
    });
    window.addEventListener("resize", () => {
      if (!activeJointChartLightboxId) return;
      scheduleJointChartLightboxLayout();
      redrawJointChartLightbox();
    });
  }
}

function refreshJointChartExpandLabels() {
  document.querySelectorAll(".viz-chart-expand-btn").forEach((btn) => {
    btn.setAttribute("aria-label", t("viz.chartExpand"));
  });
  const selectAllLabel = t("viz.chartLegendSelectAll");
  document.querySelectorAll(".viz-chart-legend-select-all-label").forEach((el) => {
    el.textContent = selectAllLabel;
  });
  document.querySelectorAll(".viz-chart-legend-select-all-toggle").forEach((el) => {
    el.setAttribute("aria-label", selectAllLabel);
  });
  const closeBtn = document.getElementById("viz-chart-lightbox-close");
  if (closeBtn) closeBtn.setAttribute("aria-label", t("viz.chartClose"));
  if (activeJointChartLightboxId) {
    const def = JOINT_CHART_DEFS[activeJointChartLightboxId];
    const titleEl = document.getElementById("viz-chart-lightbox-title");
    if (def && titleEl) titleEl.textContent = t(def.titleKey);
  }
}

function redrawCharts() {
  checkHandCmdStale();
  checkExoVibrationStale();
  const hz = pushRateSample(chartBuffers.exoRate, STREAM_EXO_JOINT, STREAM_EXO_JOINT);
  updateExoRateLabels(hz);
  const exoLeftYLim = urdfChartYLim.exoLeft;
  const exoRightYLim = urdfChartYLim.exoRight;
  drawLineChart(
    document.getElementById("chart-exo-left-joint"),
    chartBuffers.exoLeft,
    t("viz.chartExoLeft"),
    "exoLeft",
    "rad",
    { legendNamesOnly: true, showYAxis: true, yTicks: 5, yLim: exoLeftYLim }
  );
  drawLineChart(
    document.getElementById("chart-exo-right-joint"),
    chartBuffers.exoRight,
    t("viz.chartExoRight"),
    "exoRight",
    "rad",
    { legendNamesOnly: true, showYAxis: true, yTicks: 5, yLim: exoRightYLim }
  );
  drawLineChart(
    document.getElementById("chart-exo-rate"),
    chartBuffers.exoRate,
    t("viz.chartExoRate"),
    rateYAxisModes.exo === "auto" ? "exoRate" : null,
    "Hz",
    buildRateChartOpts(rateYAxisModes.exo)
  );
  updateExoVibrationLabels(computeStreamHz(STREAM_EXO_VIBRATION));
  drawExoVibrationChart(document.getElementById("chart-exo-vibration"));
  const leftHz = sampleHandRate("left");
  const rightHz = sampleHandRate("right");
  updateHandSideRateLabels("left", leftHz);
  updateHandSideRateLabels("right", rightHz);
  const handLeftYLim = urdfChartYLim.handLeft;
  const handRightYLim = urdfChartYLim.handRight;
  drawLineChart(
    document.getElementById("chart-hand-left-joint"),
    chartBuffers.handLeft,
    t("viz.chartHandLeft"),
    "handLeft",
    "rad",
    { legendNamesOnly: true, showYAxis: true, yTicks: 5, yLim: handLeftYLim }
  );
  drawLineChart(
    document.getElementById("chart-hand-right-joint"),
    chartBuffers.handRight,
    t("viz.chartHandRight"),
    "handRight",
    "rad",
    { legendNamesOnly: true, showYAxis: true, yTicks: 5, yLim: handRightYLim }
  );
  drawLineChart(
    document.getElementById("chart-hand-left-rate"),
    chartBuffers.handLeftRate,
    t("viz.chartHandLeftRate"),
    rateYAxisModes.handLeft === "auto" ? "handLeftRate" : null,
    "Hz",
    buildRateChartOpts(rateYAxisModes.handLeft)
  );
  drawLineChart(
    document.getElementById("chart-hand-right-rate"),
    chartBuffers.handRightRate,
    t("viz.chartHandRightRate"),
    rateYAxisModes.handRight === "auto" ? "handRightRate" : null,
    "Hz",
    buildRateChartOpts(rateYAxisModes.handRight)
  );
  redrawJointChartLightbox();
}

window.refreshVizPage = refreshVizFromConfig;

function resolveChartYTickValues(opts, ymin, ymax) {
  if (opts.yTickValues?.length) {
    return opts.yTickValues.filter((v) => v >= ymin && v <= ymax);
  }
  if (opts.rateYAxisAnchored) {
    return rateYTickValuesForRange(ymin, ymax);
  }
  return null;
}

function buildRateChartOpts(mode) {
  const opts = { showYAxis: true, yTicks: 5, externalRateLegend: true, rateYAxisAnchored: true };
  if (mode === "fixed") {
    opts.strictYLim = RATE_Y_FIXED;
    opts.yTickValues = RATE_Y_TICK_VALUES;
  }
  return opts;
}

function initRateYAxisOptions() {
  for (const { name, modeKey, stableKey } of RATE_YAXIS_RADIO_GROUPS) {
    const radios = document.querySelectorAll(`input[name="${name}"]`);
    if (!radios.length) continue;
    for (const el of radios) {
      if (el.checked) rateYAxisModes[modeKey] = el.value;
      el.addEventListener("change", () => {
        if (!el.checked) return;
        rateYAxisModes[modeKey] = el.value;
        if (el.value === "auto") chartYStable[stableKey] = null;
        redrawCharts();
      });
    }
  }
}

function refreshVizHandPickerLabels() {
  const pick = document.getElementById("viz-hand-pick");
  if (pick) pick.setAttribute("aria-label", t("viz.handPickLabel"));
  if (vizConfig) fillHandPicker(vizConfig);
}

const prevOnGatewayLangChange = window.onGatewayLangChange;
window.onGatewayLangChange = function onGatewayLangChangeWithViz() {
  prevOnGatewayLangChange?.();
  refreshVizHandPickerLabels();
  refreshJointChartExpandLabels();
  if (typeof redrawCharts === "function") redrawCharts();
};

async function bootVizModule() {
  initRateYAxisOptions();
  initJointChartExpand();
  try {
    await window.preloadExoVisualization();
  } catch (e) {
    console.warn("exo preload", e);
  }
  try {
    await window.initVizPage();
    if (typeof window.resizeVizViewports === "function") {
      requestAnimationFrame(() => window.resizeVizViewports());
      setTimeout(() => window.resizeVizViewports(), 400);
    }
  } catch (e) {
    console.warn("viz init", e);
  }
}

if (document.readyState === "loading") {
  document.addEventListener("DOMContentLoaded", bootVizModule);
} else {
  bootVizModule();
}
