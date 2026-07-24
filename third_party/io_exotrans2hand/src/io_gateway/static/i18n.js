/**
 * io_gateway UI 中英文文案
 */
const LANG_STORAGE_KEY = "io_gateway_lang";

/** 后端 UDP API 稳定错误码 → i18n 键 */
const UDP_API_CODES = {
  udp_already_running: "api.udp.alreadyRunning",
  udp_serial_conflict: "api.udp.serialConflict",
  udp_no_network: "api.udp.noNetwork",
  udp_bind_ip_mismatch: "api.udp.bindIpMismatch",
  udp_no_device: "api.udp.noDevice",
  udp_not_running: "api.udp.notRunning",
  udp_started: "udp.startSuccess",
  udp_stopped: "udp.stopSuccess",
  udp_start_failed: "udp.startFailed",
  udp_stop_failed: "udp.stopFailed",
  udp_busy: "api.udp.busy",
};

function udpApiCodeKey(code) {
  return UDP_API_CODES[code] || null;
}

function extractApiCode(payload) {
  if (!payload || typeof payload !== "object") return null;
  if (typeof payload.code === "string") return payload.code;
  const d = payload.detail;
  if (d && typeof d === "object" && typeof d.code === "string") return d.code;
  return null;
}

const I18N = {
  zh: {
    "page.title": "IO Gesture",
    tagline: "外骨骼手套 · 灵巧手操控平台",
    "nav.home": "首页",
    "nav.viz": "可视化",
    "section.exoModule": "外骨骼",
    "section.handModule": "灵巧手",
    "section.modelConfig": "型号配置",
    "section.monitor": "系统监控",
    "viz.exoModel": "外骨骼 URDF",
    "viz.handModel": "灵巧手型号 URDF",
    "viz.chartExoJoint": "外骨骼关节数据",
    "viz.chartExoLeft": "左侧外骨骼关节",
    "viz.chartExoRight": "右侧外骨骼关节",
    "viz.chartExoSideMetaEmpty": "关节：等待 io_esk.joint_data 数据…",
    "viz.chartExoSideMeta": "关节数：{count} · 曲线：{joints} · 末帧末关节：{last} rad",
    "viz.chartExoRate": "数据输出频率",
    "viz.chartRateYFixed": "固定",
    "viz.chartRateYAuto": "动态",
    "viz.chartExoVibration": "振动反馈强度",
    "viz.chartExoVibrationMetaEmpty": "当前约 0 Hz（等待 io_esk.vibration_feedback）",
    "viz.chartExoVibrationAxisX": "末端",
    "viz.chartExoVibrationAxisY": "数值",
    "viz.chartExoRateSource": "数据源：io_esk.joint_data（WebSocket 推送）",
    "viz.chartRateWindow": "滑动 1s 窗口",
    "viz.chartRateMeta": "当前约 {hz} {unit}",
    "viz.chartRateMetaEmpty": "当前约 0 Hz（等待 io_esk.joint_data）",
    "viz.chartDataRange": "曲线范围 {min}–{max} {unit}",
    "viz.chartUrdfRange": "URDF 限位 {min}–{max} {unit}",
    "viz.chartHandJoint": "灵巧手关节数据（合并）",
    "viz.chartHandLeft": "左手关节数据",
    "viz.chartHandRight": "右手关节数据",
    "viz.chartHandLeftRate": "左手输出频率",
    "viz.chartHandRightRate": "右手输出频率",
    "viz.chartHandRateMetaEmptyLeft": "当前约 0 Hz（等待 io_teleop.joint_cmd_left）",
    "viz.chartHandRateMetaEmptyRight": "当前约 0 Hz（等待 io_teleop.joint_cmd_right）",
    "viz.handPickLabel": "已选型号",
    "viz.noHandSelected": "（请先应用型号）",
    "viz.chartHandSideMetaEmpty": "关节：等待 cmd 数据…",
    "viz.chartHandSideMeta": "关节数：{count} · 曲线：{joints} · 末帧末关节：{last} rad",
    "viz.loading": "加载模型…",
    "viz.loadFailed": "模型加载失败",
    "viz.noUrdf": "无可用 URDF",
    "viz.chartWaiting": "等待 WebSocket 数据…",
    "viz.chartSource": "数据源",
    "viz.chartSourcePending": "数据源：—",
    "viz.chartLegendHint": "右侧：色块=曲线，上行=关节名，下行=数据流",
    "viz.chartLegendScroll": "（勾选显示曲线，滚动查看全部图例）",
    "viz.chartLegendToggle": "显示曲线：{name}",
    "viz.chartLegendSelectAll": "全选",
    "viz.chartExpand": "放大图表",
    "viz.chartClose": "关闭",
    "viz.chartHandIsCmd": "IK 关节指令（非实测反馈）",
    "viz.chartHandBimanual": "已合并 joint_cmd_left + joint_cmd_right",
    "viz.handBimanualUrdf": "双手 URDF",
    "viz.meshesMissing": "缺少外骨骼 STL 网格：请将 meshes 放到 configs/exoskeleton_urdf/meshes/",
    "hand.currentLabel": "当前型号",
    "hand.hint": "（勾选后点「应用」；不勾选再点应用将清除已选型号）",
    "hand.none": "未选择",
    "hand.savedPending": "（已保存，等待外骨骼）",
    "hand.notReady": "（进程未就绪）",
    "hand.noHands": "（无可用型号）",
    "panel.exoPorts": "外骨骼串口",
    "panel.deviceConnect": "设备连接",
    "deviceConnect.hintInline": "（左右手端口查询间隔为 3s）",
    "section.wired": "外骨骼连接状态",
    "deviceStatus.hintAria": "查看设备连接说明",
    "deviceStatus.hint": "外骨骼有线连接会自动扫描并识别左右手端口；无线配网后自动发现模块并启动 UDP 接收",
    "section.wireless": "无线模块配网",
    "section.broadcastConfig": "广播配置",
    "section.wirelessModuleStatus": "无线模块状态",
    "field.wifiModulesOnline": "在线设备 IP",
    "field.wifiIp1": "IP1",
    "field.wifiIp2": "IP2",
    "wifi.modulesNone": "无在线设备",
    "wifi.noOnlineDevice": "未检测到在线设备",
    "wifi.ipNotConnected": "(未连接)",
    "wifi.moduleStatusNone": "未检测到无线模块",
    "wifi.moduleStatusWaiting": "等待连接外骨骼手套",
    "wifi.moduleStatusReceiving": "数据接收中...",
    "wifi.moduleStatusConnected": "已连接外骨骼手套（数据接收中...）",
    "udp.autoScanning": "模块在线，自动扫描中",
    "field.wifiSsid": "SSID：",
    "wifiSsid.placeholder": "IO-AI-2.4G",
    "field.wifiPassword": "密码：",
    "wifiPassword.placeholder": "路由器密码",
    "wifiPassword.show": "显示密码",
    "wifiPassword.hide": "隐藏密码",
    "field.returnIp": "回调 IP：",
    "field.moduleStatus": "状态",
    "returnIp.placeholder": "10.42.0.1",
    "field.useBroadcast": "使用广播模式（ESPTouch）",
    "btn.wifiProvision": "开始配网",
    "btn.wifiProvisionBusy": "配网中…",
    "btn.wifiSave": "保存网络",
    "btn.wifiSaveTitle": "保存网络配置刷新自动填充",
    "btn.wifiSaveBusy": "保存中…",
    "wifi.configSaved": "配置已保存",
    "wifi.configSaveFailed": "保存失败：{detail}",
    "wifi.missingSsid": "请填写 SSID",
    "wifi.missingPassword": "请填写 WiFi 密码",
    "wifi.passwordTooShort": "密码至少 {min} 位",
    "wifi.missingReturnIp": "请填写回调 IP",
    "wifi.invalidReturnIp": "回调 IP 格式无效",
    "wifi.invalidReturnIpDetail": "回调 IP 格式无效：{ip}",
    "wifi.hintAria": "查看无线配网说明",
    "wifi.provisionHint":
      "回调 IP 须填路由器/网关地址，勿填本机 IP，也不要把 PC 地址当成网关填入。配网前请确认本机已连接目标 WiFi，且能 ping 通该网关地址。",
    "wifi.errMacLookup":
      "无法获取 {ip} 的路由器 MAC。请确认：① 本机已连接目标 WiFi；② 回调 IP 为网关地址（非本机 IP）；③ 先 ping 通 {ip} 后再试。",
    "wifi.errSsidRequired": "请设置 SSID",
    "wifi.errIpInvalid": "回调 IP 无效",
    "wifi.errProvisionDetail": "配网失败：{detail}",
    "wifi.provisionFailed": "配网失败：{detail}",
    "wifi.provisionSuccess": "配网信息广播成功",
    "api.udp.alreadyRunning": "已在运行",
    "api.udp.serialConflict": "请先断开有线外骨骼",
    "api.udp.noNetwork": "本机未配置 10.42.x 网段，无法启动 UDP 外骨骼",
    "api.udp.bindIpMismatch": "bind_ip 与本机地址不符",
    "api.udp.noDevice": "未检测到外骨骼设备",
    "api.udp.notRunning": "未启动",
    "api.udp.startFailedGeneric": "启动失败",
    "api.udp.busy": "请稍候",
    "api.unknownHands": "未知手型：{hands}",
    "api.handNotEnabled": "手型未启用：{hand}",
    "api.multiHandSync": "多手型运行时请指定 hand 字段",
    "api.writeConfigFailed": "写入配置失败：{detail}",
    "api.rawDetail": "{detail}",
    "btn.udpExoStart": "启动 UDP 接收",
    "btn.udpExoStartBusy": "启动中…",
    "btn.udpExoStop": "停止 UDP 接收",
    "btn.udpExoStopBusy": "停止中…",
    "udp.running": "UDP 外骨骼接收中",
    "udp.stopped": "UDP 外骨骼未启动",
    "udp.startSuccess": "已启动",
    "udp.stopSuccess": "已停止",
    "udp.startFailed": "启动失败",
    "udp.stopFailed": "停止失败",
    "ports.wirelessMode": "无线模式（有线已忽略）",
    "panel.handSelect": "型号选择（可多选）",
    "ports.topologyLabel": "探测拓扑",
    "ports.sideLeft": "左手",
    "ports.sideRight": "右手",
    "ports.na": "未连接",
    "ports.connected": "已连接",
    "ports.topo.none": "无设备",
    "toast.portDisconnected": "外骨骼 {side} 已断开：{port}",
    "toast.portConnected": "外骨骼 {side} 已连接：{port}",
    "toast.portSwitched": "外骨骼 {side} 端口已切换：{from} → {to}",
    "ports.topo.left": "仅左手",
    "ports.topo.right": "仅右手",
    "ports.topo.both": "双手",
    "panel.control": "控制",
    "section.hands": "型号",
    "field.side": "探测侧别（只读，跟随外骨骼）",
    "btn.apply": "应用",
    "btn.applyBusy": "应用中…",
    "btn.refreshHands": "刷新",
    "btn.refreshHandsTitle": "刷新型号列表",
    "panel.upload": "上传灵巧手型号配置",
    "upload.desc":
      "文件夹与压缩包须为同一标准结构：<code>型号名/urdf/</code>、<code>型号名/meshes/</code> 及 3 个必选 yml（与压缩包内一致）。请直接选择或拖入<strong>型号根目录</strong>，不会自动调整路径；型号名取顶层文件夹名（英文字母、数字、下划线）。",
    "upload.hintAria": "查看上传说明",
    "field.handName": "识别型号名（自动填写，只读）",
    "handName.placeholder": "选择配置后自动填写",
    "upload.clearAria": "清除已选配置",
    "upload.clearTitle": "清除选择",
    "upload.selectedArchive": "已选压缩包：{name}",
    "upload.selectedFolder": "已选文件夹：{name}",
    "upload.selectedMeta": "{count} 个文件",
    "upload.drop":
      '<p class="upload-drop-main">拖放压缩包或型号根文件夹到此处</p><p class="upload-drop-hint">压缩包：zip、tar.gz、tgz 等（须含一层型号名目录）</p><p class="upload-drop-hint">文件夹：请选 <code>型号名/</code> 根目录 · 点击选压缩包 · <kbd>Shift</kbd>+点击选文件夹</p>',
    "btn.upload": "上传配置",
    "btn.uploadBusy": "上传中…",
    "terminal.status": "状态",
    "status.apiPath": "GET /api/v1/status",
    "terminal.ws": "WebSocket 数据",
    "ws.subscribeHint":
      "页面默认订阅 io_esk.joint_data、左右 IMU 与已应用型号的 joint_cmd_left/joint_cmd_right；全部流见 GET /api/v1/streams。",
    "ws.streamPick": "数据流",
    "ws.noStream": "（暂无数据流）",
    "ws.waitingData": "等待数据…",
    "ws.noDataYet": "该流暂无数据",
    "status.loading": "加载中…",
    "status.offline": "无法连接网关，请确认 io_gateway 已启动",
    "status.httpError": "状态请求失败（HTTP {code}）：{detail}",
    "status.badResponse": "网关返回了无效的状态数据",
    "status.recovered": "已恢复与网关的连接",
    "status.staleHint": "⚠ {error}\n以下为上次成功获取的状态（可能已过期）：",
    "ws.disconnected": "未连接",
    "ws.reconnecting": "连接断开，正在重连…",
    "confirm.overwrite":
      "型号名「{name}」已存在。\n\n是否覆盖 configs/end_tools 中的原有配置？\n选择「确定」覆盖，「取消」放弃上传。",
    "confirm.overwriteFallback": "该型号",
    "confirm.clearHands":
      "未勾选型号。\n\n是否清除已应用的型号并停止 transform / controller？\n选择「确定」清除，「取消」保持不变。",
    "err.noFiles": "未包含任何有效文件",
    "err.badFolder":
      "文件夹结构不正确：须只有一个顶层文件夹（型号名），其下直接含 urdf/、meshes/ 与 3 个 yml；请勿选外层父目录，也不要只选 urdf/ 或 meshes/ 子文件夹",
    "err.missingUrdf": "缺少 {root}/urdf/ 目录（或目录内无文件）",
    "err.missingMeshes": "缺少 {root}/meshes/ 目录（或目录内无文件）",
    "err.missingYml": "缺少 {root}/{yml}",
    "err.invalidName":
      "型号名「{name}」无效：仅允许英文字母、数字和下划线（请重命名顶层文件夹）",
    "alert.folderInvalid": "配置文件夹不符合要求：\n\n{errors}",
    "alert.unrecognized":
      "无法识别上传内容：请选择文件夹，或 zip / tar / tar.gz / tgz 等压缩包",
    "alert.emptyFolder": "所选文件夹为空",
    "alert.dropFailed": "读取拖放内容失败：{msg}",
    "alert.dropFolderName": "请拖入或选择型号根文件夹（顶层文件夹名即型号名，结构与压缩包内一致）",
    "alert.pickFirst": "请先选择或拖放型号配置（压缩包或文件夹）",
    "alert.pickHand": "请先勾选至少一个型号",
    "alert.applyFailed": "应用型号失败：{detail}",
    "alert.uploadFailed": "上传失败：{detail}",
    "upload.pickFirst": "请先选择配置",
    "upload.cancelled": "已取消上传",
    "upload.validationFailed": "配置校验未通过",
    "upload.uploading": "正在上传…",
    "upload.overwriteHint": "（将覆盖已有配置）",
    "upload.recognizedFolder": "已识别型号：{name}（文件夹）{hint}，可以上传",
    "upload.recognizedArchive": "已选择压缩包：{name}{hint}，可以上传",
    "upload.success": "上传成功：{hand}（已加入型号列表）{overwrote}",
    "upload.overwrote": "（已覆盖原配置）",
    "upload.failed": "上传失败：{detail}",
  },
  en: {
    "page.title": "IO Gesture",
    tagline: "Exoskeleton Glove · Dexterous Hand Control",
    "nav.home": "Home",
    "nav.viz": "Visualization",
    "section.exoModule": "Exoskeleton",
    "section.handModule": "Dexterous Hand",
    "section.modelConfig": "Model config",
    "section.monitor": "System Monitor",
    "viz.exoModel": "Exoskeleton URDF",
    "viz.handModel": "Hand Model URDF",
    "viz.chartExoJoint": "Exo joint data",
    "viz.chartExoLeft": "Left exo joints",
    "viz.chartExoRight": "Right exo joints",
    "viz.chartExoSideMetaEmpty": "Joints: waiting for io_esk.joint_data…",
    "viz.chartExoSideMeta": "Joints: {count} · series: {joints} · last: {last} rad",
    "viz.chartExoRate": "Output rate",
    "viz.chartRateYFixed": "Fixed",
    "viz.chartRateYAuto": "Auto",
    "viz.chartExoVibration": "Vibration feedback (io_esk.vibration_feedback)",
    "viz.chartExoVibrationMetaEmpty": "Current ~0 Hz (waiting for io_esk.vibration_feedback)",
    "viz.chartExoVibrationAxisX": "Endpoint",
    "viz.chartExoVibrationAxisY": "Value",
    "viz.chartExoRateSource": "Source: io_esk.joint_data (WebSocket)",
    "viz.chartRateWindow": "1s sliding window",
    "viz.chartRateMeta": "Current ~{hz} {unit}",
    "viz.chartRateMetaEmpty": "Current ~0 Hz (waiting for io_esk.joint_data)",
    "viz.chartDataRange": "Series range {min}–{max} {unit}",
    "viz.chartUrdfRange": "URDF limits {min}–{max} {unit}",
    "viz.chartHandJoint": "Hand joints (merged)",
    "viz.chartHandLeft": "Left hand joints",
    "viz.chartHandRight": "Right hand joints",
    "viz.chartHandLeftRate": "Left output rate",
    "viz.chartHandRightRate": "Right output rate",
    "viz.chartHandRateMetaEmptyLeft": "Current ~0 Hz (waiting for io_teleop.joint_cmd_left)",
    "viz.chartHandRateMetaEmptyRight": "Current ~0 Hz (waiting for io_teleop.joint_cmd_right)",
    "viz.handPickLabel": "Selected model",
    "viz.noHandSelected": "(Apply a model first)",
    "viz.chartHandSideMetaEmpty": "Joints: waiting for cmd…",
    "viz.chartHandSideMeta": "Joints: {count} · series: {joints} · last: {last} rad",
    "viz.loading": "Loading model…",
    "viz.loadFailed": "Failed to load model",
    "viz.noUrdf": "No URDF available",
    "viz.chartWaiting": "Waiting for WebSocket data…",
    "viz.chartSource": "Source",
    "viz.chartSourcePending": "Source: —",
    "viz.chartLegendHint": "Right: color=series, line1=joint, line2=stream",
    "viz.chartLegendScroll": "(check to show series, scroll for all)",
    "viz.chartLegendToggle": "Show series: {name}",
    "viz.chartLegendSelectAll": "Select all",
    "viz.chartExpand": "Expand chart",
    "viz.chartClose": "Close",
    "viz.chartHandIsCmd": "IK joint command (not measured feedback)",
    "viz.chartHandBimanual": "merged joint_cmd_left + joint_cmd_right",
    "viz.handBimanualUrdf": "bimanual URDF",
    "viz.meshesMissing": "Exo STL meshes missing: add files under configs/exoskeleton_urdf/meshes/",
    "hand.currentLabel": "Current model(s)",
    "hand.hint": "(Select models and Apply; Apply with none selected clears applied models)",
    "hand.none": "None",
    "hand.savedPending": "(saved, waiting for exo)",
    "hand.notReady": "(processes not ready)",
    "hand.noHands": "(no models available)",
    "panel.exoPorts": "Exoskeleton ports",
    "panel.deviceConnect": "Device connection",
    "deviceConnect.hintInline": " (Left/right port probe interval: 3s)",
    "section.wired": "Exo connection status",
    "deviceStatus.hintAria": "Show device connection help",
    "deviceStatus.hint": "Wired exo gloves are auto-scanned for left/right ports. After wireless provisioning, modules are discovered automatically and UDP receiving starts.",
    "section.wireless": "Wireless provisioning",
    "section.broadcastConfig": "Broadcast",
    "section.wirelessModuleStatus": "Wireless module status",
    "field.wifiModulesOnline": "Online device IP",
    "field.wifiIp1": "IP1",
    "field.wifiIp2": "IP2",
    "wifi.modulesNone": "No devices online",
    "wifi.noOnlineDevice": "No online device detected",
    "wifi.ipNotConnected": "(not connected)",
    "wifi.moduleStatusNone": "No wireless module detected",
    "wifi.moduleStatusWaiting": "Waiting for exoskeleton glove connection",
    "wifi.moduleStatusReceiving": "Receiving data...",
    "wifi.moduleStatusConnected": "Exoskeleton glove connected (receiving data...)",
    "udp.autoScanning": "Modules online, auto-scanning",
    "field.wifiSsid": "SSID:",
    "wifiSsid.placeholder": "IO-AI-2.4G",
    "field.wifiPassword": "Password:",
    "wifiPassword.placeholder": "Router password",
    "wifiPassword.show": "Show password",
    "wifiPassword.hide": "Hide password",
    "field.returnIp": "Return IP:",
    "field.moduleStatus": "Status",
    "returnIp.placeholder": "10.42.0.1",
    "field.useBroadcast": "Use broadcast mode (ESPTouch)",
    "btn.wifiProvision": "Start provisioning",
    "btn.wifiProvisionBusy": "Provisioning…",
    "btn.wifiSave": "Save network",
    "btn.wifiSaveTitle": "Save network config; refresh to auto-fill",
    "btn.wifiSaveBusy": "Saving…",
    "wifi.configSaved": "Configuration saved",
    "wifi.configSaveFailed": "Save failed: {detail}",
    "wifi.missingSsid": "SSID is required",
    "wifi.missingPassword": "WiFi password is required",
    "wifi.passwordTooShort": "Password must be at least {min} characters",
    "wifi.missingReturnIp": "Return IP is required",
    "wifi.invalidReturnIp": "Invalid return IP format",
    "wifi.invalidReturnIpDetail": "Invalid return IP: {ip}",
    "wifi.hintAria": "Show wireless provisioning help",
    "wifi.provisionHint":
      "Return IP must be the router/gateway address, not this PC's IP—do not enter your host address as if it were the gateway. Connect to the target WiFi first and ping that gateway before provisioning.",
    "wifi.errMacLookup":
      "Could not resolve router MAC for {ip}. Check: ① connected to target WiFi; ② return IP is the gateway (not this PC); ③ ping {ip} first.",
    "wifi.errSsidRequired": "SSID is required",
    "wifi.errIpInvalid": "Invalid return IP",
    "wifi.errProvisionDetail": "Provisioning failed: {detail}",
    "wifi.provisionFailed": "Provisioning failed: {detail}",
    "wifi.provisionSuccess": "Provisioning broadcast successful",
    "api.udp.alreadyRunning": "Already running",
    "api.udp.serialConflict": "Disconnect wired exo first",
    "api.udp.noNetwork": "No 10.42.x network on this host; cannot start UDP exo",
    "api.udp.bindIpMismatch": "bind_ip mismatch",
    "api.udp.noDevice": "No exo device detected",
    "api.udp.notRunning": "Not running",
    "api.udp.startFailedGeneric": "Start failed",
    "api.udp.busy": "Please wait",
    "api.unknownHands": "Unknown model(s): {hands}",
    "api.handNotEnabled": "Model not enabled: {hand}",
    "api.multiHandSync": "Specify the hand field when multiple models are active",
    "api.writeConfigFailed": "Failed to write config: {detail}",
    "api.rawDetail": "{detail}",
    "btn.udpExoStart": "Start UDP receiver",
    "btn.udpExoStartBusy": "Starting…",
    "btn.udpExoStop": "Stop UDP receiver",
    "btn.udpExoStopBusy": "Stopping…",
    "udp.running": "UDP exo receiver running",
    "udp.stopped": "UDP exo receiver stopped",
    "udp.startSuccess": "Started",
    "udp.stopSuccess": "Stopped",
    "udp.startFailed": "Start failed",
    "udp.stopFailed": "Stop failed",
    "ports.wirelessMode": "Wireless mode (wired ignored)",
    "panel.handSelect": "Model selection (multi-select)",
    "ports.topologyLabel": "Topology",
    "ports.sideLeft": "Left",
    "ports.sideRight": "Right",
    "ports.na": "Not connected",
    "ports.connected": "Connected",
    "ports.topo.none": "No device",
    "toast.portDisconnected": "Exo {side} disconnected: {port}",
    "toast.portConnected": "Exo {side} connected: {port}",
    "toast.portSwitched": "Exo {side} port changed: {from} → {to}",
    "ports.topo.left": "Left only",
    "ports.topo.right": "Right only",
    "ports.topo.both": "Both hands",
    "panel.control": "Control",
    "section.hands": "Model",
    "field.side": "Detected side (read-only, from exoskeleton)",
    "btn.apply": "Apply",
    "btn.applyBusy": "Applying…",
    "btn.refreshHands": "Refresh",
    "btn.refreshHandsTitle": "Refresh model list",
    "panel.upload": "Upload hand model config",
    "upload.desc":
      "Folder and archive must share the same layout: <code>ModelName/urdf/</code>, <code>ModelName/meshes/</code>, and 3 required yml files (same as inside a zip). Pick or drop the <strong>model root folder</strong> directly — paths are not auto-adjusted. Model name is the top-level folder (letters, digits, underscore).",
    "upload.hintAria": "Show upload help",
    "field.handName": "Detected model name (auto, read-only)",
    "handName.placeholder": "Filled after you select a package",
    "upload.clearAria": "Clear selected package",
    "upload.clearTitle": "Clear selection",
    "upload.selectedArchive": "Archive: {name}",
    "upload.selectedFolder": "Folder: {name}",
    "upload.selectedMeta": "{count} files",
    "upload.drop":
      '<p class="upload-drop-main">Drop an archive or model root folder here</p><p class="upload-drop-hint">Archive: zip, tar.gz, tgz, etc. (must include one top-level model folder)</p><p class="upload-drop-hint">Folder: pick the <code>ModelName/</code> root · click for archive · <kbd>Shift</kbd>+click for folder</p>',
    "btn.upload": "Upload",
    "btn.uploadBusy": "Uploading…",
    "terminal.status": "Status",
    "status.apiPath": "GET /api/v1/status",
    "terminal.ws": "WebSocket",
    "ws.subscribeHint":
      "UI subscribes to io_esk.joint_data, left/right IMU, and joint_cmd_left/joint_cmd_right for applied models; full catalog: GET /api/v1/streams.",
    "ws.streamPick": "Stream",
    "ws.noStream": "(no streams)",
    "ws.waitingData": "Waiting for data…",
    "ws.noDataYet": "No data on this stream yet",
    "status.loading": "Loading…",
    "status.offline": "Cannot reach gateway — ensure io_gateway is running",
    "status.httpError": "Status request failed (HTTP {code}): {detail}",
    "status.badResponse": "Gateway returned invalid status data",
    "status.recovered": "Connection to gateway restored",
    "status.staleHint": "⚠ {error}\nLast successful status below (may be stale):",
    "ws.disconnected": "Disconnected",
    "ws.reconnecting": "Disconnected — reconnecting…",
    "confirm.overwrite":
      'Model "{name}" already exists.\n\nOverwrite the config under configs/end_tools?\nOK to overwrite, Cancel to abort.',
    "confirm.overwriteFallback": "this model",
    "confirm.clearHands":
      "No model selected.\n\nClear applied models and stop transform / controller?\nOK to clear, Cancel to keep current.",
    "err.noFiles": "No valid files included",
    "err.badFolder":
      "Invalid folder layout: exactly one top-level folder (model name) with urdf/, meshes/, and 3 yml files directly inside. Do not pick a parent folder or only urdf/ or meshes/.",
    "err.missingUrdf": "Missing {root}/urdf/ (or empty)",
    "err.missingMeshes": "Missing {root}/meshes/ (or empty)",
    "err.missingYml": "Missing {root}/{yml}",
    "err.invalidName":
      'Invalid model name "{name}": use letters, digits, and underscores only (rename the top folder)',
    "alert.folderInvalid": "Package folder is invalid:\n\n{errors}",
    "alert.unrecognized":
      "Unrecognized upload: choose a folder, or zip / tar / tar.gz / tgz archive",
    "alert.emptyFolder": "Selected folder is empty",
    "alert.dropFailed": "Failed to read dropped items: {msg}",
    "alert.dropFolderName":
      "Pick or drop the model root folder (top folder name = model name; same layout as inside a zip).",
    "alert.pickFirst": "Select or drop a model config (archive or folder) first",
    "alert.pickHand": "Select at least one model",
    "alert.applyFailed": "Apply model failed: {detail}",
    "alert.uploadFailed": "Upload failed: {detail}",
    "upload.pickFirst": "Select a config first",
    "upload.cancelled": "Upload cancelled",
    "upload.validationFailed": "Validation failed",
    "upload.uploading": "Uploading…",
    "upload.overwriteHint": " (will overwrite existing)",
    "upload.recognizedFolder": "Detected: {name} (folder){hint} — ready to upload",
    "upload.recognizedArchive": "Archive selected: {name}{hint} — ready to upload",
    "upload.success": "Uploaded: {hand} (added to model list){overwrote}",
    "upload.overwrote": " (overwritten)",
    "upload.failed": "Upload failed: {detail}",
  },
};

let currentLang =
  localStorage.getItem(LANG_STORAGE_KEY) ||
  (navigator.language.toLowerCase().startsWith("zh") ? "zh" : "en");

/** @param {string} key @param {Record<string, string>} [params] */
function t(key, params = {}) {
  let s = I18N[currentLang]?.[key] ?? I18N.zh[key] ?? key;
  for (const [k, v] of Object.entries(params)) {
    s = s.replaceAll(`{${k}}`, String(v));
  }
  return s;
}

/** 将 API/后端 detail 统一转为可读字符串 */
function normalizeDetailText(text) {
  if (text == null || text === "") return "";
  if (typeof text === "string") return text.trim();
  if (Array.isArray(text)) {
    return text
      .map((item) => {
        if (typeof item === "string") return item;
        if (item && typeof item === "object") {
          return item.msg || item.message || JSON.stringify(item);
        }
        return String(item);
      })
      .filter(Boolean)
      .join("；");
  }
  if (typeof text === "object") {
    if (typeof text.message === "string") return text.message;
    if (typeof text.msg === "string") return text.msg;
    try {
      return JSON.stringify(text);
    } catch (_) {
      return String(text);
    }
  }
  return String(text).trim();
}

/** 后端/API 原文 → i18n 键（供状态持久化与语言切换） */
function matchApiDetail(text) {
  const s = normalizeDetailText(text);
  if (!s) return null;

  const exact = {
    udp_already_running: { key: "api.udp.alreadyRunning", params: {} },
    udp_serial_conflict: { key: "api.udp.serialConflict", params: {} },
    udp_no_network: { key: "api.udp.noNetwork", params: {} },
    udp_bind_ip_mismatch: { key: "api.udp.bindIpMismatch", params: {} },
    udp_no_device: { key: "api.udp.noDevice", params: {} },
    udp_not_running: { key: "api.udp.notRunning", params: {} },
    udp_started: { key: "udp.startSuccess", params: {} },
    udp_stopped: { key: "udp.stopSuccess", params: {} },
    udp_busy: { key: "api.udp.busy", params: {} },
    "配网信息广播成功": { key: "wifi.provisionSuccess", params: {} },
    "Provisioning broadcast successful": { key: "wifi.provisionSuccess", params: {} },
    "SSID 不能为空": { key: "wifi.missingSsid", params: {} },
    "SSID is required": { key: "wifi.missingSsid", params: {} },
    "回调 IP 不能为空": { key: "wifi.missingReturnIp", params: {} },
    "Return IP is required": { key: "wifi.missingReturnIp", params: {} },
    "请设置 SSID。": { key: "wifi.errSsidRequired", params: {} },
    "IP address invalid": { key: "wifi.errIpInvalid", params: {} },
    "UDP 外骨骼已在运行": { key: "api.udp.alreadyRunning", params: {} },
    "UDP exo receiver already running": { key: "api.udp.alreadyRunning", params: {} },
    "检测到有线外骨骼正在运行，请先拔掉有线设备后再启动无线模式": {
      key: "api.udp.serialConflict",
      params: {},
    },
    "Wired exo is active. Disconnect wired devices before starting wireless mode": {
      key: "api.udp.serialConflict",
      params: {},
    },
    "未检测到 UDP 外骨骼设备，请确认手套已连接并发送数据": {
      key: "api.udp.noDevice",
      params: {},
    },
    "No UDP exo device detected. Confirm the glove is connected and sending data": {
      key: "api.udp.noDevice",
      params: {},
    },
    "UDP 外骨骼已启动": { key: "udp.startSuccess", params: {} },
    "UDP exo receiver started": { key: "udp.startSuccess", params: {} },
    "UDP 外骨骼已停止": { key: "udp.stopSuccess", params: {} },
    "UDP exo receiver stopped": { key: "udp.stopSuccess", params: {} },
    "UDP 外骨骼未在运行": { key: "api.udp.notRunning", params: {} },
    "UDP exo receiver is not running": { key: "api.udp.notRunning", params: {} },
    "无法启动 UDP 外骨骼": { key: "api.udp.startFailedGeneric", params: {} },
    "Failed to start UDP exo receiver": { key: "api.udp.startFailedGeneric", params: {} },
    "多手型运行时请指定 hand 字段": { key: "api.multiHandSync", params: {} },
    "Specify the hand field when multiple models are active": {
      key: "api.multiHandSync",
      params: {},
    },
  };
  if (exact[s]) return exact[s];

  const patterns = [
    [
      /^无法获取路由器在 (.+) 的 MAC 地址，请检查连接。$/,
      "wifi.errMacLookup",
      (m) => ({ ip: m[1] }),
    ],
    [
      /^Could not resolve router MAC for (.+)\. Check connection\.$/,
      "wifi.errMacLookup",
      (m) => ({ ip: m[1] }),
    ],
    [/^配网失败: 无法获取路由器在 (.+) 的 MAC 地址，请检查连接。$/, "wifi.errMacLookup", (m) => ({ ip: m[1] })],
    [/^Provisioning failed: Could not resolve router MAC for (.+)\.$/, "wifi.errMacLookup", (m) => ({ ip: m[1] })],
    [/^配网失败: (.+)$/, "wifi.errProvisionDetail", (m) => ({ detail: m[1] })],
    [/^Provisioning failed: (.+)$/, "wifi.errProvisionDetail", (m) => ({ detail: m[1] })],
    [/^回调 IP 格式无效: (.+)$/, "wifi.invalidReturnIpDetail", (m) => ({ ip: m[1] })],
    [/^Invalid return IP: (.+)$/, "wifi.invalidReturnIpDetail", (m) => ({ ip: m[1] })],
    [/^未知手型: (.+)$/, "api.unknownHands", (m) => ({ hands: m[1] })],
    [/^Unknown model\(s\): (.+)$/, "api.unknownHands", (m) => ({ hands: m[1] })],
    [/^手型未启用: (.+)$/, "api.handNotEnabled", (m) => ({ hand: m[1] })],
    [/^Model not enabled: (.+)$/, "api.handNotEnabled", (m) => ({ hand: m[1] })],
    [/^写入配置失败: (.+)$/, "api.writeConfigFailed", (m) => ({ detail: m[1] })],
    [/^Failed to write config: (.+)$/, "api.writeConfigFailed", (m) => ({ detail: m[1] })],
  ];

  for (const [re, key, paramsFn] of patterns) {
    const m = s.match(re);
    if (m) return { key, params: paramsFn ? paramsFn(m) : {} };
  }

  return { key: "api.rawDetail", params: { detail: s } };
}

/** 将 API/后端返回文案本地化为当前语言 */
function localizeApiDetail(text) {
  const matched = matchApiDetail(text);
  if (!matched) return "";
  return t(matched.key, matched.params);
}

function setLang(lang) {
  if (!I18N[lang] || lang === currentLang) return;
  currentLang = lang;
  localStorage.setItem(LANG_STORAGE_KEY, lang);
  applyLanguage();
  if (typeof window.onGatewayLangChange === "function") {
    window.onGatewayLangChange();
  }
}

function applyLanguage() {
  document.documentElement.lang = currentLang === "zh" ? "zh-CN" : "en";
  document.title = t("page.title");

  document.querySelectorAll("[data-i18n]").forEach((el) => {
    if (el.id === "hand-selected") return;
    el.textContent = t(el.dataset.i18n);
  });
  document.querySelectorAll("[data-i18n-html]").forEach((el) => {
    el.innerHTML = t(el.dataset.i18nHtml);
  });
  document.querySelectorAll("[data-i18n-placeholder]").forEach((el) => {
    el.placeholder = t(el.dataset.i18nPlaceholder);
  });
  document.querySelectorAll("[data-i18n-aria-label]").forEach((el) => {
    el.setAttribute("aria-label", t(el.dataset.i18nAriaLabel));
  });
  document.querySelectorAll("[data-i18n-title]").forEach((el) => {
    el.setAttribute("title", t(el.dataset.i18nTitle));
  });

  document.querySelectorAll(".lang-switch [data-lang]").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.lang === currentLang);
    btn.setAttribute("aria-pressed", btn.dataset.lang === currentLang ? "true" : "false");
  });
}

function setupLangSwitcher() {
  document.querySelectorAll(".lang-switch [data-lang]").forEach((btn) => {
    btn.addEventListener("click", () => setLang(btn.dataset.lang));
  });
}

applyLanguage();
setupLangSwitcher();
