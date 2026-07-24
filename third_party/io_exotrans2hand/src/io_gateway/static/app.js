/**
 * io_gateway 简易 Web 控制台前端逻辑
 *
 * - loadHands：GET /api/v1/hands/configs
 * - upload：POST /api/v1/hands/configs/upload
 * - apply：POST /api/v1/hands/select
 * - WebSocket /ws：订阅 io_esk.* / io_align.* / io_teleop.*
 */

const REQUIRED_YML_FILES = [
  "tf_transform_v2.yml",
  "controller_v2_3_left.yml",
  "controller_v2_3_right.yml",
];
// 与后端 hand_upload._SAFE_HAND_NAME、算法包 robot_name 一致
const HAND_NAME_RE = /^[A-Za-z0-9_]{1,128}$/;
const SKIP_TOP = new Set(["__MACOSX", ".DS_Store"]);
const ARCHIVE_EXT_RE = /\.(zip|tar|tgz|tbz2|txz)$/i;
const ARCHIVE_COMPOUND_RE = /\.tar\.(gz|bz2|xz)$/i;
const ARCHIVE_STRIP_RE = /\.(tar\.(gz|bz2|xz)|tgz|tbz2|txz|zip|tar|gz)$/i;

let existingHandDirs = [];
/** @type {{ kind: "folder", folderName: string|null, entries: { relativePath: string, file: File }[] } | { kind: "archive", folderName: string, file: File } | null} */
let pendingUploadPackage = null;
let pendingOverwrite = false;
/** @type {{ key: string, params?: Record<string, string>, isError: boolean } | { raw: string, isError: boolean } | null} */
let uploadStatusState = null;
/** @type {{ key: string, params: Record<string, string>, isError: boolean } | { raw: string, isError: boolean } | null} */
let wifiStatusState = null;
/** @type {{ key: string, params: Record<string, string>, isError: boolean } | { raw: string, isError: boolean } | null} */
let udpStatusState = null;
let lastHandLabel = null;
let lastStatusData = null;
let statusPollOffline = false;
let lastStatusPollError = "";
let lastPortSignature = null;
let portsMonitorReady = false;
let portDisconnectClearTimer = null;
const portDisconnectClearPending = new Set();

function statusExo(data) {
  return data?.exo || {};
}

function statusActiveHands(data) {
  return data?.active_hands || [];
}

function statusConfiguredHands(data) {
  return data?.configured_hands || [];
}

function statusExoBindings(data) {
  return statusExo(data).bindings || {};
}

function statusExoTopology(data) {
  return statusExo(data).topology || "none";
}

function statusWireless(data) {
  return data?.wireless || {};
}

function isUdpExoActive(data) {
  const exo = statusExo(data);
  return exo.transport === "udp" && Boolean(exo.running);
}

function emitPortDisconnect(side) {
  if (side !== "left" && side !== "right") return;
  window.dispatchEvent(
    new CustomEvent("gateway-port-disconnect", { detail: { side } })
  );
}

async function flushPendingPortDisconnectClear() {
  portDisconnectClearTimer = null;
  const pending = [...portDisconnectClearPending];
  portDisconnectClearPending.clear();
  if (!pending.length) return;
  let devices = {};
  try {
    const res = await fetch("/api/v1/status");
    devices = statusExoBindings(await res.json());
  } catch (_) {
    for (const side of pending) emitPortDisconnect(side);
    return;
  }
  for (const side of pending) {
    if (!devices[side]) emitPortDisconnect(side);
  }
}

function queuePortDisconnectClear(sides) {
  for (const s of sides) portDisconnectClearPending.add(s);
  if (portDisconnectClearTimer) clearTimeout(portDisconnectClearTimer);
  portDisconnectClearTimer = setTimeout(flushPendingPortDisconnectClear, 500);
}

function confirmOverwrite(robotName) {
  return confirm(t("confirm.overwrite", { name: robotName }));
}

function restoreButtonLabels() {
  const apply = document.getElementById("apply");
  const upload = document.getElementById("upload-hand");
  const wifiProvision = document.getElementById("wifi-provision");
  const wifiSave = document.getElementById("wifi-save-config");
  const udpToggle = document.getElementById("udp-exo-toggle");
  if (apply) {
    if (apply.dataset.busy === "1") {
      apply.textContent = t("btn.applyBusy");
    } else {
      apply.textContent = t("btn.apply");
      updateApplyButtonState();
    }
  }
  if (upload) upload.textContent = upload.disabled ? t("btn.uploadBusy") : t("btn.upload");
  if (wifiProvision) {
    wifiProvision.textContent = wifiProvision.disabled
      ? t("btn.wifiProvisionBusy")
      : t("btn.wifiProvision");
  }
  if (wifiSave) {
    wifiSave.textContent = wifiSave.disabled ? t("btn.wifiSaveBusy") : t("btn.wifiSave");
  }
  if (udpToggle && udpToggle.dataset.busy !== "1") {
    const udpActive = udpToggle.dataset.udpActive === "1";
    udpToggle.textContent = udpActive ? t("btn.udpExoStop") : t("btn.udpExoStart");
  }
}

const WIFI_PASSWORD_MIN_LEN = 8;

function getWifiFormValues() {
  return {
    ssid: document.getElementById("wifi-ssid")?.value?.trim() || "",
    password: document.getElementById("wifi-password")?.value || "",
    return_ip: document.getElementById("wifi-return-ip")?.value?.trim() || "",
  };
}

function validateWifiPassword(password) {
  if (!password) return { ok: false, key: "wifi.missingPassword" };
  if (password.length < WIFI_PASSWORD_MIN_LEN) {
    return { ok: false, key: "wifi.passwordTooShort", params: { min: WIFI_PASSWORD_MIN_LEN } };
  }
  return { ok: true };
}

function applyWifiFormValues(data) {
  const ssidEl = document.getElementById("wifi-ssid");
  const passwordEl = document.getElementById("wifi-password");
  const returnIpEl = document.getElementById("wifi-return-ip");
  if (ssidEl) ssidEl.value = data?.ssid || "";
  if (passwordEl) passwordEl.value = data?.password || "";
  if (returnIpEl) returnIpEl.value = data?.return_ip || "";
}

async function loadWifiConfig() {
  try {
    const res = await fetch("/api/v1/wifi/config");
    if (!res.ok) return;
    applyWifiFormValues(await res.json());
  } catch (_) {
    /* 忽略 */
  }
}

function setUploadStatusKey(key, params, isError) {
  uploadStatusState = { key, params: params || {}, isError };
  setUploadStatus(t(key, params), isError);
}

function setUploadStatusRaw(text, isError) {
  uploadStatusState = text ? { raw: text, isError } : null;
  setUploadStatus(text, isError);
}

function refreshUploadStatusDisplay() {
  if (!uploadStatusState) {
    setUploadStatus("", false);
    return;
  }
  if ("raw" in uploadStatusState) {
    setUploadStatus(uploadStatusState.raw, uploadStatusState.isError);
  } else {
    setUploadStatus(
      t(uploadStatusState.key, i18nParams(uploadStatusState.params)),
      uploadStatusState.isError
    );
  }
}

function setWifiStatusKey(key, params, isError) {
  wifiStatusState = key ? { key, params: params || {}, isError } : null;
  setWifiStatus(key ? t(key, params) : "", isError);
}

function setWifiStatusFromApi(text, isError, { fallbackKey = "wifi.provisionFailed" } = {}) {
  const normalized = normalizeDetailText(text);
  const matched = matchApiDetail(normalized);
  if (!text) {
    wifiStatusState = null;
    setWifiStatus("", false);
    return;
  }
  if (!normalized) {
    wifiStatusState = null;
    setWifiStatus("", false);
    return;
  }
  if (matched) {
    if (isError && matched.key === "api.rawDetail" && fallbackKey) {
      wifiStatusState = { key: fallbackKey, params: { detail: matched.params.detail }, isError };
    } else {
      wifiStatusState = { key: matched.key, params: matched.params || {}, isError };
    }
    setWifiStatus(t(wifiStatusState.key, wifiStatusState.params), isError);
    return;
  }
  wifiStatusState = { raw: text, isError };
  setWifiStatus(text, isError);
}

function refreshWifiStatusDisplay() {
  if (!wifiStatusState) {
    setWifiStatus("", false);
    return;
  }
  if ("raw" in wifiStatusState) {
    setWifiStatus(wifiStatusState.raw, wifiStatusState.isError);
  } else {
    setWifiStatus(
      t(wifiStatusState.key, i18nParams(wifiStatusState.params)),
      wifiStatusState.isError
    );
  }
}

function setUdpExoStatusFromResponse(data, isError, { fallbackKey = null } = {}) {
  const code = extractApiCode(data);
  const i18nKey = code ? udpApiCodeKey(code) : null;
  if (i18nKey) {
    setUdpExoStatusKey(i18nKey, {}, isError);
    return;
  }
  const text =
    typeof data === "string"
      ? data
      : apiErrorRaw(data, "") || data?.message || "";
  if (!text && fallbackKey) {
    setUdpExoStatusKey(fallbackKey, {}, isError);
    return;
  }
  setUdpExoStatusFromApi(text, isError, { fallbackKey });
}

function setUdpExoStatusFromApi(text, isError, { fallbackKey = null } = {}) {
  const matched = matchApiDetail(text);
  if (!text) {
    udpStatusState = null;
    renderUdpModule(lastStatusData || {});
    return;
  }
  if (matched) {
    if (isError && matched.key === "api.rawDetail" && fallbackKey) {
      udpStatusState = { key: fallbackKey, params: { detail: matched.params.detail }, isError: true };
    } else {
      udpStatusState = { key: matched.key, params: matched.params || {}, isError };
    }
  } else {
    udpStatusState = { raw: text, isError };
  }
  renderUdpModule(lastStatusData || {});
}

function setUdpExoStatusKey(key, params, isError) {
  udpStatusState = { key, params: params || {}, isError };
  renderUdpModule(lastStatusData || {});
}

function refreshUdpStatusDisplay() {
  renderUdpModule(lastStatusData || {});
}

function udpStatusMessage() {
  if (!udpStatusState) return "";
  if ("raw" in udpStatusState) return udpStatusState.raw;
  return t(udpStatusState.key, i18nParams(udpStatusState.params));
}

function apiErrorRaw(payload, fallback) {
  if (payload == null) return fallback;
  if (typeof payload === "string") return payload;
  const d = payload.detail !== undefined ? payload.detail : payload;
  return normalizeDetailText(d) || fallback;
}

function apiErrorDetail(payload, fallback) {
  return localizeApiDetail(apiErrorRaw(payload, fallback));
}

function i18nParams(params) {
  if (!params) return {};
  if (!params.detail) return { ...params };
  return { ...params, detail: localizeApiDetail(params.detail) };
}

function apiCollisionHand(payload) {
  const d = payload?.detail;
  if (d && typeof d === "object" && d.hand) return d.hand;
  return document.getElementById("hand-name")?.value || null;
}

function isArchiveFilename(name) {
  const base = ((name || "").replace(/\\/g, "/").split("/").pop() || "").trim();
  const lower = base.toLowerCase();
  return ARCHIVE_EXT_RE.test(lower) || ARCHIVE_COMPOUND_RE.test(lower);
}

/** 与改文件夹前一致：直接匹配 file.name 扩展名 */
function isArchiveFile(file) {
  const name = (file?.name || "").toLowerCase();
  return ARCHIVE_EXT_RE.test(name) || ARCHIVE_COMPOUND_RE.test(name);
}

function archiveBasename(file) {
  const name = file?.name || file?.webkitRelativePath || "";
  return ((name || "").replace(/\\/g, "/").split("/").pop() || "").trim();
}

function inferArchiveDisplayName(filename) {
  const base = (filename || "").replace(/\\/g, "/").split("/").pop() || "";
  return base.replace(ARCHIVE_STRIP_RE, "");
}

function toFileArray(files) {
  if (!files) return [];
  return Array.from(files);
}

function cleanUploadPath(raw) {
  return (raw || "").replace(/\\/g, "/").replace(/^\/+/, "");
}

/** 从 File 取上传相对路径（webkitRelativePath 或 name，不做规约） */
function fileUploadPath(file) {
  const rel = cleanUploadPath(file.webkitRelativePath);
  const name = cleanUploadPath(file.name);
  if (rel && rel.includes("/")) return rel;
  if (name && name.includes("/")) return name;
  return rel || name;
}

function makeStagedFile(file, relativePath) {
  const rel = cleanUploadPath(relativePath);
  if (!rel || rel === file.name) return file;
  return new File([file], rel, { type: file.type, lastModified: file.lastModified });
}

/** 按浏览器原始相对路径组包（与压缩包解压路径规则一致，不重排目录） */
function buildFolderPackage(rawFiles) {
  const files = toFileArray(rawFiles);
  if (!files.length) return null;

  const entries = [];
  for (const file of files) {
    const relativePath = cleanUploadPath(fileUploadPath(file));
    if (!relativePath || relativePath.endsWith("/")) continue;
    const top = relativePath.split("/")[0];
    if (SKIP_TOP.has(top)) continue;
    entries.push({ relativePath, file });
  }
  if (!entries.length) return null;

  const tops = new Set(entries.map((e) => e.relativePath.split("/")[0]));
  const folderName = tops.size === 1 ? [...tops][0] : null;

  return { kind: "folder", folderName, entries };
}

function stagedFolderPaths(pkg) {
  return pkg.entries.map((e) => e.relativePath);
}

/** 与后端 hand_upload.validate_hand_package 对齐（硬性标准目录树） */
function validateHandPackage(paths) {
  const norm = [];
  for (const raw of paths) {
    if (!raw || raw.endsWith("/")) continue;
    const rel = raw.replace(/\\/g, "/").replace(/^\/+/, "");
    const top = rel.split("/")[0];
    if (SKIP_TOP.has(top)) continue;
    norm.push(rel);
  }
  if (norm.length === 0) {
    return { robotName: null, errors: [t("err.noFiles")] };
  }

  const tops = new Set(norm.map((p) => p.split("/")[0]));
  if (tops.size !== 1) {
    return { robotName: null, errors: [t("err.badFolder")] };
  }

  const root = [...tops][0];
  const prefix = `${root}/`;
  const rootLower = root.toLowerCase();
  const hasUrdf = norm.some((p) => {
    const parts = p.split("/");
    return (
      parts.length >= 3 &&
      parts[0].toLowerCase() === rootLower &&
      parts[1].toLowerCase() === "urdf"
    );
  });
  const hasMeshes = norm.some((p) => {
    const parts = p.split("/");
    return (
      parts.length >= 3 &&
      parts[0].toLowerCase() === rootLower &&
      parts[1].toLowerCase() === "meshes"
    );
  });
  const ymlAtRoot = new Set(
    norm
      .filter((p) => p.startsWith(prefix) && p.split("/").length === 2)
      .map((p) => p.split("/").pop())
  );

  const errors = [];
  if (!hasUrdf) errors.push(t("err.missingUrdf", { root }));
  if (!hasMeshes) errors.push(t("err.missingMeshes", { root }));
  for (const yml of REQUIRED_YML_FILES) {
    if (!ymlAtRoot.has(yml)) errors.push(t("err.missingYml", { root, yml }));
  }
  if (errors.length === 0 && !HAND_NAME_RE.test(root)) {
    errors.push(t("err.invalidName", { name: root }));
  }

  return { robotName: root, errors };
}

/** 识别暂存文件夹包并处理撞名 */
function recognizeFolderPackage(pkg, { alertOnFail = true } = {}) {
  const paths = stagedFolderPaths(pkg);
  const check = validateHandPackage(paths);
  if (check.errors.length > 0) {
    console.warn("[upload] 文件夹校验失败", {
      tops: [...new Set(paths.map((p) => p.split("/")[0]))],
      sample: paths.slice(0, 6),
      errors: check.errors,
    });
    if (alertOnFail) {
      alert(t("alert.folderInvalid", { errors: check.errors.join("\n") }));
    }
    return { ok: false, ...check };
  }
  if (existingHandDirs.includes(check.robotName)) {
    if (alertOnFail) {
      if (confirmOverwrite(check.robotName)) {
        return { ok: true, ...check, overwrite: true };
      }
      return { ok: false, ...check, collision: true, cancelled: true };
    }
    return { ok: false, ...check, collision: true };
  }
  return { ok: true, ...check };
}

function looksLikeFolderUpload(files) {
  if (!files || files.length === 0) return false;
  if (files.length > 1) return true;
  const f = files[0];
  if (isArchiveFile(f)) return false;
  const path = (f.webkitRelativePath || f.name || "").replace(/\\/g, "/");
  return path.includes("/");
}

function clearPendingUpload() {
  pendingUploadPackage = null;
  pendingOverwrite = false;
  document.getElementById("hand-name").value = "";
  const archiveInput = document.getElementById("hand-upload-input");
  const folderInput = document.getElementById("hand-folder-input");
  if (archiveInput) archiveInput.value = "";
  if (folderInput) folderInput.value = "";
  uploadStatusState = null;
  setUploadStatus("", false);
  refreshUploadDropDisplay();
}

function refreshUploadDropDisplay() {
  const zone = document.getElementById("upload-drop");
  const idle = document.getElementById("upload-drop-idle");
  const selected = document.getElementById("upload-drop-selected");
  const selectedMain = document.getElementById("upload-drop-selected-main");
  const selectedHint = document.getElementById("upload-drop-selected-hint");
  const clearBtn = document.getElementById("upload-clear");
  const pkg = pendingUploadPackage;

  if (!zone || !idle || !selected || !selectedMain || !selectedHint) return;

  if (!pkg) {
    zone.classList.remove("has-selection");
    idle.hidden = false;
    selected.hidden = true;
    if (clearBtn) clearBtn.hidden = true;
    return;
  }

  zone.classList.add("has-selection");
  idle.hidden = true;
  selected.hidden = false;
  if (clearBtn) clearBtn.hidden = false;

  if (pkg.kind === "archive") {
    const name = pkg.uploadName || archiveBasename(pkg.file);
    selectedMain.textContent = t("upload.selectedArchive", { name });
    selectedHint.textContent = "";
    return;
  }

  const paths = stagedFolderPaths(pkg);
  const check = validateHandPackage(paths);
  const root = check.robotName || pkg.folderName || paths[0]?.split("/")[0] || "—";
  selectedMain.textContent = t("upload.selectedFolder", { name: root });
  selectedHint.textContent = t("upload.selectedMeta", { count: String(pkg.entries.length) });
}

function applyRecognizedPackage(pkg, check, { archiveName = null } = {}) {
  pendingUploadPackage = pkg;
  pendingOverwrite = !!check?.overwrite;
  const nameInput = document.getElementById("hand-name");

  if (pkg.kind === "archive") {
    nameInput.value = pkg.folderName || "";
    let hint = "";
    if (pkg.folderName && existingHandDirs.includes(pkg.folderName)) {
      hint = t("upload.overwriteHint");
    }
    setUploadStatusKey("upload.recognizedArchive", { name: archiveName || pkg.uploadName || archiveBasename(pkg.file), hint }, false);
  } else {
    nameInput.value = check.robotName || pkg.folderName || "";
    const overwriteHint = pendingOverwrite ? t("upload.overwriteHint") : "";
    setUploadStatusKey(
      "upload.recognizedFolder",
      { name: check.robotName, hint: overwriteHint },
      false
    );
  }
  refreshUploadDropDisplay();
}

async function loadExistingHandDirs() {
  try {
    const res = await fetch("/api/v1/hands/dirs");
    existingHandDirs = (await res.json()).map((d) => d.id);
  } catch (_) {
    existingHandDirs = [];
  }
}

function handChainReady(hand, processes) {
  if (!processes || !hand) return false;
  const tkey = `transform@${hand}`;
  const info = processes[tkey];
  return Boolean(info && info.running);
}

/** 仅更新「当前手型」展示，不改动下拉框（避免轮询覆盖用户正在选择的项） */
function syncHandLabel(hands, processes, savedHands) {
  const el = document.getElementById("hand-selected");
  const applied = Array.isArray(hands) ? hands.filter(Boolean) : hands ? [hands] : [];
  const saved = Array.isArray(savedHands) ? savedHands.filter(Boolean) : [];
  const display = applied.length ? applied : saved;
  if (!display.length) {
    lastHandLabel = null;
    el.textContent = t("hand.none");
    return;
  }
  const list = display;
  lastHandLabel = list.join(", ");
  if (!applied.length && saved.length) {
    el.textContent = `${lastHandLabel} ${t("hand.savedPending")}`;
    return;
  }
  const allReady = list.every((h) => handChainReady(h, processes));
  el.textContent = allReady ? lastHandLabel : `${lastHandLabel} ${t("hand.notReady")}`;
}

function portSignature(data) {
  const d = statusExoBindings(data);
  return JSON.stringify({
    topology: statusExoTopology(data),
    left: d.left || "",
    right: d.right || "",
  });
}

function topologyLabel(topology) {
  const key = `ports.topo.${topology || "none"}`;
  const label = t(key);
  return label === key ? topology || "none" : label;
}

function showPortToast(message, kind = "info") {
  const container = document.getElementById("port-toast-container");
  if (!container || !message) return;
  const el = document.createElement("div");
  el.className = `port-toast ${kind}`;
  el.textContent = message;
  container.appendChild(el);
  requestAnimationFrame(() => el.classList.add("show"));
  setTimeout(() => {
    el.classList.remove("show");
    setTimeout(() => el.remove(), 280);
  }, 4200);
}

function appendWirelessIpRow(box, sideLabel, { ip, status, statusTone = "", pathEmpty = false, rowClass = "", statusHidden = false } = {}) {
  const row = document.createElement("div");
  row.className = "port-row" + (rowClass ? ` ${rowClass}` : "");

  const sideEl = document.createElement("span");
  sideEl.className = "port-side";
  sideEl.textContent = sideLabel;

  const pathEl = document.createElement("code");
  pathEl.className = "port-path" + (pathEmpty ? " empty" : "");

  const ipEl = document.createElement("span");
  ipEl.className = "port-path-ip";
  ipEl.textContent = ip;

  pathEl.appendChild(ipEl);
  if (!statusHidden && status) {
    const statusEl = document.createElement("span");
    statusEl.className = "port-path-status" + (statusTone ? ` ${statusTone}` : "");
    statusEl.textContent = ` (${status})`;
    pathEl.appendChild(statusEl);
  }

  row.append(sideEl, pathEl);
  box.appendChild(row);
}

function endpointIp(endpoint) {
  if (!endpoint) return "";
  return endpoint.includes(":") ? endpoint.split(":")[0] : endpoint;
}

function topologyBoundIps(data) {
  const devices = statusExoBindings(data);
  return new Set(Object.values(devices).map(endpointIp).filter(Boolean));
}

/** 在线 IP 超过 2 个时，优先展示已绑定外骨骼的 IP（与探测侧 bind_lock 一致） */
function pickWirelessDisplayIps(onlineIps, boundIps, maxSlots = 2) {
  const online = Array.isArray(onlineIps) ? onlineIps : [];
  if (online.length <= maxSlots) return [...online];
  const bound = online.filter((ip) => boundIps.has(ip));
  const others = online.filter((ip) => !boundIps.has(ip));
  return [...bound, ...others].slice(0, maxSlots);
}

/** 绑定 IP 且 exo 进程在跑，与设备连接「已连接」一致 */
function isWirelessIpConnected(ip, data, boundIps) {
  if (!boundIps.has(ip)) return false;
  return Boolean(statusExo(data).running);
}

function formatWirelessIpSlotDisplay(ip, data, boundIps) {
  if (isWirelessIpConnected(ip, data, boundIps)) {
    return {
      ip,
      status: t("wifi.moduleStatusReceiving"),
      statusTone: "ok",
      pathEmpty: false,
    };
  }
  return {
    ip,
    status: t("wifi.moduleStatusWaiting"),
    statusTone: "",
    pathEmpty: false,
  };
}

function formatWirelessIpSlotEmpty() {
  return {
    ip: t("wifi.noOnlineDevice"),
    status: "",
    statusTone: "",
    pathEmpty: true,
    statusHidden: true,
  };
}

function renderUdpModule(data) {
  const box = document.getElementById("udp-module-display");
  if (!box) return;

  const wireless = statusWireless(data);
  const onlineIps = Array.isArray(wireless.online_ips) ? wireless.online_ips : [];
  const boundIps = topologyBoundIps(data);
  const displayIps = pickWirelessDisplayIps(onlineIps, boundIps, 2);

  box.innerHTML = "";

  const slots = [
    { label: t("field.wifiIp1"), ip: displayIps[0] || null },
    { label: t("field.wifiIp2"), ip: displayIps[1] || null },
  ];

  for (let i = 0; i < slots.length; i++) {
    const { label, ip } = slots[i];
    let row;
    if (ip) {
      row = formatWirelessIpSlotDisplay(ip, data, boundIps);
    } else {
      row = formatWirelessIpSlotEmpty();
    }

    appendWirelessIpRow(box, label, {
      ip: row.ip,
      status: row.status,
      statusTone: row.statusTone,
      pathEmpty: row.pathEmpty,
      statusHidden: row.statusHidden,
      rowClass: row.statusTone ? "module-status-row" : "",
    });
  }
}

function syncUdpExoButtons(data) {
  renderUdpModule(data);
}

/** 外骨骼有线/无线状态区：首屏占位与轮询刷新共用 */
function renderConnectStatusPanels(data) {
  renderExoPorts(data);
  renderUdpModule(data);
}

function renderExoPorts(data) {
  const box = document.getElementById("exo-ports-display");
  if (!box) return;

  const devices = statusExoBindings(data);
  const exoRunning = Boolean(statusExo(data).running);

  const sides = [
    { key: "left", label: t("ports.sideLeft") },
    { key: "right", label: t("ports.sideRight") },
  ];
  box.innerHTML = "";
  for (const { key, label } of sides) {
    const port = devices[key];
    const row = document.createElement("div");
    row.className = "port-row";

    const sideEl = document.createElement("span");
    sideEl.className = "port-side";
    sideEl.textContent = label;

    const pathEl = document.createElement("code");
    pathEl.className = "port-path" + (port ? "" : " empty");
    pathEl.textContent = port || t("ports.na");

    const connected = Boolean(port) && exoRunning;
    const badge = document.createElement("span");
    badge.className = "port-badge" + (connected ? " on" : "");
    badge.textContent = connected ? t("ports.connected") : t("ports.na");

    row.append(sideEl, pathEl, badge);
    box.appendChild(row);
  }
}

function checkPortChange(data) {
  const sig = portSignature(data);
  if (!portsMonitorReady) {
    lastPortSignature = sig;
    portsMonitorReady = true;
    return;
  }
  if (sig === lastPortSignature) return;

  const before = lastPortSignature ? JSON.parse(lastPortSignature) : { left: "", right: "" };
  const after = JSON.parse(sig);
  const sideMeta = [
    { key: "left", label: t("ports.sideLeft") },
    { key: "right", label: t("ports.sideRight") },
  ];

  const disconnected = [];
  for (const { key, label } of sideMeta) {
    const prev = before[key] || "";
    const next = after[key] || "";
    if (prev === next) continue;

    if (prev && !next) {
      disconnected.push(key);
      showPortToast(t("toast.portDisconnected", { side: label, port: prev }), "warn");
    } else if (!prev && next) {
      showPortToast(t("toast.portConnected", { side: label, port: next }), "ok");
      void scheduleHandsSyncAfterExo();
    } else if (prev && next) {
      showPortToast(
        t("toast.portSwitched", { side: label, from: prev, to: next }),
        "info"
      );
    }
  }

  if (disconnected.length === 1) {
    emitPortDisconnect(disconnected[0]);
  } else if (disconnected.length >= 2) {
    // 全量重扫时 status 可能瞬态两侧都为空，延迟后再确认仍断开的侧
    queuePortDisconnectClear(disconnected);
  }

  lastPortSignature = sig;
}

const STATUS_API_PATH = "/api/v1/status";
/** 与 gateway.yaml probe_interval_sec 对齐，便于与外骨骼编排同步 */
const STATUS_POLL_MS = 1000;
/** 外骨骼接入后快速轮询直到 hand_choose 应用（覆盖 startup_delay_sec ~2s） */
const HANDS_SYNC_BURST_MS = [0, 300, 600, 1000, 1500, 2200, 3000];

let handsSyncBurstToken = 0;

function isStatusPayloadValid(data) {
  const exo = statusExo(data);
  return (
    data &&
    typeof data === "object" &&
    typeof exo.topology === "string" &&
    data.processes &&
    typeof data.processes === "object"
  );
}

function isFetchNetworkError(err) {
  return (
    err instanceof TypeError &&
    /fetch|network|failed|load/i.test(String(err.message || err))
  );
}

function formatStatusPollError(err, res, data) {
  if (res && !res.ok) {
    let detail = res.statusText || "";
    if (data && typeof data.detail === "string") detail = data.detail;
    else if (data && data.detail != null) {
      try {
        detail = JSON.stringify(data.detail);
      } catch (_) {
        /* ignore */
      }
    }
    return t("status.httpError", { code: res.status, detail: detail || "—" });
  }
  if (isFetchNetworkError(err)) return t("status.offline");
  return String(err?.message || err || t("status.offline"));
}

function setGatewayStatusBanner(offline, message) {
  const el = document.getElementById("gateway-status-banner");
  if (!el) return;
  if (offline) {
    el.textContent = message || t("status.offline");
    el.hidden = false;
    el.classList.add("show");
  } else {
    el.textContent = "";
    el.hidden = true;
    el.classList.remove("show");
  }
}

function renderStatusPre(data, { error = "" } = {}) {
  const statusEl = document.getElementById("status");
  if (!statusEl) return;
  if (error) {
    statusEl.textContent = data
      ? `${t("status.staleHint", { error })}\n\n${JSON.stringify(data, null, 2)}`
      : error;
    return;
  }
  statusEl.textContent = JSON.stringify(data, null, 2);
}

function sameHandList(a, b) {
  const left = [...(a || [])].sort().join("\0");
  const right = [...(b || [])].sort().join("\0");
  return left === right;
}

/** 后台已应用手型变化时：同步勾选、重订 WS、刷新 URDF/图表 */
async function onRuntimeHandsChanged(data) {
  const hands = statusActiveHands(data);
  const configured = statusConfiguredHands(data);
  const checkboxHands = hands.length ? hands : (configured.length ? configured : []);
  setSelectedHands(checkboxHands);
  syncHandsApplyBaseline(hands);
  await subscribeWsFromServer();
  if (typeof window.refreshVizPage === "function") {
    await window.refreshVizPage();
  }
}

function exoIsAvailable(data) {
  if (!data) return false;
  if (isUdpExoActive(data)) return true;
  return statusExoTopology(data) !== "none";
}

/** 外骨骼可用后短周期拉 status，尽快完成手型 WS 订阅与 URDF 刷新 */
async function scheduleHandsSyncAfterExo() {
  const token = ++handsSyncBurstToken;
  let elapsed = 0;
  for (const targetMs of HANDS_SYNC_BURST_MS) {
    if (token !== handsSyncBurstToken) return;
    const wait = targetMs - elapsed;
    elapsed = targetMs;
    if (wait > 0) {
      await new Promise((resolve) => setTimeout(resolve, wait));
    }
    if (token !== handsSyncBurstToken) return;

    const prevHands = statusActiveHands(lastStatusData);
    const prevTopology = statusExoTopology(lastStatusData);
    const prevExoRunning = Boolean(statusExo(lastStatusData).running);
    try {
      const res = await fetch(STATUS_API_PATH);
      const data = await res.json();
      if (!res.ok || !isStatusPayloadValid(data)) continue;
      applyStatusData(data, { prevHands, prevTopology, prevExoRunning });
      const hands = statusActiveHands(data);
      const saved = statusConfiguredHands(data);
      if (hands.length && (!saved.length || sameHandList(hands, saved))) return;
    } catch (_) {
      /* 下一轮重试 */
    }
  }
}

function applyStatusData(data, { prevHands = [], prevTopology = "none", prevExoRunning = false } = {}) {
  const hands = statusActiveHands(data);
  const topology = statusExoTopology(data);
  const handsChanged = !sameHandList(hands, prevHands);
  const exoNow = exoIsAvailable(data);
  const exoBefore = prevTopology !== "none" || prevExoRunning;
  const exoBecameAvailable = exoNow && !exoBefore;
  const savedHands = statusConfiguredHands(data);
  lastStatusData = data;
  syncHandLabel(hands, data.processes, savedHands);
  renderConnectStatusPanels(data);
  checkPortChange(data);
  if (handsChanged) {
    void onRuntimeHandsChanged(data);
  } else if (exoBecameAvailable) {
    void subscribeWsFromServer({ force: true });
    if (savedHands.length && !hands.length) {
      void scheduleHandsSyncAfterExo();
    }
  }
  renderStatusPre(data);
  window.dispatchEvent(new CustomEvent("gateway-status", { detail: data }));
}

/** 轮询 GET /api/v1/status 并显示 JSON；失败时保留上次数据并提示 */
async function refreshStatus() {
  const prevHands = statusActiveHands(lastStatusData);
  const prevTopology = statusExoTopology(lastStatusData);
  const prevExoRunning = Boolean(statusExo(lastStatusData).running);
  const wasOffline = statusPollOffline;
  let res = null;
  let data = null;
  try {
    res = await fetch(STATUS_API_PATH);
    try {
      data = await res.json();
    } catch (_) {
      throw new Error(t("status.badResponse"));
    }
    if (!res.ok) {
      throw new Error(formatStatusPollError(null, res, data));
    }
    if (!isStatusPayloadValid(data)) {
      throw new Error(t("status.badResponse"));
    }
  } catch (err) {
    const message =
      res && !res.ok
        ? formatStatusPollError(null, res, data)
        : formatStatusPollError(err);
    statusPollOffline = true;
    lastStatusPollError = message;
    setGatewayStatusBanner(true, message);
    renderStatusPre(lastStatusData, { error: message });
    if (!wasOffline) showPortToast(message, "warn");
    return false;
  }

  statusPollOffline = false;
  lastStatusPollError = "";
  setGatewayStatusBanner(false);
  if (wasOffline) showPortToast(t("status.recovered"), "ok");
  applyStatusData(data, { prevHands, prevTopology, prevExoRunning });
  return true;
}

function normalizePreferredHands(preferred) {
  if (!preferred) return null;
  if (Array.isArray(preferred)) return preferred.filter(Boolean);
  return [preferred];
}

let handsApplyBaselineKey = "";

function handsSelectionKey(hands) {
  return [...(hands || [])].sort().join("\0");
}

function syncHandsApplyBaseline(hands) {
  handsApplyBaselineKey = handsSelectionKey(hands);
  updateApplyButtonState();
}

function isHandsSelectionDirty() {
  return handsSelectionKey(getSelectedHands()) !== handsApplyBaselineKey;
}

function updateApplyButtonState() {
  const btn = document.getElementById("apply");
  if (!btn || btn.dataset.busy === "1") return;
  btn.disabled = !isHandsSelectionDirty();
}

/** 从复选框列表读取已选手型 */
function getSelectedHands() {
  const box = document.getElementById("hand-checkboxes");
  if (!box) return [];
  return [...box.querySelectorAll('input[type="checkbox"][data-hand]:checked')]
    .map((el) => el.dataset.hand)
    .filter(Boolean);
}

function setSelectedHands(handIds) {
  const active = new Set(handIds || []);
  const box = document.getElementById("hand-checkboxes");
  if (!box) return;
  for (const el of box.querySelectorAll('input[type="checkbox"][data-hand]')) {
    el.checked = active.has(el.dataset.hand);
  }
}

/** 填充手型复选框列表（保留当前勾选） */
function renderHandOptions(hands, preferred) {
  const box = document.getElementById("hand-checkboxes");
  if (!box) return;
  const preferredList = normalizePreferredHands(preferred);
  const prevSet = new Set(preferredList ?? getSelectedHands());
  box.innerHTML = "";
  if (hands.length === 0) {
    box.textContent = t("hand.noHands");
    return;
  }
  for (const h of hands) {
    const id = h.id;
    const label = document.createElement("label");
    const cb = document.createElement("input");
    cb.type = "checkbox";
    cb.dataset.hand = id;
    cb.checked = prevSet.has(id);
    label.appendChild(cb);
    label.appendChild(document.createTextNode(id));
    box.appendChild(label);
  }
  bindHandCheckboxStreamRefresh();
}

/** 加载手型下拉列表 */
async function loadHands(preferred) {
  const res = await fetch("/api/v1/hands/configs");
  const hands = await res.json();
  renderHandOptions(hands, preferred);
  if (hands.length > 0) {
    // 初次加载：下拉框与服务器当前手型对齐（此后不再被轮询改写）
    try {
      const st = await fetch("/api/v1/status");
      const data = await st.json();
      syncHandLabel(statusActiveHands(data), data.processes, statusConfiguredHands(data));
      if (!preferred) {
        const activeHands = statusActiveHands(data);
        const configured = statusConfiguredHands(data);
        const active = activeHands.length
          ? activeHands
          : (configured.length ? configured : []);
        setSelectedHands(active);
        syncHandsApplyBaseline(active);
      }
    } catch (_) {
      /* 忽略 */
    }
  }
  await refreshStatus();
  return hands;
}

/** 首屏并行加载，避免重复请求 /status */
async function initPage() {
  document.getElementById("status").textContent = t("status.loading");
  renderWsPanel();
  const wsPick = document.getElementById("ws-stream-pick");
  if (wsPick) wsPick.addEventListener("change", renderWsPanel);
  await loadStreamCatalogTemplate();
  await Promise.all([loadExistingHandDirs(), loadHands(), loadWifiConfig()]);
  setupWifiPasswordToggle();
}

function refreshWifiPasswordToggleLabels() {
  const input = document.getElementById("wifi-password");
  const btn = document.getElementById("wifi-password-toggle");
  if (!input || !btn) return;
  const shown = input.type === "text";
  btn.setAttribute("aria-pressed", shown ? "true" : "false");
  btn.setAttribute("aria-label", t(shown ? "wifiPassword.hide" : "wifiPassword.show"));
  btn.title = t(shown ? "wifiPassword.hide" : "wifiPassword.show");
}

function setupWifiPasswordToggle() {
  const input = document.getElementById("wifi-password");
  const btn = document.getElementById("wifi-password-toggle");
  if (!input || !btn) return;
  btn.addEventListener("click", () => {
    input.type = input.type === "password" ? "text" : "password";
    refreshWifiPasswordToggleLabels();
  });
  refreshWifiPasswordToggleLabels();
}

function setUploadStatus(text, isError) {
  const el = document.getElementById("upload-status");
  el.textContent = text;
  el.classList.remove("ok", "err");
  if (text) el.classList.add(isError ? "err" : "ok");
}

function setWifiStatus(text, isError) {
  const el = document.getElementById("wifi-status");
  if (!el) return;
  el.textContent = text;
  el.classList.remove("ok", "err");
  if (text) el.classList.add(isError ? "err" : "ok");
}

function setUdpExoStatus(text, isError) {
  setUdpExoStatusFromApi(text, isError);
}

/** ESP-Touch 无线配网 */
document.getElementById("wifi-provision").onclick = async () => {
  const btn = document.getElementById("wifi-provision");
  const { ssid, password, return_ip: returnIp } = getWifiFormValues();
  if (!ssid) {
    setWifiStatus(t("wifi.missingSsid"), true);
    return;
  }
  if (!returnIp) {
    setWifiStatus(t("wifi.missingReturnIp"), true);
    return;
  }
  const pwdCheck = validateWifiPassword(password);
  if (!pwdCheck.ok) {
    setWifiStatusKey(pwdCheck.key, pwdCheck.params || {}, true);
    return;
  }
  btn.disabled = true;
  btn.textContent = t("btn.wifiProvisionBusy");
  setWifiStatus("", false);
  wifiStatusState = null;
  try {
    const res = await fetch("/api/v1/wifi/provision", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ ssid, password, return_ip: returnIp }),
    });
    if (!res.ok) {
      let errBody = {};
      try {
        errBody = await res.json();
      } catch (_) {
        /* 忽略 */
      }
      setWifiStatusFromApi(apiErrorRaw(errBody, res.statusText), true);
      return;
    }
    const data = await res.json();
    setWifiStatusFromApi(data.message || t("wifi.provisionSuccess"), false);
  } catch (e) {
    setWifiStatusFromApi(String(e), true);
  } finally {
    btn.disabled = false;
    btn.textContent = t("btn.wifiProvision");
  }
};

document.getElementById("wifi-save-config").onclick = async () => {
  const btn = document.getElementById("wifi-save-config");
  const payload = getWifiFormValues();
  if (payload.return_ip && !/^(\d{1,3}\.){3}\d{1,3}$/.test(payload.return_ip)) {
    setWifiStatusKey("wifi.configSaveFailed", { detail: t("wifi.invalidReturnIp") }, true);
    return;
  }
  const pwdCheck = validateWifiPassword(payload.password);
  if (!pwdCheck.ok) {
    setWifiStatusKey(pwdCheck.key, pwdCheck.params || {}, true);
    return;
  }
  btn.disabled = true;
  btn.textContent = t("btn.wifiSaveBusy");
  try {
    const res = await fetch("/api/v1/wifi/config", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    if (!res.ok) {
      let detail = res.statusText;
      try {
        const err = await res.json();
        detail = err.detail || JSON.stringify(err);
      } catch (_) {
        /* 忽略 */
      }
      setWifiStatusFromApi(detail, true, { fallbackKey: "wifi.configSaveFailed" });
      return;
    }
    setWifiStatusKey("wifi.configSaved", {}, false);
  } catch (e) {
    setWifiStatusKey("wifi.configSaveFailed", { detail: String(e) }, true);
  } finally {
    btn.disabled = false;
    btn.textContent = t("btn.wifiSave");
  }
};

function onFilesReady(files) {
  const list = toFileArray(files);
  if (!list.length) {
    clearPendingUpload();
    return;
  }

  pendingOverwrite = false;

  // 压缩包：单文件 + isArchiveFile(file.name)
  if (list.length === 1 && isArchiveFile(list[0])) {
    const f = list[0];
    const pkg = {
      kind: "archive",
      folderName: inferArchiveDisplayName(f.name),
      file: f,
      uploadName: f.name,
    };
    applyRecognizedPackage(pkg, { ok: true }, { archiveName: f.name });
    return;
  }

  // 文件夹：原始相对路径，硬性标准目录树（与压缩包一致）
  if (!looksLikeFolderUpload(list)) {
    alert(t("alert.unrecognized"));
    clearPendingUpload();
    return;
  }

  const pkg = buildFolderPackage(list);
  if (!pkg) {
    alert(t("alert.emptyFolder"));
    clearPendingUpload();
    return;
  }
  const check = recognizeFolderPackage(pkg);
  if (!check.ok) {
    clearPendingUpload();
    return;
  }
  pkg.folderName = check.robotName || pkg.folderName;
  applyRecognizedPackage(pkg, check);
}

async function walkDirHandle(dirHandle, prefix) {
  const out = [];
  for await (const [name, handle] of dirHandle.entries()) {
    const rel = prefix ? `${prefix}/${name}` : name;
    if (handle.kind === "file") {
      const file = await handle.getFile();
      out.push(makeStagedFile(file, rel));
    } else if (handle.kind === "directory") {
      out.push(...(await walkDirHandle(handle, rel)));
    }
  }
  return out;
}

function openArchivePicker() {
  const input = document.getElementById("hand-upload-input");
  input.value = "";
  input.click();
}

function openFolderPicker() {
  const input = document.getElementById("hand-folder-input");
  input.value = "";
  input.click();
}

async function pickFolderFlow() {
  if (window.isSecureContext && typeof window.showDirectoryPicker === "function") {
    try {
      const dir = await window.showDirectoryPicker();
      const files = await walkDirHandle(dir, dir.name);
      if (files.length === 0) {
        alert(t("alert.emptyFolder"));
        return;
      }
      onFilesReady(files, dir.name);
      return;
    } catch (e) {
      if (e.name === "AbortError") return;
      console.warn("showDirectoryPicker 不可用，改用文件夹选择器", e);
    }
  }
  openFolderPicker();
}

function openUploadPicker(folderMode = false) {
  if (folderMode) {
    pickFolderFlow();
  } else {
    openArchivePicker();
  }
}

async function walkFileEntry(entry, prefix) {
  const files = [];
  if (entry.isFile) {
    const file = await new Promise((resolve, reject) => {
      entry.file(resolve, reject);
    });
    const rel = prefix ? `${prefix}/${file.name}` : file.name;
    files.push(makeStagedFile(file, rel));
    return files;
  }
  if (!entry.isDirectory) return files;

  const reader = entry.createReader();
  const readAll = async () => {
    const batch = await new Promise((resolve, reject) => {
      reader.readEntries(resolve, reject);
    });
    if (!batch.length) return;
    for (const child of batch) {
      const sub = prefix ? `${prefix}/${child.name}` : child.name;
      files.push(...(await walkFileEntry(child, sub)));
    }
    await readAll();
  };
  await readAll();
  return files;
}

/** 拖放收集：目录走 Entry 递归（前缀为拖入文件夹名），压缩包用 dataTransfer.files */
async function collectDropFiles(dataTransfer) {
  const items = [...dataTransfer.items].filter((i) => i.kind === "file");
  for (const item of items) {
    const entry = item.webkitGetAsEntry?.();
    if (entry?.isDirectory) {
      const files = await walkFileEntry(entry, entry.name || "");
      if (files.length) return files;
    }
  }
  return toFileArray(dataTransfer.files);
}

function setupUploadDropZone() {
  const zone = document.getElementById("upload-drop");
  const archiveInput = document.getElementById("hand-upload-input");
  const folderInput = document.getElementById("hand-folder-input");
  const clearBtn = document.getElementById("upload-clear");

  zone.addEventListener("click", (e) => {
    if (e.target.closest("#upload-clear")) return;
    openUploadPicker(e.shiftKey);
  });
  archiveInput.addEventListener("change", () => onFilesReady(archiveInput.files));
  folderInput.addEventListener("change", () => onFilesReady(folderInput.files));
  clearBtn?.addEventListener("click", (e) => {
    e.preventDefault();
    e.stopPropagation();
    clearPendingUpload();
  });

  zone.addEventListener("dragenter", (e) => {
    e.preventDefault();
  });
  zone.addEventListener("dragover", (e) => {
    e.preventDefault();
    zone.classList.add("dragover");
  });
  zone.addEventListener("dragleave", (e) => {
    if (e.currentTarget.contains(e.relatedTarget)) return;
    zone.classList.remove("dragover");
  });
  zone.addEventListener("drop", async (e) => {
    e.preventDefault();
    zone.classList.remove("dragover");
    try {
      const files = await collectDropFiles(e.dataTransfer);
      onFilesReady(files);
    } catch (err) {
      alert(t("alert.dropFailed", { msg: err.message || String(err) }));
    }
  });
}

/** 上传手型配置（压缩包或文件夹），成功后刷新手型列表 */
async function uploadHandConfig() {
  const btn = document.getElementById("upload-hand");
  const pkg = pendingUploadPackage;

  if (!pkg) {
    alert(t("alert.pickFirst"));
    setUploadStatusKey("upload.pickFirst", {}, true);
    return;
  }

  if (pkg.kind === "folder" && !pendingOverwrite) {
    const check = recognizeFolderPackage(pkg, { alertOnFail: false });
    if (!check.ok) {
      setUploadStatusKey(
        check.cancelled ? "upload.cancelled" : "upload.validationFailed",
        {},
        true
      );
      if (check.cancelled) clearPendingUpload();
      return;
    }
    pendingOverwrite = !!check.overwrite;
  }

  const buildForm = (overwrite) => {
    const form = new FormData();
    if (overwrite) form.append("overwrite", "true");
    if (pkg.kind === "archive") {
      const name = pkg.uploadName || archiveBasename(pkg.file);
      form.append("files", pkg.file, name);
      return form;
    }
    for (const { relativePath, file } of pkg.entries) {
      form.append("files", file, relativePath);
    }
    return form;
  };

  btn.disabled = true;
  btn.textContent = t("btn.uploadBusy");
  setUploadStatusKey("upload.uploading", {}, false);
  try {
    let overwrite = pendingOverwrite;
    let res = await fetch("/api/v1/hands/configs/upload", {
      method: "POST",
      body: buildForm(overwrite),
    });
    let payload = {};
    try {
      payload = await res.json();
    } catch (_) {
      payload = {};
    }

    if (res.status === 409) {
      const hand = apiCollisionHand(payload);
      if (!confirmOverwrite(hand || t("confirm.overwriteFallback"))) {
        setUploadStatusKey("upload.cancelled", {}, false);
        return;
      }
      overwrite = true;
      res = await fetch("/api/v1/hands/configs/upload", {
        method: "POST",
        body: buildForm(true),
      });
      try {
        payload = await res.json();
      } catch (_) {
        payload = {};
      }
    }

    if (!res.ok) {
      const rawDetail = apiErrorRaw(payload, res.statusText);
      alert(t("alert.uploadFailed", { detail: localizeApiDetail(rawDetail) }));
      setUploadStatusKey("upload.failed", { detail: rawDetail }, true);
      return;
    }
    const overwrote = overwrite ? t("upload.overwrote") : "";
    setUploadStatusKey("upload.success", { hand: payload.hand, overwrote }, false);
    const keepSelection = getSelectedHands();
    if (payload.hands) {
      renderHandOptions(payload.hands, keepSelection);
    } else {
      await loadHands(keepSelection);
    }
    await loadExistingHandDirs();
    await refreshStatus();
    clearPendingUpload();
  } finally {
    btn.disabled = false;
    btn.textContent = t("btn.upload");
  }
}

/** 应用手型（侧别由探测拓扑决定；不勾选时清除已选手型） */
document.getElementById("apply").onclick = async () => {
  const btn = document.getElementById("apply");
  const hands = getSelectedHands();
  if (hands.length === 0 && !confirm(t("confirm.clearHands"))) {
    return;
  }
  btn.disabled = true;
  btn.dataset.busy = "1";
  btn.textContent = t("btn.applyBusy");
  let ok = false;
  try {
    const res = await fetch("/api/v1/hands/select", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ hands }),
    });
    if (!res.ok) {
      let rawDetail = res.statusText;
      try {
        const err = await res.json();
        rawDetail = apiErrorRaw(err, res.statusText);
      } catch (_) {
        /* 忽略 */
      }
      alert(t("alert.applyFailed", { detail: localizeApiDetail(rawDetail) }));
      return;
    }
    const data = await res.json();
    ok = true;
    setSelectedHands(statusActiveHands(data));
    await subscribeWsFromServer();
    await refreshStatus();
    if (typeof window.refreshVizPage === "function") window.refreshVizPage();
  } finally {
    delete btn.dataset.busy;
    btn.textContent = t("btn.apply");
    if (ok) syncHandsApplyBaseline(getSelectedHands());
    else updateApplyButtonState();
  }
};

/** WS 各 stream 最新一帧（首页下拉展示用） */
const wsStreamCache = new Map();
let wsSubscribedStreams = [];
/** 从 GET /streams 解析的全局流 + 手型作用域 base_id，用于勾选时展开下拉项 */
let streamCatalogTemplate = null;

async function loadStreamCatalogTemplate() {
  try {
    const res = await fetch("/api/v1/streams");
    const items = await res.json();
    const subscribe = items.filter((s) => s.direction === "subscribe");
    const global = [];
    const handScopedBases = new Set();
    for (const s of subscribe) {
      const baseId = s.base_id || s.id;
      if (s.scope === "hand") handScopedBases.add(baseId);
      else global.push(baseId);
    }
    streamCatalogTemplate = {
      global: [...new Set(global)],
      handScopedBases: [...handScopedBases],
    };
  } catch (_) {
    streamCatalogTemplate = {
      global: [
        "io_esk.tf",
        "io_esk.joint_data",
        "io_esk.joystick_data",
        "io_esk.imu_data_right",
        "io_esk.imu_data_left",
      ],
      handScopedBases: [
        "io_align.tf",
        "io_align.poses_left",
        "io_align.poses_right",
        "io_teleop.joint_cmd_left",
        "io_teleop.joint_cmd_right",
      ],
    };
  }
}

/**
 * 内置页面默认订阅（可视化 + 终端调试够用）。
 * 全量 stream id 仍见 GET /api/v1/streams；客户/脚本可自选任意组合。
 */
const WS_UI_GLOBAL = [
  "io_esk.joint_data",
  "io_esk.imu_data_right",
  "io_esk.imu_data_left",
];
/** 客户 publish 流：网关 UI 订阅以展示调用与数据（见 gateway.yaml publish_streams） */
const WS_UI_PUBLISH = ["io_esk.vibration_feedback"];
const WS_UI_HAND_BASES = ["io_teleop.joint_cmd_left", "io_teleop.joint_cmd_right"];
const WS_DEFAULT_STREAM = "io_esk.joint_data";
/** stream 目录不可用时的最低订阅（与 WS_UI_GLOBAL / WS_UI_PUBLISH 一致） */
const WS_SUBSCRIBE_FALLBACK = [...WS_UI_GLOBAL, ...WS_UI_PUBLISH];

function wsSubscribeFallbackIds(handIds) {
  const ids = [...WS_SUBSCRIBE_FALLBACK];
  for (const base of WS_UI_HAND_BASES) {
    for (const h of handIds || []) ids.push(`${base}.${h}`);
  }
  return [...new Set(ids)].sort();
}

function pickDefaultWsStream(ids) {
  if (ids.includes(WS_DEFAULT_STREAM)) return WS_DEFAULT_STREAM;
  const jointCmd = ids.find((id) => id.includes("joint_cmd") || id.includes("joint_data"));
  if (jointCmd) return jointCmd;
  return ids[0];
}

function streamIdsForHands(handIds) {
  const tpl = streamCatalogTemplate;
  if (!tpl) return [];
  const ids = [...tpl.global];
  for (const base of tpl.handScopedBases) {
    for (const h of handIds) ids.push(`${base}.${h}`);
  }
  return [...new Set(ids)].sort();
}

/** 内置 UI 精简订阅列表 */
function streamIdsForUiSubscribe(handIds) {
  const tpl = streamCatalogTemplate;
  if (!tpl) {
    console.warn("[WS] streamCatalogTemplate 未加载，使用 fallback 订阅");
    return wsSubscribeFallbackIds(handIds);
  }
  const globalSet = new Set(tpl.global);
  const handBaseSet = new Set(tpl.handScopedBases);
  const ids = WS_UI_GLOBAL.filter((id) => globalSet.has(id));
  for (const id of WS_UI_PUBLISH) ids.push(id);
  for (const base of WS_UI_HAND_BASES) {
    if (!handBaseSet.has(base)) continue;
    for (const h of handIds) ids.push(`${base}.${h}`);
  }
  const out = [...new Set(ids)].sort();
  if (!out.length) {
    console.warn("[WS] subscribe 列表为空，使用 fallback");
    return wsSubscribeFallbackIds(handIds);
  }
  return out;
}

/** 已应用手型优先，否则用勾选（尚未 apply 时） */
async function resolveSubscribeHands() {
  try {
    const st = await (await fetch("/api/v1/status")).json();
    if (statusActiveHands(st).length) return statusActiveHands(st);
  } catch (_) {
    /* ignore */
  }
  return getSelectedHands();
}

function pruneWsStreamCache(allowedIds) {
  const allowed = new Set(allowedIds);
  for (const key of [...wsStreamCache.keys()]) {
    if (!allowed.has(key)) wsStreamCache.delete(key);
  }
}

function rebuildWsStreamPicker() {
  const sel = document.getElementById("ws-stream-pick");
  if (!sel) return;
  const prev = sel.value;
  const ids = [...wsSubscribedStreams].sort();
  sel.innerHTML = "";
  if (!ids.length) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = t("ws.noStream");
    sel.appendChild(opt);
    sel.disabled = true;
    return;
  }
  sel.disabled = false;
  for (const id of ids) {
    const opt = document.createElement("option");
    opt.value = id;
    opt.textContent = wsStreamCache.has(id) ? id : `${id} · ${t("ws.waitingData")}`;
    sel.appendChild(opt);
  }
  if (prev && ids.includes(prev)) sel.value = prev;
  else sel.value = pickDefaultWsStream(ids);
  renderWsPanel();
}

/** 按当前手型刷新 WS 下拉（apply 后 subscribeWs=true 会同步精简订阅） */
async function refreshWsStreamPickerFromSelection(subscribeWs = false) {
  if (!streamCatalogTemplate) await loadStreamCatalogTemplate();
  const hands = await resolveSubscribeHands();
  wsSubscribedStreams = streamIdsForUiSubscribe(hands);
  pruneWsStreamCache(wsSubscribedStreams);
  rebuildWsStreamPicker();
  const sock = ws;
  if (subscribeWs && sock?.readyState === WebSocket.OPEN && wsSubscribedStreams.length) {
    sock.send(JSON.stringify({ op: "subscribe", streams: wsSubscribedStreams }));
  }
}

function bindHandCheckboxStreamRefresh() {
  const box = document.getElementById("hand-checkboxes");
  if (!box) return;
  if (!box.dataset.streamRefreshBound) {
    box.dataset.streamRefreshBound = "1";
    box.addEventListener("change", (e) => {
      if (e.target.matches('input[type="checkbox"][data-hand]')) {
        refreshWsStreamPickerFromSelection(false);
        updateApplyButtonState();
      }
    });
  }
  refreshWsStreamPickerFromSelection(false);
}

function renderWsPanel() {
  const pre = document.getElementById("ws");
  const sel = document.getElementById("ws-stream-pick");
  if (!pre) return;
  const sock = ws;
  if (!sock || sock.readyState !== WebSocket.OPEN) {
    pre.textContent = wsReconnectTimer ? t("ws.reconnecting") : t("ws.disconnected");
    if (sel) {
      sel.innerHTML = "";
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = t("ws.noStream");
      sel.appendChild(opt);
      sel.disabled = true;
    }
    return;
  }
  if (!sel || !sel.value) {
    pre.textContent = t("ws.waitingData");
    return;
  }
  const entry = wsStreamCache.get(sel.value);
  if (!entry) {
    pre.textContent = t("ws.noDataYet");
    return;
  }
  const text = JSON.stringify(
    { stream: entry.stream, data: entry.data, receivedAt: entry.receivedAt },
    null,
    2
  );
  pre.textContent = text.length > 16000 ? `${text.slice(0, 16000)}\n…` : text;
}

let wsPickerRebuildScheduled = false;

function scheduleWsPickerRebuild() {
  if (wsPickerRebuildScheduled) return;
  wsPickerRebuildScheduled = true;
  requestAnimationFrame(() => {
    wsPickerRebuildScheduled = false;
    rebuildWsStreamPicker();
  });
}

function ingestWsMessage(raw) {
  let obj;
  try {
    obj = JSON.parse(raw);
  } catch (_) {
    return;
  }
  if (obj.op === "published" || obj.op === "error") return;
  const stream = obj.stream;
  const data = obj.data;
  if (!stream || data === undefined) return;
  wsStreamCache.set(stream, {
    stream,
    data,
    receivedAt: new Date().toISOString(),
  });
  scheduleWsPickerRebuild();
  const sel = document.getElementById("ws-stream-pick");
  if (sel && sel.value === stream) renderWsPanel();
}

/** 内置 UI：只订阅需要的流；全量目录见 GET /api/v1/streams */
async function subscribeWsFromServer({ force = false } = {}) {
  await loadStreamCatalogTemplate();
  const hands = await resolveSubscribeHands();
  const streams = streamIdsForUiSubscribe(hands);
  wsSubscribedStreams = streams;
  pruneWsStreamCache(streams);
  rebuildWsStreamPicker();
  if (!streams.length) {
    console.error("[WS] 无法构造 subscribe 列表");
    return;
  }
  const sock = ws;
  if (!sock || sock.readyState !== WebSocket.OPEN) return;
  const key = streams.join("\0");
  if (!force && key === wsLastSubscribedKey && wsLastSubscribedConnId === wsConnectionId) {
    return;
  }
  sock.send(JSON.stringify({ op: "subscribe", streams }));
  wsLastSubscribedKey = key;
  wsLastSubscribedConnId = wsConnectionId;
}

let ws = null;
let wsIntentionalClose = false;
let wsReconnectAttempt = 0;
let wsReconnectTimer = null;
let wsConnectionId = 0;
let wsLastSubscribedKey = "";
let wsLastSubscribedConnId = 0;
const WS_RECONNECT_BASE_MS = 1000;
const WS_RECONNECT_MAX_MS = 30000;

function scheduleWsReconnect() {
  if (wsIntentionalClose || wsReconnectTimer) return;
  const delay = Math.min(
    WS_RECONNECT_BASE_MS * 2 ** wsReconnectAttempt,
    WS_RECONNECT_MAX_MS
  );
  wsReconnectAttempt += 1;
  wsReconnectTimer = setTimeout(() => {
    wsReconnectTimer = null;
    connectWebSocket();
  }, delay);
  renderWsPanel();
}

function connectWebSocket() {
  if (wsIntentionalClose) return;
  // CONNECTING / OPEN / CLOSING 均不重复建连；等 onclose 后再由 scheduleWsReconnect 重连
  if (ws && ws.readyState !== WebSocket.CLOSED) {
    return;
  }

  const proto = location.protocol === "https:" ? "wss" : "ws";
  const sock = new WebSocket(`${proto}://${location.host}/ws`);
  ws = sock;

  sock.onopen = () => {
    wsReconnectAttempt = 0;
    wsConnectionId += 1;
    wsLastSubscribedKey = "";
    void subscribeWsFromServer({ force: true });
  };

  sock.onmessage = (ev) => {
    ingestWsMessage(ev.data);
    window.dispatchEvent(new CustomEvent("gateway-ws", { detail: ev.data }));
  };

  sock.onclose = () => {
    if (sock !== ws) return;
    renderWsPanel();
    if (!wsIntentionalClose) scheduleWsReconnect();
  };
}

window.addEventListener("beforeunload", () => {
  wsIntentionalClose = true;
  if (wsReconnectTimer) {
    clearTimeout(wsReconnectTimer);
    wsReconnectTimer = null;
  }
  if (ws) {
    ws.onclose = null;
    ws.close();
    ws = null;
  }
});

connectWebSocket();

document.getElementById("refresh-hands").onclick = async () => {
  await loadHands();
  await loadExistingHandDirs();
};
document.getElementById("upload-hand").onclick = uploadHandConfig;
setupUploadDropZone();

window.onGatewayLangChange = () => {
  restoreButtonLabels();
  refreshUploadStatusDisplay();
  refreshUploadDropDisplay();
  refreshWifiStatusDisplay();
  refreshUdpStatusDisplay();
  refreshWifiPasswordToggleLabels();
  if (lastStatusData) {
    syncHandLabel(
      statusActiveHands(lastStatusData),
      lastStatusData.processes,
      statusConfiguredHands(lastStatusData),
    );
    renderConnectStatusPanels(lastStatusData);
    const toggleBtn = document.getElementById("udp-exo-toggle");
    if (toggleBtn && toggleBtn.dataset.busy !== "1") {
      const udpActive = isUdpExoActive(lastStatusData);
      toggleBtn.dataset.udpActive = udpActive ? "1" : "0";
      toggleBtn.textContent = udpActive ? t("btn.udpExoStop") : t("btn.udpExoStart");
    }
  } else {
    syncHandLabel(null, null, null);
  }
  const statusEl = document.getElementById("status");
  if (statusEl && !lastStatusData) {
    statusEl.textContent = statusPollOffline
      ? lastStatusPollError || t("status.offline")
      : t("status.loading");
  }
  if (statusPollOffline) {
    setGatewayStatusBanner(true, lastStatusPollError || t("status.offline"));
  }
  const sock = ws;
  if (!sock || sock.readyState !== WebSocket.OPEN) renderWsPanel();
  else rebuildWsStreamPicker();
  loadHands(getSelectedHands().length ? getSelectedHands() : null);
};

renderConnectStatusPanels(lastStatusData || {});
void refreshStatus();
initPage();
setInterval(refreshStatus, STATUS_POLL_MS);
