"""
REST API 路由（控制面）。

前缀 /api/v1，与 IO_PLATFORM_SPEC §7.1 对齐。
数据面走 WebSocket /ws，不走 REST。
"""

from __future__ import annotations
import asyncio
import re
from typing import Any, Optional
import logging
from fastapi import APIRouter, File, Form, HTTPException, Query, Request, UploadFile
from fastapi.responses import Response
from pydantic import BaseModel, Field

from io_gateway.backend.config_loader import (
    asset_http_root_for_exo,
    asset_http_root_for_hand,
    build_subscribe_stream_catalog,
    exo_asset_urls,
    exo_urdf_rel_path,
    hand_asset_urls,
    hand_urdf_rel_in_hand_dir,
    list_hand_dirs,
    list_valid_hands,
    load_wifi_provision_config,
    resolve_urdf_path,
    rewrite_urdf_for_browser,
    save_hand_choose_config,
    save_wifi_provision_config,
    viz_urdf_api_url,
)
from io_gateway.backend.glove_manager import ESPTouchConfig, glove_manager, wifi_heartbeat
from io_gateway.backend.hand_upload import handle_hand_upload
from io_gateway.backend.state import ExoLinkMode, build_status_payload
from io_gateway.backend.orchestrator.probe_cycle import build_udp_allowlist_snapshot

router = APIRouter(prefix="/api/v1")
logger = logging.getLogger(__name__)




def _form_bool(value: str | bool | None) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    return str(value).strip().lower() in ("1", "true", "yes", "on")


async def _sync_hands_list(request: Request) -> list[str]:
    """从磁盘扫描可用手型并写回 RuntimeState.hands_list（仅在显式刷新手型列表时调用）。"""
    hands = list_valid_hands(request.app.state.cfg)
    async with request.app.state.state.lock:
        request.app.state.state.hands_list = list(hands)
    return hands

class HandSelectBody(BaseModel):
    hands: list[str] = Field(default_factory=list)

    def resolved_hands(self) -> list[str]:
        return list(dict.fromkeys(self.hands))

class FrequencyBody(BaseModel):
    hz: int = Field(gt=0, le=500)


_IPV4_RE = re.compile(r"^(\d{1,3}\.){3}\d{1,3}$")


class WifiProvisionBody(BaseModel):
    ssid: str = Field(min_length=1, max_length=64)
    password: str = Field(min_length=8, max_length=128)
    return_ip: str = Field(min_length=7, max_length=45)


@router.get("/bootstrap")
async def bootstrap(request: Request) -> dict[str, Any]:
    """
    GET headless 启动信息：手型列表 + 当前端口扫描（无需浏览器）。
    等价于启动日志中的报告，JSON 形式。
    """
    from io_gateway.backend.headless.report import build_startup_report

    cfg = request.app.state.cfg
    exo = request.app.state.supervisor.exo_running()
    state = request.app.state.state
    report = build_startup_report(cfg, exo_running=exo)
    payload = report.to_dict()
    async with state.lock:
        payload["active_hands"] = list(state.hands)
        payload["configured_hands"] = list(cfg.hand_choose)
        payload["available_hands"] = list(report.hands)
    logger.info(
        "[API] bootstrap topology=%s hands=%d selected=%s saved=%s",
        payload.get("topology"),
        len(payload.get("hands") or []),
        payload.get("active_hands"),
        payload.get("configured_hands"),
    )
    return payload


@router.get("/probe")
async def probe_now(request: Request) -> dict[str, Any]:
    """GET 立即重新探测拓扑（串口或 UDP，不等待 probe 周期）。"""
    from io_gateway.backend.orchestrator.probe import ProbeResult, probe_topology
    from io_gateway.backend.orchestrator.probe_udp import devices_to_udp_port_scans, probe_udp_topology
    from io_gateway.backend.state import ExoLinkMode

    cfg = request.app.state.cfg
    state = request.app.state.state
    exo = request.app.state.supervisor.exo_running()
    async with state.lock:
        udp_mode = state.exo_link_mode == ExoLinkMode.UDP
    if udp_mode:
        result = await asyncio.to_thread(probe_udp_topology, cfg)
    else:
        result = await asyncio.to_thread(probe_topology, cfg, exo_running=exo)
    async with state.lock:
        if exo:
            # exo 占用串口时无法重新 open；拓扑/设备读运行时缓存
            payload = {
                "topology": state.topology.value,
                "devices": dict(state.topology_devices),
                "port_scans": [e.to_dict() for e in (result.port_scans or [])],
                "exo_running": True,
            }
        else:
            payload = {
                "topology": result.topology.value,
                "devices": dict(result.devices),
                "port_scans": [e.to_dict() for e in (result.port_scans or [])],
                "exo_running": False,
            }
            state.topology = result.topology
            state.topology_devices = dict(result.devices)
            state.last_probe = {
                "port_scans": payload.get("port_scans") or [],
                "exo_running": payload.get("exo_running"),
            }
    logger.info(
        "[API] probe topology=%s devices=%s exo_running=%s",
        payload["topology"],
        payload["devices"],
        payload["exo_running"],
    )
    return payload

# 获取手型配置列表
@router.get("/hands/configs")
async def list_hands(request: Request) -> list[dict[str, str]]:
    """GET 可用手型列表（扫描 end_tools，并同步到 state.hands_list）。"""
    hands = await _sync_hands_list(request)
    logger.info("[API] hands/configs 有效手型 %d 个", len(hands))
    return [{"id": h} for h in hands]

# 内部使用，不对外
@router.get("/hands/dirs")
async def list_hand_dirs_api(request: Request) -> list[dict[str, str]]:
    """GET end_tools 下全部目录名（用于上传撞名检测）。"""
    cfg = request.app.state.cfg
    dirs = list_hand_dirs(cfg)
    logger.info("[API] hands/dirs 目录 %d 个", len(dirs))
    return [{"id": d} for d in dirs]


@router.post("/hands/configs/upload")
async def upload_hand_config(
    request: Request,
    files: list[UploadFile] = File(...),
    name: str | None = Form(None),
    overwrite: str | None = Form(None),
) -> dict[str, Any]:
    """
    POST 上传手型配置到 configs/end_tools/<hand>/。
    Content-Type: multipart/form-data

    表单字段：
      - files（必填，可重复）：压缩包（单文件）或文件夹内多个文件（filename 为相对路径）
      - name（可选）：覆盖自动识别的手型名
      - overwrite（可选）：传 true/1/yes 时覆盖同名目录

    压缩包：zip、tar、tar.gz、tgz、tar.bz2、tar.xz 等。
    包/文件夹结构须含 urdf/、meshes/ 及 3 个 yml（tf_transform_v2.yml 等）。

    成功 200：{"ok": true, "hand": "...", "valid": true, "hands": [{"id": "..."}, ...]}
    撞名 409：{"detail": {"message": "...", "hand": "...", "collision": true}}
    """
    cfg = request.app.state.cfg
    cfg.end_tools_dir.mkdir(parents=True, exist_ok=True)
    ow = _form_bool(overwrite)

    try:
        result = await handle_hand_upload(cfg, files, name, overwrite=ow)
    except HTTPException as exc:
        logger.warning(
            "[手型上传] API 失败 status=%s detail=%s",
            exc.status_code,
            exc.detail,
        )
        raise

    async with request.app.state.state.lock:
        request.app.state.state.hands_list = [h["id"] for h in result["hands"]]
    logger.info(
        "[API] hands/configs/upload 成功 hand=%s valid=%s",
        result.get("hand"),
        result.get("valid"),
    )
    return result

# 配置手型选择
@router.post("/hands/select")
async def select_hand(body: HandSelectBody, request: Request) -> dict[str, Any]:
    """POST 选手型（hands 数组；空数组表示清空并停止 transform/controller）。"""
    cfg = request.app.state.cfg
    hands = body.resolved_hands()
    logger.info("[手型] POST /hands/select 收到 %d 个: %s", len(hands), hands or "(清空)")
    if hands:
        valid = set(list_valid_hands(cfg))
        unknown = [h for h in hands if h not in valid]
        if unknown:
            logger.warning("[API] hands/select 未知手型: %s", unknown)
            raise HTTPException(400, f"未知手型: {', '.join(unknown)}")
    orch = request.app.state.orchestrator
    await orch.on_hands_select(hands)
    try:
        save_hand_choose_config(cfg.gateway_path, hands)
        cfg.hand_choose = list(dict.fromkeys(hands))
    except OSError as exc:
        logger.exception("[API] hands/select 保存 hand_choose 失败")
        raise HTTPException(500, f"保存 hand_choose 失败: {exc}") from exc
    state = request.app.state.state
    async with state.lock:
        hands_out = list(state.hands)
        side = state.effective_side()
        topo = state.topology.value
    result = {
        "ok": True,
        "active_hands": hands_out,
        "configured_hands": list(cfg.hand_choose),
        "exo": {"controller_side": side},
    }
    logger.info(
        "[API] hands/select 完成 active_hands=%s controller_side=%s topology=%s",
        result["active_hands"],
        side,
        topo,
    )
    return result

# 配置外骨骼数据发布频率
@router.post("/runtime/frequency")
async def set_frequency(body: FrequencyBody, request: Request) -> dict[str, Any]:
    """POST 修改 exo_tf 发布频率（Hz），写 yaml 并重启 exo_tf。"""
    logger.info("[API] runtime/frequency 请求 hz=%s", body.hz)
    orch = request.app.state.orchestrator
    await orch.on_frequency_change(body.hz)
    logger.info("[API] runtime/frequency 完成 hz=%s", body.hz)
    return {"ok": True, "hz": body.hz}

# 获取数据流目录
@router.get("/streams")
async def list_streams(request: Request) -> list[dict[str, Any]]:
    """
    GET 数据流目录。
    手型相关流按当前 state.hands 展开为 stream_id.robotname（如 io_align.tf.BrainCo_Revo2）。
    """
    cfg = request.app.state.cfg
    state = request.app.state.state
    async with state.lock:
        hands = list(state.hands)
    items: list[dict[str, Any]] = build_subscribe_stream_catalog(cfg, hands)
    for p in cfg.publish_streams:
        entry: dict[str, Any] = {
            "id": p.id,
            "topic": p.topic,
            "type": p.msg_type,
            "direction": "publish",
        }
        if p.data_length:
            entry["data_length"] = p.data_length
        items.append(entry)
    logger.info(
        "[API] streams 返回 %d 项（hands=%s）",
        len(items),
        hands or "(未选)",
    )
    return items

# 状态获取
@router.get("/status")
async def get_status(request: Request) -> dict[str, Any]:
    """GET 运行时快照：exo、手型、无线模块、子进程、Zenoh 桥。"""
    state = request.app.state.state
    bridge = request.app.state.bridge
    supervisor = request.app.state.supervisor
    cfg = request.app.state.cfg
    ready_ips = wifi_heartbeat.get_ready_ips()
    online_ips = wifi_heartbeat.get_online_ips()
    async with state.lock:
        bound = dict(state.topology_devices)
        payload = build_status_payload(
            state,
            configured_hands=list(cfg.hand_choose),
            exo_running=supervisor.exo_running(),
            zenoh_bridge=bridge.available,
            wireless=wifi_heartbeat.get_status(),
            udp_allowlist=build_udp_allowlist_snapshot(
                cfg, bound, ready_ips, online_ips=online_ips
            ),
        )
    running = [n for n, i in payload.get("processes", {}).items() if i.get("running")]
    exo = payload.get("exo") or {}
    logger.debug(
        "[API] status topology=%s active_hands=%s running=%s zenoh=%s",
        exo.get("topology"),
        payload.get("active_hands"),
        running,
        (payload.get("zenoh") or {}).get("bridge_available"),
    )
    return payload


@router.get("/wifi/heartbeat/devices")
async def wifi_heartbeat_devices() -> dict[str, Any]:
    """GET 无线模块 UDP 心跳在线 IP 列表（8889）。"""
    return wifi_heartbeat.get_status()


@router.get("/wifi/config")
async def get_wifi_config(request: Request) -> dict[str, str]:
    """GET 无线配网默认参数（来自 gateway.yaml wifi_provision）。"""
    cfg = request.app.state.cfg
    payload = load_wifi_provision_config(cfg.gateway_path)
    logger.debug("[API] wifi/config 读取 ssid=%s return_ip=%s", payload["ssid"], payload["return_ip"])
    return payload


@router.put("/wifi/config")
async def put_wifi_config(body: WifiProvisionBody, request: Request) -> dict[str, Any]:
    """PUT 保存无线配网默认参数到 gateway.yaml。"""
    ssid = body.ssid.strip()
    return_ip = body.return_ip.strip()
    if return_ip and not _IPV4_RE.match(return_ip):
        raise HTTPException(400, f"回调 IP 格式无效: {return_ip}")

    cfg = request.app.state.cfg
    try:
        save_wifi_provision_config(
            cfg.gateway_path,
            ssid=ssid,
            password=body.password,
            return_ip=return_ip,
        )
    except FileNotFoundError as e:
        raise HTTPException(404, str(e)) from e
    except OSError as e:
        logger.error("[API] wifi/config 写入失败: %s", e)
        raise HTTPException(500, f"写入配置失败: {e}") from e

    logger.info("[API] wifi/config 已保存 ssid=%s return_ip=%s", ssid, return_ip)
    return {"ok": True, "ssid": ssid, "return_ip": return_ip}


@router.post("/wifi/provision")
async def wifi_provision(body: WifiProvisionBody) -> dict[str, Any]:
    """POST ESP-Touch 无线配网：广播 SSID/密码/回调 IP 给待配网设备。"""
    ssid = body.ssid.strip()
    return_ip = body.return_ip.strip()
    if not _IPV4_RE.match(return_ip):
        raise HTTPException(400, f"回调 IP 格式无效: {return_ip}")

    cfg = ESPTouchConfig(ssid, body.password, return_ip)
    logger.info("[API] wifi/provision ssid=%s return_ip=%s", ssid, return_ip)
    success, message = await asyncio.to_thread(glove_manager.start_esp_touch, cfg)
    if not success:
        logger.warning("[API] wifi/provision 失败: %s", message)
        raise HTTPException(400, message)
    logger.info("[API] wifi/provision 成功 ssid=%s", ssid)
    return {"success": True, "message": message}


# 可视化使用
@router.get("/visualization/urdf/exo")
async def visualization_urdf_exo(request: Request) -> Response:
    """
    GET 外骨骼 URDF（读取仓库内原始文件，仅在响应中改写 mesh 路径供浏览器加载）。
    不修改 configs/exoskeleton_urdf 下的源文件。
    """
    cfg = request.app.state.cfg
    path = resolve_urdf_path(cfg, exo_urdf_rel_path(cfg))
    if not path.is_file():
        raise HTTPException(404, f"外骨骼 URDF 不存在: {path}")
    xml = rewrite_urdf_for_browser(
        path.read_text(encoding="utf-8"),
        asset_http_root_for_exo(),
    )
    return Response(content=xml, media_type="application/xml")


@router.get("/visualization/urdf/{hand}")
async def visualization_urdf_hand(
    hand: str,
    request: Request,
    side: str = Query("left", pattern="^(left|right)$"),
) -> Response:
    """GET 灵巧手 URDF（同上，不改 end_tools 源文件）。"""
    cfg = request.app.state.cfg
    if hand not in set(list_valid_hands(cfg)):
        raise HTTPException(404, f"未知手型: {hand}")
    rel = hand_urdf_rel_in_hand_dir(cfg, hand, side)
    if not rel:
        raise HTTPException(404, f"手型 {hand} 无 {side} URDF 配置")
    path = resolve_urdf_path(cfg, f"configs/end_tools/{hand}/{rel}")
    if not path.is_file():
        raise HTTPException(404, f"URDF 不存在: {path}")
    xml = rewrite_urdf_for_browser(
        path.read_text(encoding="utf-8"),
        asset_http_root_for_hand(hand),
    )
    return Response(content=xml, media_type="application/xml")


def _exo_meshes_status(cfg) -> dict[str, Any]:
    """检查外骨骼 STL 是否存在于 configs/exoskeleton_urdf/meshes/（含 v4/v5/v6 等子目录）。"""
    meshes_dir = (cfg.root / "configs" / "exoskeleton_urdf" / "meshes").resolve()
    if not meshes_dir.is_dir():
        return {
            "ready": False,
            "dir": str(meshes_dir),
            "message": "缺少目录 configs/exoskeleton_urdf/meshes/（URDF 引用的 STL 未随仓库提供）",
        }
    stl_count = len(list(meshes_dir.glob("**/*.STL"))) + len(
        list(meshes_dir.glob("**/*.stl"))
    )
    if stl_count == 0:
        return {
            "ready": False,
            "dir": str(meshes_dir),
            "message": "meshes 目录为空，请放入 R_base_link.STL 等网格文件",
        }
    return {"ready": True, "dir": str(meshes_dir), "stl_count": stl_count}


@router.get("/visualization/config")
async def visualization_config(request: Request) -> dict[str, Any]:
    """GET 可视化页 URDF 资源 URL（浏览器走 API 读程序内 URDF，网格仍用 /assets/）。"""
    cfg = request.app.state.cfg
    state = request.app.state.state
    _, exo_pkg = exo_asset_urls(cfg)
    exo_url = viz_urdf_api_url()
    exo_meshes = _exo_meshes_status(cfg)
    async with state.lock:
        selected = list(state.hands)
        side = state.effective_side()
    hands_out: list[dict[str, str]] = []
    for hand in list_valid_hands(cfg):
        left = hand_asset_urls(cfg, hand, "left")
        right = hand_asset_urls(cfg, hand, "right")
        left_rel = hand_urdf_rel_in_hand_dir(cfg, hand, "left")
        right_rel = hand_urdf_rel_in_hand_dir(cfg, hand, "right")
        entry: dict[str, str] = {"id": hand}
        if left:
            entry["urdf_left"] = viz_urdf_api_url(hand, side="left")
            entry["package_left"] = left[1]
        if right:
            entry["urdf_right"] = viz_urdf_api_url(hand, side="right")
            entry["package_right"] = right[1]
        if left_rel:
            entry["urdf_left_rel"] = left_rel
        if right_rel:
            entry["urdf_right_rel"] = right_rel
        if left_rel and right_rel:
            entry["urdf_mode"] = "combined" if left_rel == right_rel else "split"
        elif left_rel or right_rel:
            entry["urdf_mode"] = "combined"
        hands_out.append(entry)
    primary = selected[0] if selected else None
    pick_side = "right" if side == "right" else "left"
    hand_url = hand_pkg = None
    if primary:
        picked = hand_asset_urls(cfg, primary, pick_side) or hand_asset_urls(
            cfg, primary, "left"
        )
        if picked:
            hand_url = viz_urdf_api_url(primary, side=pick_side)
            hand_pkg = picked[1]
    return {
        "exo": {"urdf": exo_url, "package": exo_pkg, "meshes": exo_meshes},
        "selected_hands": selected,
        "side": side,
        "primary_hand": primary,
        "hand": {"urdf": hand_url, "package": hand_pkg, "hand": primary, "side": pick_side}
        if hand_url
        else None,
        "hands": hands_out,
    }
