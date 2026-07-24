"""
io_gateway 主入口。
职责：
  - 启动 FastAPI（REST + WebSocket + 静态页）
  - 在 lifespan 中初始化：配置、状态、子进程监管、Zenoh 桥、编排循环
  - 不修改现有算法包，仅 subprocess 调用

启动：
  head（默认）：scripts/run_gateway.sh  或  python -m io_gateway.backend.main
  headless：    scripts/run_gateway.sh --headless  或  python -m io_gateway.backend.main --headless
"""

from __future__ import annotations
import asyncio
import logging
import os
import sys
import time
from contextlib import asynccontextmanager
from pathlib import Path
import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from starlette.requests import Request

from io_gateway.backend.api.routes import router as api_router
from io_gateway.backend.runtime_mode import mode_name, parse_cli
from io_gateway.backend.config_loader import (
    load_gateway_config,
    project_root,
    ws_default_subscribe_stream_ids,
    bootstrap_hand_choose_config,
)
from io_gateway.backend.glove_manager import wifi_heartbeat
from io_gateway.backend.orchestrator.manager import Orchestrator
from io_gateway.backend.orchestrator.supervisor import Supervisor, main_log_path
from io_gateway.backend.zenoh.bridge import ZenohBridge
from io_gateway.backend.state import RuntimeState
from io_gateway.backend.ws.ws_hub import WsHub

logger = logging.getLogger("io_gateway")

_LOG_FMT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"


def setup_gateway_logging(logs_dir: Path) -> Path:
    """主进程日志写入 logs/YYYY-MM-DD/io_gateway.log，同时保留终端输出。"""
    log_file = main_log_path(logs_dir)
    formatter = logging.Formatter(_LOG_FMT)
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.handlers.clear()

    fh = logging.FileHandler(log_file, encoding="utf-8")
    fh.setFormatter(formatter)
    root.addHandler(fh)

    sh = logging.StreamHandler()
    sh.setFormatter(formatter)
    root.addHandler(sh)

    # 应用层 middleware 已记录 API；降低 uvicorn 访问日志重复
    logging.getLogger("uvicorn.access").setLevel(logging.WARNING)

    return log_file

def _pkg_dir() -> Path:
    """io_gateway 包根（templates、static）；编译为 .so 时 __file__ 不可用。"""
    root = os.environ.get("IO_EXOTRANS2HAND_ROOT", "").strip()
    if root:
        return Path(root).resolve() / "src" / "io_gateway"
    return Path(__file__).resolve().parents[1]


# io_gateway/ 包根目录（templates、static 所在）
PKG_DIR = _pkg_dir()

_STATIC_ASSET_FILES = ("app.js", "i18n.js", "viz.js")


def static_asset_rev() -> str:
    """静态资源 cache-bust 版本（随 JS 文件修改时间变化）。"""
    mtimes: list[int] = []
    static_dir = PKG_DIR / "static"
    for name in _STATIC_ASSET_FILES:
        path = static_dir / name
        if path.is_file():
            mtimes.append(int(path.stat().st_mtime))
    return str(max(mtimes)) if mtimes else "0"


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期：启动时拉起各组件，退出时停编排与子进程。"""
    cfg = load_gateway_config()
    bootstrap_hand_choose_config(cfg)
    os.environ.setdefault("IO_EXOTRANS2HAND_ROOT", str(cfg.root))
    os.environ.setdefault(
        "ZENOH_CONFIG",
        str(cfg.root / "configs" / "config" / "zenoh.json5"),
    )
    state = RuntimeState()
    supervisor = Supervisor(cfg, state)       # 子进程启停
    ws_hub = WsHub(max_fps=cfg.websocket_max_fps)
    ws_hub.set_loop(asyncio.get_running_loop())
    ws_hub.start()

    # Zenoh 订阅在独立线程，回调线程安全地投递到 ws_hub
    bridge = ZenohBridge(cfg, on_message=ws_hub.ingest)
    bridge.start()
    orchestrator = Orchestrator(cfg, state, supervisor, bridge=bridge)  # USB 探测 + 编排

    wifi_heartbeat.configure_from_gateway(cfg.gateway_raw)
    wifi_heartbeat.start_listener()

    app.state.cfg = cfg
    app.state.state = state
    app.state.supervisor = supervisor
    app.state.orchestrator = orchestrator
    app.state.ws_hub = ws_hub
    app.state.bridge = bridge

    cfg.logs_dir.mkdir(parents=True, exist_ok=True)
    ui_mode = mode_name(getattr(app.state, "headless", False))
    ws_fps = cfg.websocket_max_fps
    logger.info(
        "io_gateway 启动 mode=%s root=%s port=%s ws_max_fps=%s",
        ui_mode,
        cfg.root,
        cfg.listen_port,
        "off" if ws_fps <= 0 else ws_fps,
    )
    await supervisor.sweep_orphan_processes(include_exo=True)
    orchestrator.start()  # 后台 probe 循环

    yield

    await orchestrator.stop()
    await ws_hub.stop()
    bridge.stop()
    wifi_heartbeat.stop_listener()
    logger.info("io_gateway 已停止")


def create_app(*, headless: bool = False) -> FastAPI:
    """构建 FastAPI 应用。head：控制台 + 静态资源；headless：仅 API + WebSocket。"""
    title = "IO Gateway (headless)" if headless else "IO Gateway"
    app = FastAPI(title=title, lifespan=lifespan)
    app.state.headless = headless

    @app.middleware("http")
    async def log_api_requests(request: Request, call_next):
        path = request.url.path
        if not path.startswith("/api/v1"):
            return await call_next(request)
        client = request.client.host if request.client else "-"
        t0 = time.monotonic()
        response = await call_next(request)
        if path == "/api/v1/status":
            return response
        ms = (time.monotonic() - t0) * 1000
        # logger.info(
        #     "[HTTP] %s %s -> %s (%.0fms) client=%s",
        #     request.method,
        #     path,
        #     response.status_code,
        #     ms,
        #     client,
        # )
        return response

    app.include_router(api_router)
    cfg = load_gateway_config()

    if not headless:
        from fastapi.staticfiles import StaticFiles
        from fastapi.templating import Jinja2Templates

        templates = Jinja2Templates(directory=str(PKG_DIR / "templates"))
        app.mount("/static", StaticFiles(directory=str(PKG_DIR / "static")), name="static")

        root = project_root()
        exo_assets = root / "configs" / "exoskeleton_urdf"
        if exo_assets.is_dir():
            app.mount(
                "/assets/exo",
                StaticFiles(directory=str(exo_assets)),
                name="assets_exo",
            )
        hands_assets = cfg.end_tools_dir
        if hands_assets.is_dir():
            app.mount(
                "/assets/hands",
                StaticFiles(directory=str(hands_assets)),
                name="assets_hands",
            )

        @app.get("/")
        async def index(request: Request):
            """Web 控制台（head 模式）。"""
            cfg = request.app.state.cfg
            response = templates.TemplateResponse(
                request,
                "index.html",
                {
                    "gateway_version": cfg.version,
                    "asset_rev": static_asset_rev(),
                },
            )
            response.headers["Cache-Control"] = "no-cache, no-store, must-revalidate"
            response.headers["Pragma"] = "no-cache"
            return response
    else:
        @app.get("/")
        async def headless_root() -> JSONResponse:
            """headless 根路径：指向 API，无 HTML 控制台。"""
            return JSONResponse(
                {
                    "mode": mode_name(True),
                    "message": "io_gateway headless — 使用 REST / WebSocket，无 Web UI",
                    "api_prefix": "/api/v1",
                    "bootstrap": "/api/v1/bootstrap",
                    "status": "/api/v1/status",
                    "websocket": "/ws",
                    "docs": "/docs",
                }
            )

    @app.websocket("/ws")
    async def websocket_endpoint(ws: WebSocket):
        """
        WebSocket 数据面。
        连接时不推送；客户端发 subscribe 选择流（全量目录见 GET /api/v1/streams）。
        客户端可发：
          {"op":"subscribe","streams":["io_esk.joint_data","io_teleop.joint_cmd_left.HandA",...]}
          {"op":"publish","stream":"io_esk.vibration_feedback","data":{"data":[...]}}
        服务端推送：{"stream":"io_esk.tf","data":{...}}
        省略 streams 时按 default_streams / default_streams_by_side 展开（见 config_loader）。
        """
        cfg = app.state.cfg
        state = app.state.state
        bridge = app.state.bridge
        async with state.lock:
            hands = list(state.hands)
            side = state.effective_side()
        subscribe_fallback = ws_default_subscribe_stream_ids(cfg, hands, side)
        client = ws.client.host if ws.client else "-"
        await app.state.ws_hub.connect(ws, [])
        logger.info("[WS] 连接 client=%s", client)
        try:
            while True:
                raw = await ws.receive_text()
                try:
                    import json

                    msg = json.loads(raw)
                    op = msg.get("op")
                    if op == "subscribe":
                        if "streams" in msg and isinstance(msg["streams"], list):
                            streams = msg["streams"]
                        else:
                            streams = subscribe_fallback
                        await app.state.ws_hub.set_subscriptions(ws, streams)
                        # logger.info(
                        #     "[WS] subscribe client=%s count=%d streams=%s",
                        #     client,
                        #     len(streams),
                        #     streams[:8] if len(streams) > 8 else streams,
                        # )
                    elif op == "publish":
                        stream_id = msg.get("stream")
                        data = msg.get("data")
                        ok = bool(
                            stream_id
                            and isinstance(data, dict)
                            and bridge.publish(stream_id, data)
                        )
                        if ok:
                            app.state.ws_hub.ingest(stream_id, data)
                            reply = json.dumps(
                                {"op": "published", "stream": stream_id}
                            )
                        else:
                            reply = json.dumps(
                                {
                                    "op": "error",
                                    "reason": "publish_failed",
                                    "stream": stream_id,
                                }
                            )
                            logger.warning(
                                "[WS] publish 失败 client=%s stream=%s",
                                client,
                                stream_id,
                            )
                        logger.info(
                            "[WS] publish client=%s stream=%s ok=%s",
                            client,
                            stream_id,
                            ok,
                        )
                        try:
                            await ws.send_text(reply)
                        except Exception:
                            break
                    else:
                        logger.warning(
                            "[WS] 未知 op client=%s op=%s",
                            client,
                            op,
                        )
                except json.JSONDecodeError:
                    logger.warning("[WS] 无效 JSON client=%s raw=%s", client, raw[:120])
        except WebSocketDisconnect:
            logger.info("[WS] 断开 client=%s", client)
        finally:
            await app.state.ws_hub.disconnect(ws)

    return app





def main(argv: list[str] | None = None) -> None:
    """uvicorn 启动入口。"""
    args = parse_cli(argv)
    root = project_root()
    src_dir = root / "src"
    for p in (src_dir, root):
        if p.is_dir() and str(p) not in sys.path:
            sys.path.insert(0, str(p))
    cfg = load_gateway_config()
    log_file = setup_gateway_logging(cfg.logs_dir)
    ui_mode = mode_name(args.headless)
    logger.info("主进程日志 -> %s mode=%s", log_file, ui_mode)
    app = create_app(headless=args.headless)
    uvicorn.run(
        app,
        host=cfg.listen_host,
        port=cfg.listen_port,
        reload=False,
        log_config=None,
    )


if __name__ == "__main__":
    main()
