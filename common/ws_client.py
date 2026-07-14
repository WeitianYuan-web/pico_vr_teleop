"""WebXR WSS 客户端循环（收包 + 可选并行控制任务）。"""

from __future__ import annotations

import asyncio
import json
import ssl
from collections.abc import Awaitable, Callable
from typing import Any


async def run_webxr_ws_loop(
    ws_uri: str,
    on_payload: Callable[[dict], None],
    *,
    control_coro_factory: Callable[[], Awaitable[Any]] | None = None,
    reconnect_delay_s: float = 2.0,
    connected_message: str | None = None,
) -> None:
    """
    /**
     * @brief 连接 WebXR WSS，解析 JSON 后回调；断线自动重连
     *
     * @param on_payload 每帧 VR 数据回调（同步）
     * @param control_coro_factory 若提供，连接成功后 create_task 并行控制环
     */
    """
    import websockets

    ssl_ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    ssl_ctx.check_hostname = False
    ssl_ctx.verify_mode = ssl.CERT_NONE
    print(f"[Network] 连接 WebXR: {ws_uri}")
    while True:
        try:
            async with websockets.connect(ws_uri, ssl=ssl_ctx) as ws:
                if connected_message:
                    print(connected_message)
                else:
                    print("[Network] WebXR 已连接")
                control_task = None
                if control_coro_factory is not None:
                    control_task = asyncio.create_task(control_coro_factory())
                try:
                    while True:
                        msg = await ws.recv()
                        try:
                            payload = json.loads(msg)
                        except json.JSONDecodeError:
                            continue
                        if isinstance(payload, dict):
                            on_payload(payload)
                finally:
                    if control_task is not None:
                        control_task.cancel()
                        try:
                            await control_task
                        except asyncio.CancelledError:
                            pass
        except Exception as exc:
            print(f"[Network] 连接中断: {exc}，{reconnect_delay_s:g} 秒后重连")
            await asyncio.sleep(reconnect_delay_s)
