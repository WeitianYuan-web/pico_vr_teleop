import asyncio
import json
import os
import sys
import time
import urllib.request
import websockets

API = os.environ.get("GATEWAY_URL", "http://127.0.0.1:8080").rstrip("/")
WS = os.environ.get("GATEWAY_WS", "ws://127.0.0.1:8080/ws")
DURATION = 3  # 秒


def list_subscribe_ids() -> list[str]:

    with urllib.request.urlopen(f"{API}/api/v1/streams", timeout=5) as resp:
        items = json.loads(resp.read())
    return [x["id"] for x in items if x.get("direction") == "subscribe" and x.get("id")]


async def active_stream_ids(stream_ids: list[str]) -> list[str]:
    
    active: set[str] = set()
    deadline = time.time() + DURATION
    async with websockets.connect(WS) as ws:
        await ws.send(json.dumps({"op": "subscribe", "streams": stream_ids}))
        while time.time() < deadline:
            try:
                raw = await asyncio.wait_for(ws.recv(), timeout=0.3)
            except asyncio.TimeoutError:
                continue
            obj = json.loads(raw)
            sid = obj.get("stream")
            if sid and isinstance(obj.get("data"), dict):
                active.add(sid)
    return sorted(active)


if __name__ == "__main__":
    try:
        ids = list_subscribe_ids()
    except OSError as e:
        print(f"无法访问 {API}/api/v1/streams: {e}", file=sys.stderr)
        sys.exit(1)

    if not ids:
        print("无 subscribe 流（gateway 未启动或未选手型？）")
        sys.exit(1)

    print(f"订阅 {len(ids)} 个流，探测 {DURATION}s …")
    active = asyncio.run(active_stream_ids(ids))
    if active:
        print("有数据的 stream id:")
        for sid in active:
            print(f"  {sid}")
    else:
        print("未收到任何数据，检查 gateway / exo / transform 是否已启动")
