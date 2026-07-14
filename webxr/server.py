import asyncio
import websockets
import http.server
import socketserver
import threading
import json
import os
import sys
import ssl
import socket
import time

# 切换到脚本所在目录以便正确伺服 index.html
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# 保存所有已连接的 WebSocket 客户端 (PICO 生产者 + 可视化消费者)
CONNECTED_CLIENTS = set()

def get_local_ip():
    """
    /**
     * @brief 获取本机的局域网 IP 地址
     */
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        ip = s.getsockname()[0]
    except Exception:
        ip = '127.0.0.1'
    finally:
        s.close()
    return ip

def make_ssl_context():
    """
    /**
     * @brief 创建并加载自签名证书的 SSL 上下文
     */
    """
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.load_cert_chain(certfile="cert.pem", keyfile="key.pem")
    return context

def start_http_server():
    """
    /**
     * @brief 启动 HTTPS 服务器，伺服 PICO 网页(index.html) 与主机可视化页面(viz.html)
     *        监听 8000 端口
     */
    """
    PORT = 8000
    Handler = http.server.SimpleHTTPRequestHandler
    socketserver.TCPServer.allow_reuse_address = True
    context = make_ssl_context()
    try:
        with socketserver.TCPServer(("0.0.0.0", PORT), Handler) as httpd:
            httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
            ip = get_local_ip()
            print(f"[HTTP] 前端网页服务已启动！")
            print(f"[HTTP] PICO 头显访问 (采集端): https://{ip}:{PORT}/index.html")
            print(f"[HTTP] 本机浏览器访问 (可视化端): https://localhost:{PORT}/viz.html")
            print(f"[HTTP] (自签名证书会有安全警告，需点击「高级」->「继续访问」)")
            httpd.serve_forever()
    except OSError:
        print(f"\n[HTTP 错误] 端口 {PORT} 无法绑定，请检查占用情况。")
        sys.exit(1)

async def handle_ws(websocket):
    """
    /**
     * @brief 处理 WebSocket 连接：接收 PICO 数据，广播给所有可视化客户端
     *        通过限频(10Hz) 打印避免阻塞 asyncio 的单线程事件循环
     * @param websocket WebSocket 连接对象
     */
    """
    CONNECTED_CLIENTS.add(websocket)
    peer = websocket.remote_address
    print(f"\n[WebSocket] 新客户端接入: {peer} (当前连接数: {len(CONNECTED_CLIENTS)})")
    
    last_print_time = 0
    
    try:
        async for message in websocket:
            # 1. 广播给除发送者以外的所有客户端 (即转发给可视化页面) - 必须立即转发！
            others = CONNECTED_CLIENTS - {websocket}
            if others:
                websockets.broadcast(others, message)

            # 2. 限制终端打印频率（最大10Hz）
            #    如果在 Cursor 终端、IDE 或 Windows cmd 里，高频（72Hz-90Hz）的 print+flush 会导致 CPU 占用率过高，
            #    阻塞 asyncio 单线程，从而引发 TCP 缓冲区积压，在电脑端出现严重的视觉卡顿！
            now = time.time()
            if now - last_print_time >= 0.1:  # 0.1 秒打印一次，不影响真实的后台高频广播
                last_print_time = now
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue

                head = data.get("head")
                if head:
                    head_str = f"头显(x:{head['x']:5.2f}, y:{head['y']:5.2f}, z:{head['z']:5.2f})"
                else:
                    head_str = "头显(未追踪)"

                ctrl_str = ""
                for c in data.get("controllers", []):
                    name = "左手" if c.get('handedness') == 'left' else "右手"
                    trig = c.get('trigger', 0)
                    grip = c.get('grip', 0)
                    qx = c.get('qx', 0.0)
                    qy = c.get('qy', 0.0)
                    qz = c.get('qz', 0.0)
                    qw = c.get('qw', 1.0)
                    ctrl_str += (f" | {name}(x:{c['x']:5.2f},y:{c['y']:5.2f},z:{c['z']:5.2f} "
                                 f"q:[{qx:+.2f},{qy:+.2f},{qz:+.2f},{qw:+.2f}] "
                                 f"扳机:{trig:.2f} 握把:{grip:.2f})")

                print(f"\r[实时数据(10Hz抽样)] {head_str}{ctrl_str}          ", end="", flush=True)

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CONNECTED_CLIENTS.discard(websocket)
        print(f"\n[WebSocket] 客户端断开: {peer} (当前连接数: {len(CONNECTED_CLIENTS)})")

async def start_ws_server():
    """
    /**
     * @brief 启动安全 WebSocket (WSS) 服务器，监听 8081 端口
     */
    """
    print("[WebSocket] 接收/广播服务已启动，正在监听 WSS 端口: 8081")
    ssl_context = make_ssl_context()
    try:
        async with websockets.serve(handle_ws, "0.0.0.0", 8081, ssl=ssl_context):
            await asyncio.Future()  # 永久运行
    except OSError:
        print(f"\n[WebSocket 错误] 端口 8081 无法绑定，请检查。")
        sys.exit(1)

if __name__ == "__main__":
    print("=" * 60)
    print(" Pico WebXR 测试服务端 (HTTPS + 优化后) ".center(54, "="))
    print("=" * 60)

    threading.Thread(target=start_http_server, daemon=True).start()

    try:
        asyncio.run(start_ws_server())
    except KeyboardInterrupt:
        print("\n[系统] 已手动退出服务端。")