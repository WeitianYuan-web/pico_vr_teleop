import asyncio
import http.server
import json
import os
import socket
import socketserver
import ssl
import subprocess
import sys
import threading
import time
from pathlib import Path

import websockets

# 切换到脚本所在目录以便正确伺服 index.html
os.chdir(os.path.dirname(os.path.abspath(__file__)))

# 保存所有已连接的 WebSocket 客户端 (PICO 生产者 + 可视化消费者)
CONNECTED_CLIENTS = set()

CERT_FILE = "cert.pem"
KEY_FILE = "key.pem"


def _iface_ipv4(iface: str) -> str | None:
    """Return first non-loopback IPv4 on iface, or None."""
    try:
        import fcntl
        import struct

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            packed = fcntl.ioctl(
                sock.fileno(),
                0x8915,  # SIOCGIFADDR
                struct.pack("256s", iface[:15].encode("utf-8")),
            )
            return socket.inet_ntoa(packed[20:24])
        finally:
            sock.close()
    except OSError:
        return None


def list_ipv4_candidates() -> list[tuple[str, str, bool]]:
    """[(iface, ip, is_wireless), ...] for UP interfaces with IPv4."""
    net = Path("/sys/class/net")
    out: list[tuple[str, str, bool]] = []
    if not net.is_dir():
        return out
    for iface_path in sorted(net.iterdir()):
        iface = iface_path.name
        if iface == "lo":
            continue
        try:
            state = (iface_path / "operstate").read_text().strip()
        except OSError:
            continue
        if state not in ("up", "unknown"):
            continue
        ip = _iface_ipv4(iface)
        if not ip or ip.startswith("127."):
            continue
        is_wifi = (iface_path / "wireless").exists()
        out.append((iface, ip, is_wifi))
    return out


def get_local_ip():
    """
    Prefer Wi-Fi for the PICO URL (headset is on WLAN).
    Override with WEBXR_HOST_IP / WEBXR_IFACE if needed.
    """
    override = os.environ.get("WEBXR_HOST_IP", "").strip()
    if override:
        return override

    prefer_iface = os.environ.get("WEBXR_IFACE", "").strip()
    candidates = list_ipv4_candidates()

    if prefer_iface:
        for iface, ip, _ in candidates:
            if iface == prefer_iface:
                return ip

    for iface, ip, is_wifi in candidates:
        if is_wifi:
            return ip

    # Prefer non-robot ethernet over Meta/docker tunnels.
    for iface, ip, _ in candidates:
        if ip.startswith("192.168.127."):
            continue
        if iface.lower().startswith(("meta", "docker", "br-", "veth")):
            continue
        return ip

    # Fallback: route-based guess (may pick robot ethernet)
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("10.255.255.255", 1))
        return s.getsockname()[0]
    except Exception:
        if candidates:
            return candidates[0][1]
        return "127.0.0.1"
    finally:
        s.close()


def _cert_san_text() -> str:
    if not os.path.isfile(CERT_FILE):
        return ""
    try:
        return subprocess.check_output(
            ["openssl", "x509", "-in", CERT_FILE, "-noout", "-text"],
            text=True,
            stderr=subprocess.DEVNULL,
        )
    except (OSError, subprocess.CalledProcessError):
        return ""


def _cert_covers_current_ips() -> bool:
    text = _cert_san_text()
    if "Subject Alternative Name" not in text:
        return False
    for _, ip, _ in list_ipv4_candidates():
        if f"IP Address:{ip}" not in text and f"IP Address: {ip}" not in text:
            return False
    return True


def ensure_tls_cert() -> None:
    """
    Ensure self-signed cert covers localhost + current LAN IPs (SAN).
    Old CN=localhost-only certs often make PICO abort large HTTPS downloads mid-way.
    """
    force = os.environ.get("WEBXR_REGEN_CERT", "").strip() in ("1", "true", "yes")
    if not force and _cert_covers_current_ips():
        return

    ips = sorted({ip for _, ip, _ in list_ipv4_candidates()} | {"127.0.0.1"})
    san = "DNS:localhost," + ",".join(f"IP:{ip}" for ip in ips)
    conf = (
        "[req]\n"
        "distinguished_name=req_distinguished_name\n"
        "x509_extensions=v3_req\n"
        "prompt=no\n"
        "[req_distinguished_name]\n"
        "CN=pico-webxr\n"
        "[v3_req]\n"
        f"subjectAltName={san}\n"
        "extendedKeyUsage=serverAuth\n"
    )
    conf_path = Path(".webxr_openssl.cnf")
    conf_path.write_text(conf, encoding="utf-8")
    try:
        subprocess.check_call(
            [
                "openssl",
                "req",
                "-x509",
                "-newkey",
                "rsa:2048",
                "-nodes",
                "-keyout",
                KEY_FILE,
                "-out",
                CERT_FILE,
                "-days",
                "825",
                "-config",
                str(conf_path),
                "-extensions",
                "v3_req",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        print(f"[TLS] 已生成/更新自签证书（SAN: {san}）")
        print("[TLS] 头显若曾点过「继续访问」，请重新接受一次证书警告")
    except (OSError, subprocess.CalledProcessError) as exc:
        print(f"[TLS] 警告: 无法自动生成证书 ({exc})；继续使用现有 cert.pem")
    finally:
        try:
            conf_path.unlink(missing_ok=True)
        except OSError:
            pass


def make_ssl_context():
    """创建并加载自签名证书的 SSL 上下文。"""
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    # Large static assets (three.module.js ~1.2MB) over headset Wi-Fi.
    context.options |= getattr(ssl, "OP_IGNORE_UNEXPECTED_EOF", 0)
    context.load_cert_chain(certfile=CERT_FILE, keyfile=KEY_FILE)
    return context


class QuietThreadingHTTPSServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True
    daemon_threads = True


class WebXRRequestHandler(http.server.SimpleHTTPRequestHandler):
    extensions_map = {
        **getattr(http.server.SimpleHTTPRequestHandler, "extensions_map", {}),
        ".js": "text/javascript",
        ".mjs": "text/javascript",
        ".json": "application/json",
        ".wasm": "application/wasm",
        ".html": "text/html",
    }

    def copyfile(self, source, outputfile):
        try:
            super().copyfile(source, outputfile)
        except (BrokenPipeError, ConnectionResetError, ssl.SSLEOFError, ssl.SSLError):
            # Headset browsers often cancel / drop mid-download; don't crash the worker.
            pass

    def finish(self) -> None:
        try:
            super().finish()
        except (BrokenPipeError, ConnectionResetError, ssl.SSLEOFError, ssl.SSLError):
            pass


def start_http_server():
    """
    启动 HTTPS 服务器，伺服 PICO 网页(index.html) 与主机可视化页面(viz.html)。
    使用多线程：单线程时 three.module.js(~1.2MB) 会卡住后续资源，页面停在「初始化中」。
    """
    PORT = 8000
    context = make_ssl_context()
    try:
        with QuietThreadingHTTPSServer(("0.0.0.0", PORT), WebXRRequestHandler) as httpd:
            httpd.socket = context.wrap_socket(httpd.socket, server_side=True)
            ip = get_local_ip()
            print("[HTTP] 前端网页服务已启动！")
            print(f"[HTTP] PICO 头显访问 (采集端 / 优先无线): https://{ip}:{PORT}/index.html")
            for iface, cand_ip, is_wifi in list_ipv4_candidates():
                tag = "wifi" if is_wifi else "wired"
                mark = " <- 使用中" if cand_ip == ip else ""
                print(f"[HTTP]   {iface} ({tag}): https://{cand_ip}:{PORT}/index.html{mark}")
            print(f"[HTTP] 本机浏览器访问 (可视化端): https://localhost:{PORT}/viz.html")
            print("[HTTP] (自签名证书会有安全警告，需点击「高级」->「继续访问」)")
            print("[HTTP] 可覆盖: WEBXR_HOST_IP=x.x.x.x 或 WEBXR_IFACE=wlp18s0")
            httpd.serve_forever()
    except OSError:
        print(f"\n[HTTP 错误] 端口 {PORT} 无法绑定，请检查占用情况。")
        sys.exit(1)


async def handle_ws(websocket):
    """
    处理 WebSocket 连接：接收 PICO 数据，广播给所有可视化客户端。
    通过限频(10Hz) 打印避免阻塞 asyncio 的单线程事件循环。
    """
    CONNECTED_CLIENTS.add(websocket)
    peer = websocket.remote_address
    print(f"\n[WebSocket] 新客户端接入: {peer} (当前连接数: {len(CONNECTED_CLIENTS)})")

    last_print_time = 0.0

    try:
        async for message in websocket:
            others = CONNECTED_CLIENTS - {websocket}
            if others:
                websockets.broadcast(others, message)

            now = time.time()
            if now - last_print_time >= 0.1:
                last_print_time = now
                try:
                    data = json.loads(message)
                except json.JSONDecodeError:
                    continue

                head = data.get("head")
                if head:
                    head_str = (
                        f"头显(x:{head['x']:5.2f}, y:{head['y']:5.2f}, z:{head['z']:5.2f})"
                    )
                else:
                    head_str = "头显(未追踪)"

                ctrl_str = ""
                for c in data.get("controllers", []):
                    name = "左手" if c.get("handedness") == "left" else "右手"
                    trig = c.get("trigger", 0)
                    grip = c.get("grip", 0)
                    qx = c.get("qx", 0.0)
                    qy = c.get("qy", 0.0)
                    qz = c.get("qz", 0.0)
                    qw = c.get("qw", 1.0)
                    ctrl_str += (
                        f" | {name}(x:{c['x']:5.2f},y:{c['y']:5.2f},z:{c['z']:5.2f} "
                        f"q:[{qx:+.2f},{qy:+.2f},{qz:+.2f},{qw:+.2f}] "
                        f"扳机:{trig:.2f} 握把:{grip:.2f})"
                    )

                print(
                    f"\r[实时数据(10Hz抽样)] {head_str}{ctrl_str}          ",
                    end="",
                    flush=True,
                )

    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        CONNECTED_CLIENTS.discard(websocket)
        print(f"\n[WebSocket] 客户端断开: {peer} (当前连接数: {len(CONNECTED_CLIENTS)})")


async def start_ws_server():
    """启动安全 WebSocket (WSS) 服务器，监听 8081 端口。"""
    print("[WebSocket] 接收/广播服务已启动，正在监听 WSS 端口: 8081")
    ssl_context = make_ssl_context()
    try:
        async with websockets.serve(handle_ws, "0.0.0.0", 8081, ssl=ssl_context):
            await asyncio.Future()
    except OSError:
        print("\n[WebSocket 错误] 端口 8081 无法绑定，请检查。")
        sys.exit(1)


if __name__ == "__main__":
    print("=" * 60)
    print(" Pico WebXR 测试服务端 (HTTPS + 优化后) ".center(54, "="))
    print("=" * 60)

    ensure_tls_cert()
    threading.Thread(target=start_http_server, daemon=True).start()

    try:
        asyncio.run(start_ws_server())
    except KeyboardInterrupt:
        print("\n[系统] 已手动退出服务端。")
