# 02 · Installation & Startup

> Audience: Deployers (end users can jump to [2.6 Quick start](#26-quick-start-checklist))

## 2.1 Prerequisites

| Item | Requirement |
|------|-------------|
| OS | **Ubuntu 22.04**, **x86_64** |
| Python | Bundled **Python 3.10** (`bundle/opt/python/bin/python3`), no system Python needed |
| ROS | Runtime uses prebuilt ROS2 (Humble) components in bundle; hardware bridge scripts need system ROS Humble |
| Desktop (head mode) | Needs `xdg-open` / `sensible-browser` to auto-open the browser |
| Serial permission | User must be in the **dialout** group; USB serial needs udev rules |
| Network | HTTP listens on `0.0.0.0`; wireless UDP probe needs a `10.42.x` address on this host |
| Privilege | Installing udev rules requires **sudo** |

Build metadata is in `bundle/BUILD_INFO`: `ubuntu=22.04 / ros_distro=humble / python=3.10 / arch=x86_64-linux-gnu`.

## 2.2 Top-level directory layout

```text
{root}/
├── io-gateway.desktop      # Desktop shortcut template (expanded by install-desktop.sh)
├── bundle/                 # Prebuilt self-contained runtime
│   ├── BUILD_INFO
│   ├── opt/python/         # Bundled Python 3.10
│   ├── opt/io-deps/        # C++ deps + protoc etc.
│   ├── opt/zenoh/          # Zenoh library
│   ├── python/             # pip site-packages
│   └── install/bin/        # Prebuilt nodes: exo_tf_comm / exo_tf_udp_comm / tf_transform_comm
├── src/                    # App source (much is Cython-compiled to .so)
│   ├── io_gateway/         # Gateway (FastAPI + orchestrator + Zenoh bridge + web UI)
│   ├── io_unicontroller/   # Finger retargeting controller
│   └── io_bus_proto/       # Protobuf codec
├── configs/
│   ├── config/
│   │   ├── gateway.yaml    # Main config (port, hands, subprocess commands, bundle paths)
│   │   ├── zenoh.json5     # Zenoh networking (loopback only)
│   │   └── topics.yaml     # Topic mapping
│   ├── end_tools/          # Hand configs (Inspire_RH56F2, Inspire_RH5DG2, ...)
│   ├── exoskeleton_urdf/   # Exoskeleton URDF (3D viz)
│   ├── udev/               # Serial udev rule template
│   └── IO.png              # App icon
├── scripts/
│   ├── install-desktop.sh  # Desktop install
│   ├── run_gateway.sh      # Gateway launch
│   ├── bundle-env.sh       # Environment loader
│   └── Inspire_Hardware_Bridge/  # RS485 bridge scripts
├── logs/YYYY-MM-DD/        # Runtime logs (per-day)
└── tools/                  # Optional debug tools (zenoh2ros_bridge.py etc.)
```

## 2.3 Install: `scripts/install-desktop.sh`

Expands the desktop shortcut template and optionally installs serial udev rules and adds the current user to dialout.

```bash
cd {root}

./scripts/install-desktop.sh                # default: desktop + app menu + udev
./scripts/install-desktop.sh --no-app-menu  # skip app menu
./scripts/install-desktop.sh --no-udev      # skip udev / dialout
IO_EXOTRANS2HAND_ROOT=/opt/io_project ./scripts/install-desktop.sh  # custom project root
```

It does three things:

1. **Generate desktop shortcut**: read `io-gateway.desktop`, replace `@IO_ROOT@` with the absolute project path, write to the desktop and make it executable. Searchable as "IO Gateway" / "IO Gesture".
2. **Install app menu entry** (default): write `~/.local/share/applications/io-gateway.desktop` and refresh the database.
3. **Install udev serial rules** (default, sudo): copy `configs/udev/99-io-exo-serial.rules` to `/etc/udev/rules.d/`, reload + trigger, and `usermod -aG dialout $USER`.

udev rule highlights (`configs/udev/99-io-exo-serial.rules`):

```text
# STM32 Virtual ComPort (common exoskeleton)
SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0660", GROUP="dialout"
# Fallback: all ACM / USB serial
KERNEL=="ttyACM[0-9]*", MODE="0660", GROUP="dialout"
KERNEL=="ttyUSB[0-9]*", MODE="0660", GROUP="dialout"
```

> **Important**: after being added to dialout, you must **log out and back in (or reboot)** for it to take effect, otherwise opening the serial port fails with a permission error.

## 2.4 Startup: `scripts/run_gateway.sh`

### head vs headless

| Mode | Command | Behavior |
|------|---------|----------|
| **head** (default) | `./scripts/run_gateway.sh` | Serves the web console and 3D assets; auto-opens the browser when ready |
| **headless** | `./scripts/run_gateway.sh --headless` | REST API + WebSocket only; root returns JSON guidance; for SSH / systemd |
| head, no browser | `./scripts/run_gateway.sh --no-browser` | or set `GATEWAY_NO_BROWSER=1` |

Mechanism: `run_gateway.sh` first `source scripts/bundle-env.sh` to load the bundle environment (auto-sets `IO_PYTHON`, `PYTHONPATH`, `LD_LIBRARY_PATH`, `PATH`), then launches the gateway with the bundle Python. Since `main` is Cython-compiled to `.so`, it is entered via import rather than `python -m`.

### Environment variables

| Variable | Effect | Notes |
|----------|--------|-------|
| `IO_EXOTRANS2HAND_ROOT` | Project root, path resolution base | Auto-detected; overridable |
| `GATEWAY_PORT` | **Only affects** the browser URL and readiness check in `run_gateway.sh` | Does **not** override the actual listen port |
| `GATEWAY_NO_BROWSER` | Disable auto-open in head mode | `=1` equals `--no-browser` |
| `IO_PYTHON` / `PREFIX` / `PYTHONPATH` / `LD_LIBRARY_PATH` | bundle runtime paths | Set by `bundle-env.sh` |

> To **change the actual listen port**, edit `listen_port` in `configs/config/gateway.yaml`, not just `GATEWAY_PORT`. If the script auto-opens the browser, keep them consistent.

## 2.5 Listen addresses and ports

From `configs/config/gateway.yaml`: `listen_host: 0.0.0.0`, `listen_port: 8080`.

| Service | Address | Notes |
|---------|---------|-------|
| Web console (head) | `http://<host>:8080/` | locally `http://127.0.0.1:8080/` |
| REST API | `http://<host>:8080/api/v1/` | see [07 Configuration Reference](./07-configuration-reference.md) |
| API docs | `http://<host>:8080/docs` | FastAPI auto-generated |
| WebSocket | `ws://<host>:8080/ws` | stream subscribe/publish |
| Wireless UDP probe | `10.42.0.2:8888` | `udp_probe` section |
| Wireless heartbeat | `0.0.0.0:8889` | `wifi_heartbeat` section |
| Zenoh | `tcp/127.0.0.1:0` (dynamic, loopback only) | `zenoh.json5` |

## 2.6 Quick start checklist

1. Confirm the system is **Ubuntu 22.04 x86_64**
2. Place the project in the target directory
3. Run `./scripts/install-desktop.sh` (shortcut + udev + dialout)
4. **Log out and back in** (to apply dialout)
5. Connect the exoskeleton USB devices
6. Run `./scripts/run_gateway.sh` (or double-click the "IO Gesture" desktop icon)
7. Open `http://127.0.0.1:8080/` in a browser

## 2.7 Common ops commands

```bash
# Start (head / headless)
./scripts/run_gateway.sh
./scripts/run_gateway.sh --headless

# Custom project root
IO_EXOTRANS2HAND_ROOT=/opt/io_project ./scripts/run_gateway.sh

# Tail today's main log
tail -f logs/$(date +%Y-%m-%d)/io_gateway.log

# Health check
curl -s http://127.0.0.1:8080/api/v1/status | python3 -m json.tool
```

## 2.8 Build scripts (not needed for operation)

The release is prebuilt; the following are for the build pipeline only: `scripts/cython_build.sh` (Cython build), `scripts/gen_protobuf.sh` (protobuf generation), `scripts/install_protobuf_bundle.sh` (protobuf build/install). See [09 Developer Guide](./09-developer-guide.md).

---

Next: [03 Web Console](./03-web-console.md)
