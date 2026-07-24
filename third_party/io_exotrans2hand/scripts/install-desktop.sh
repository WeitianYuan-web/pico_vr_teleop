#!/usr/bin/env bash
# 将 io-gateway.desktop 安装到桌面，并可选安装外骨骼串口 udev 规则。
#
# 用法：
#   ./scripts/install-desktop.sh
#   ./scripts/install-desktop.sh ~/Desktop/io-gateway.desktop
#   ./scripts/install-desktop.sh --no-app-menu
#   ./scripts/install-desktop.sh --no-udev
#   IO_EXOTRANS2HAND_ROOT=/opt/io_project ./scripts/install-desktop.sh
#
# 说明：
#   - .desktop 的 Icon 只认绝对路径，须用本脚本展开 @IO_ROOT@
#   - 默认安装 configs/udev/99-io-exo-serial.rules（需 sudo）
#   - 并将当前用户加入 dialout 组（新加入须注销后重新登录）
set -euo pipefail

INSTALL_APP_MENU=1
INSTALL_UDEV=1

_desktop_dir() {
  if command -v xdg-user-dir >/dev/null 2>&1; then
    xdg-user-dir DESKTOP 2>/dev/null && return
  fi
  for candidate in "$HOME/Desktop" "$HOME/桌面"; do
    if [[ -d "$candidate" ]]; then
      echo "$candidate"
      return
    fi
  done
  echo "$HOME/Desktop"
}

_write_desktop() {
  local dest="$1"
  mkdir -p "$(dirname "$dest")"
  sed "s|@IO_ROOT@|$ROOT|g" "$TEMPLATE" > "$dest"
  chmod +x "$dest"
  if command -v gio >/dev/null 2>&1; then
    gio set "$dest" metadata::trusted true 2>/dev/null || true
  fi
}

_user_in_group() {
  local group="$1"
  id -nG "${2:-$USER}" 2>/dev/null | tr ' ' '\n' | grep -qx "$group"
}

_install_serial_udev() {
  local src="$ROOT/configs/udev/99-io-exo-serial.rules"
  local dest="/etc/udev/rules.d/99-io-exo-serial.rules"

  if [[ ! -f "$src" ]]; then
    echo "警告: 缺少 udev 规则模板 $src，已跳过串口权限安装" >&2
    return 0
  fi

  if ! command -v sudo >/dev/null 2>&1; then
    echo "警告: 未找到 sudo，无法安装 udev 规则。请手动执行：" >&2
    echo "  sudo cp $src $dest" >&2
    echo "  sudo udevadm control --reload-rules && sudo udevadm trigger" >&2
  else
    echo "安装串口 udev 规则 -> $dest"
    sudo cp "$src" "$dest"
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    echo "udev 规则已生效（ttyACM* / ttyUSB* → 组 dialout）"
  fi

  if _user_in_group dialout; then
    echo "用户 $USER 已在 dialout 组"
  elif command -v sudo >/dev/null 2>&1; then
    echo "将用户 $USER 加入 dialout 组…"
    sudo usermod -aG dialout "$USER"
    echo "已加入 dialout 组；请注销并重新登录（或重启）后串口权限才会生效"
  else
    echo "警告: 请让管理员执行: sudo usermod -aG dialout $USER" >&2
  fi
}

_show_help() {
  sed -n '2,14p' "$0" | sed 's/^# \?//'
}

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${IO_EXOTRANS2HAND_ROOT:-$(cd "$SCRIPT_DIR/.." && pwd)}"
TEMPLATE="$ROOT/io-gateway.desktop"
ICON="$ROOT/configs/IO.png"

DEST=""
for arg in "$@"; do
  case "$arg" in
    --no-app-menu)
      INSTALL_APP_MENU=0
      ;;
    --no-udev)
      INSTALL_UDEV=0
      ;;
    -h|--help)
      _show_help
      exit 0
      ;;
    -*)
      echo "未知参数: $arg" >&2
      exit 1
      ;;
    *)
      if [[ -z "$DEST" ]]; then
        DEST="$arg"
      else
        echo "多余参数: $arg" >&2
        exit 1
      fi
      ;;
  esac
done

DEST="${DEST:-$(_desktop_dir)/io-gateway.desktop}"

if [[ ! -f "$TEMPLATE" ]]; then
  echo "缺少模板: $TEMPLATE" >&2
  exit 1
fi
if [[ ! -f "$ICON" ]]; then
  echo "缺少图标: $ICON" >&2
  exit 1
fi

_write_desktop "$DEST"

APP_DEST=""
if [[ "$INSTALL_APP_MENU" -eq 1 ]]; then
  APP_DEST="$HOME/.local/share/applications/io-gateway.desktop"
  _write_desktop "$APP_DEST"
  if command -v update-desktop-database >/dev/null 2>&1; then
    update-desktop-database "$HOME/.local/share/applications" 2>/dev/null || true
  fi
fi

if [[ "$INSTALL_UDEV" -eq 1 ]]; then
  _install_serial_udev
fi

echo "已安装桌面快捷方式: $DEST"
if [[ -n "$APP_DEST" ]]; then
  echo "应用菜单: $APP_DEST（可在启动器搜索「IO Gateway」或「IO Gesture」）"
fi
echo "项目根: $ROOT"
echo "图标:   $ICON"
