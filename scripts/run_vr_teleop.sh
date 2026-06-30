#!/bin/bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

exec "${PROJECT_DIR}/webxr_test/run_teleop_piper_webxr.sh" "$@"
