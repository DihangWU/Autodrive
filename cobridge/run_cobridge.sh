#!/usr/bin/env bash
set -euo pipefail
BRIDGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 独立地图导出/预览仍使用现有入口；默认启动完整共享联仿桥。
if [[ "${1:-}" == "--map-only" ]]; then
    shift
    exec "$HOME/carla-venv/bin/python" -u "$BRIDGE_DIR/map/map_bridge.py" "$@"
fi
exec "$HOME/carla-venv/bin/python" -u "$BRIDGE_DIR/bridge/ensure_bridge.py" "$@"
