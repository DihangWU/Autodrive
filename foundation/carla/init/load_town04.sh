#!/usr/bin/env bash
set -euo pipefail

# 路径与可调参数；向 Python 传参，避免在 heredoc 中重复配置。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
PYTHON_BIN="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"
CARLA_HOST="${1:-127.0.0.1}"
CARLA_PORT="${2:-2000}"
MAP_NAME="${3:-Town04}"
RPC_TIMEOUT="${4:-120}"

"$PYTHON_BIN" - "$CARLA_HOST" "$CARLA_PORT" "$MAP_NAME" "$RPC_TIMEOUT" <<'PYTHON'
import sys
import carla

host, port, map_name, timeout = sys.argv[1:]
client = carla.Client(host, int(port))
client.set_timeout(float(timeout))
world = client.load_world(map_name)
print('当前地图：', world.get_map().name)
PYTHON
