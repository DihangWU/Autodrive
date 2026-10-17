#!/usr/bin/env bash
set -euo pipefail

# 路径与可调参数
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd -- "$SCRIPT_DIR/../../.." && pwd)"
PYTHON_BIN="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"
ENTRY_FILE="$PROJECT_DIR/foundation/carla/mouse_move/mouse_move.py"
LOG_DIR="$PROJECT_DIR/.runtime/carla/mouse_move"
LOG_FILE="$LOG_DIR/panel.log"

for argument in "$@"; do
    if [[ "$argument" == "--help" || "$argument" == "-h" ]]; then
        exec "$PYTHON_BIN" -u "$ENTRY_FILE" "$@"
    fi
done
mkdir -p "$LOG_DIR"

nohup "$PYTHON_BIN" -u "$ENTRY_FILE" "$@" \
    >>"$LOG_FILE" 2>&1 </dev/null &
PANEL_PID=$!

echo "CARLA 自由视角面板已后台启动，PID：$PANEL_PID"
echo "日志：$LOG_FILE"
