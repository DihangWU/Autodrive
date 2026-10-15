#!/usr/bin/env bash
set -euo pipefail

PANEL_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
for argument in "$@"; do
    if [[ "$argument" == "--help" || "$argument" == "-h" ]]; then
        exec "$HOME/carla-venv/bin/python" -u "$PANEL_DIR/mouse_move.py" "$@"
    fi
done

PROJECT_DIR="$(cd -- "$PANEL_DIR/../../.." && pwd)"
LOG_DIR="$PROJECT_DIR/.runtime/carla/mouse_move"
mkdir -p "$LOG_DIR"
LOG_FILE="$LOG_DIR/panel.log"

nohup "$HOME/carla-venv/bin/python" -u "$PANEL_DIR/mouse_move.py" "$@" \
    >>"$LOG_FILE" 2>&1 </dev/null &
PANEL_PID=$!

echo "CARLA 自由视角面板已后台启动，PID：$PANEL_PID"
echo "日志：$LOG_FILE"
