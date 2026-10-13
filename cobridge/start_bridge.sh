#!/usr/bin/env bash
set -euo pipefail
# 脚本遇到问题时及时停止

BRIDGE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# 获取脚本的绝对路径

####################################################################################
################################## Map Bridge ######################################
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    exec "$HOME/carla-venv/bin/python" "$BRIDGE_DIR/map/map_bridge.py" "$@"
fi # 如果第一个参数是 --help 或 -h，就在当前终端显示帮助，不进入后台启动流程。

mkdir -p "$BRIDGE_DIR/runtime"
BRIDGE_LOG="$BRIDGE_DIR/runtime/map_bridge.log"

# 覆盖写日志
nohup "$HOME/carla-venv/bin/python" -u "$BRIDGE_DIR/map/map_bridge.py" "$@" \
    >"$BRIDGE_LOG" 2>&1 </dev/null &  
# nohup = 关闭终端继续执行，& = 后台运行让终端 立刻可用，$@ = 原样传递脚本参数
####################################################################################
####################################################################################


BRIDGE_PID=$!  # $! = 启动台进程编号，方便查找定位
echo "Bridge 程序已在后台启动，PID：$BRIDGE_PID"
echo "启动结果和错误请查看日志：$BRIDGE_LOG"
