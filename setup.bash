#!/usr/bin/env bash

AUTODRIVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 各入口路径集中维护
CARLA_ENTRY="$AUTODRIVE_ROOT/foundation/carla/init/carla_init.sh"
MAP_ENTRY="$AUTODRIVE_ROOT/foundation/carla/init/load_town04.sh"
CONVERT_ENTRY="$AUTODRIVE_ROOT/foundation/carla/map_convert/carla2sumo.sh"
SUMO_ENTRY="$AUTODRIVE_ROOT/foundation/sumo/init/sumo_init.sh"
BRIDGE_ENTRY="$AUTODRIVE_ROOT/foundation/cobridge/run_cobridge.sh"
RSU_ENTRY="$AUTODRIVE_ROOT/modules/rsu/rsu.sh"
LINCOLN_ENTRY="$AUTODRIVE_ROOT/modules/lincoln/add_lincoln.sh"
NPC_ENTRY="$AUTODRIVE_ROOT/modules/npcvehicules/npcvehicules.sh"
CAMERA_ENTRY="$AUTODRIVE_ROOT/foundation/carla/mouse_move/mouse_move.sh"

run_carla() {
    bash "$CARLA_ENTRY" "$@"
}

choose_town04() {
    bash "$MAP_ENTRY" "$@"
}

map_carla2sumo() {
    bash "$CONVERT_ENTRY" "$@"
}

run_sumo() {
    bash "$SUMO_ENTRY" "$@"
}

run_cobridge() {
    bash "$BRIDGE_ENTRY" "$@"
}

run_rsu() {
    bash "$RSU_ENTRY" "$@"
}

mouse_move_carla() {
    bash "$CAMERA_ENTRY" "$@"
}

add_lincoln() {
    bash "$LINCOLN_ENTRY" "$@"
}

add_npc() {
    bash "$NPC_ENTRY" "$@"
}

# VANET：每个独立程序保留单项入口；内部业务由所属主文件调用。
_vanet_python() {
    local vanet_python="${AUTODRIVE_PYTHON:-$HOME/carla-venv/bin/python}"
    PYTHONPATH="$AUTODRIVE_ROOT${PYTHONPATH:+:$PYTHONPATH}" "$vanet_python" -u "$@"
}

run_omnet() {
    _vanet_python -m vanet.omnet.build_and_launch.run_network "$@"
}

build_vanet_network() {
    _vanet_python -m vanet.omnet.build_and_launch.build_network "$@"
}

add_vanet_npc() {
    _vanet_python -m vanet.experiments.experiment_session_management.control_cli initialize "$@"
}

vanet_status() {
    _vanet_python -m vanet.experiments.experiment_session_management.control_cli status "$@"
}

vanet_control() {
    _vanet_python -m vanet.experiments.experiment_session_management.control_cli "$@"
}

run_vanet() {
    local vanet_start_script
    vanet_start_script="$(_vanet_python -c 'from vanet.project_paths import VANET_START_SCRIPT_FILE; print(VANET_START_SCRIPT_FILE)')" || return
    bash "$vanet_start_script" "$@"
}
