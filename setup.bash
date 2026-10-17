#!/usr/bin/env bash

AUTODRIVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 各入口路径集中维护
CARLA_ENTRY="$AUTODRIVE_ROOT/foundation/carla/init/carla_init.sh"
MAP_ENTRY="$AUTODRIVE_ROOT/foundation/carla/init/load_town04.sh"
CONVERT_ENTRY="$AUTODRIVE_ROOT/foundation/carla/map_convert/carla2sumo.sh"
SUMO_ENTRY="$AUTODRIVE_ROOT/foundation/sumo/init/sumo_init.sh"
BRIDGE_ENTRY="$AUTODRIVE_ROOT/foundation/cobridge/run_cobridge.sh"
RSU_ENTRY="$AUTODRIVE_ROOT/modules/rsu/rsu.sh"
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
