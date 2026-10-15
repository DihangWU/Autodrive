#!/usr/bin/env bash

AUTODRIVE_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

run_carla() {
    bash "$AUTODRIVE_ROOT/simuenv/carla/init/carla_init.sh" "$@"
}

choose_town04() {
    bash "$AUTODRIVE_ROOT/simuenv/carla/init/choose_town04.sh" "$@"
}

map_carla2sumo() {
    bash "$AUTODRIVE_ROOT/simuenv/map_convert/carla2sumo.sh" "$@"
}

run_sumo() {
    bash "$AUTODRIVE_ROOT/simuenv/sumo/init/sumo_init.sh" "$@"
}

run_rsu() {
    bash "$AUTODRIVE_ROOT/env_modules/rsu/rsu.sh" "$@"
}

mouse_move_carla() {
    bash "$AUTODRIVE_ROOT/simuenv/carla/mouse_move/mouse_move.sh" "$@"
}
