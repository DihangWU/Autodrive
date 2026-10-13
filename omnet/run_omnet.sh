#!/usr/bin/env bash
set -eo pipefail
OMNET_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OMNET_ROOT="${OMNET_ROOT:-$HOME/omnetpp-6.1}"
VEINS_ROOT="${VEINS_ROOT:-$HOME/veins-5.3.1}"
source "$OMNET_ROOT/setenv" >/dev/null
export LD_LIBRARY_PATH="$VEINS_ROOT/src:${LD_LIBRARY_PATH:-}"
cd "$OMNET_DIR"
mkdir -p results
exec python3 "$OMNET_DIR/runtime/omnet_session.py" opp_run -u "${OMNET_UI:-Qtenv}" --image-path="$VEINS_ROOT/images" -n ".:$VEINS_ROOT/src/veins" -l "$VEINS_ROOT/src/veins" -l ./town04vanet -c Live "$@"
