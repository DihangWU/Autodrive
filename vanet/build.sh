#!/usr/bin/env bash
set -eo pipefail
VANET_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
OMNET_ROOT="${OMNET_ROOT:-$HOME/omnetpp-6.1}"
VEINS_ROOT="${VEINS_ROOT:-$HOME/veins-5.3.1}"
source "$OMNET_ROOT/setenv" >/dev/null
python3 "$VANET_DIR/tools/export_roads.py"
cd "$VANET_DIR/omnet"
opp_makemake --force --make-so -o town04vanet -I"$VEINS_ROOT/src" -L"$VEINS_ROOT/src" -lveins
make MODE=release -j2
