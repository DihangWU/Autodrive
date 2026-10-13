#!/usr/bin/env bash
set -e

~/carla-venv/bin/python - <<'PY'
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(120)
world = client.load_world('Town04')
print('当前地图：', world.get_map().name)
PY