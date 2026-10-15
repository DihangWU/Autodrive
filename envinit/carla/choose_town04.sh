#!/usr/bin/env bash
set -e # 当脚本中的命令执行失败时，立即退出脚本，而不是继续执行后面的命令

~/carla-venv/bin/python - <<'PY'
import carla

client = carla.Client('localhost', 2000)
client.set_timeout(120)
world = client.load_world('Town04')
print('当前地图：', world.get_map().name)
PY