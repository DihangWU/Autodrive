"""只导入 CARLA 官方联仿的模型、灯光和坐标工具，不启动官方独立时钟。"""
import os
from pathlib import Path
import sys
from types import SimpleNamespace

PROJECT_DIR = Path(__file__).resolve().parents[3]
OFFICIAL_SUMO_DIR = PROJECT_DIR / 'foundation/carla/carla-official/Co-Simulation/Sumo'
SUMO_TOOLS_DIR = Path(os.environ.get('SUMO_HOME', '/usr/share/sumo')) / 'tools'


def load_official_tools():
    for folder in (OFFICIAL_SUMO_DIR, SUMO_TOOLS_DIR):
        if str(folder) not in sys.path:
            sys.path.append(str(folder))
    import carla
    import sumolib
    import traci.constants as constants
    from sumo_integration.bridge_helper import BridgeHelper
    from sumo_integration.sumo_simulation import SumoActor, SumoActorClass
    from sumo_integration.constants import SPAWN_OFFSET_Z
    return SimpleNamespace(carla=carla, sumolib=sumolib, constants=constants,
        helper=BridgeHelper, actor=SumoActor, actor_class=SumoActorClass,
        spawn_offset_z=SPAWN_OFFSET_Z)
