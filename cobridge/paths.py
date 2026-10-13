"""Project paths, independent of the working directory; official sources are read-only."""
from pathlib import Path
PROJECT_DIR = Path(__file__).resolve().parent.parent
BRIDGE_DIR = PROJECT_DIR / 'cobridge/bridge'
LINCOLN_DIR = PROJECT_DIR / 'carla/Lincoln'
RSU_DIR = PROJECT_DIR / 'carla/rsu'
VISUAL_DIR = PROJECT_DIR / 'carla/vanet_visual'
SUMO_CO_SIM_DIR = PROJECT_DIR / 'cobridge/carla-source-official/Co-Simulation/Sumo'
VANET_DIR = PROJECT_DIR / 'vanet'
CONFIG = VANET_DIR / 'town04.json'
SUMO_GUI_ASSETS_DIR = PROJECT_DIR / 'sumo/gui/assets'
OMNET_DIR = PROJECT_DIR / 'omnet'
