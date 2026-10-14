"""Project paths, independent of the working directory; official sources are read-only."""
from pathlib import Path
PROJECT_DIR = Path(__file__).resolve().parents[2]
LINCOLN_DIR = PROJECT_DIR / 'scenes/lincoln'
RSU_DIR = PROJECT_DIR / 'scenes/rsu'
VISUAL_DIR = PROJECT_DIR / 'scenes/vanet_visual'
SUMO_CO_SIM_DIR = PROJECT_DIR / 'cobridge/carla-source-official/Co-Simulation/Sumo'
VANET_DIR = PROJECT_DIR / 'vanet'
CONFIG = VANET_DIR / 'town04.json'
RSU_SUMO_DIR = RSU_DIR / 'sumo'
OMNET_DIR = PROJECT_DIR / 'omnet'
