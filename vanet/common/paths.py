"""Shared project paths, independent of the caller's working directory."""
from pathlib import Path

VANET_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = VANET_DIR.parent
LINCOLN_DIR = PROJECT_DIR / 'lincoln'
CONFIG = VANET_DIR / 'town04.json'
ASSETS_DIR = VANET_DIR / 'assets'
OMNET_DIR = VANET_DIR / 'omnet'
SUMO_CO_SIM_DIR = PROJECT_DIR / 'carla-source/Co-Simulation/Sumo'
