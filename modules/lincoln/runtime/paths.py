"""Lincoln 专属资源与运行产物路径，统一在此处维护。"""
from pathlib import Path

MODULE_DIR = Path(__file__).resolve().parents[1]
PROJECT_DIR = MODULE_DIR.parents[1]
CONFIG_DIR = MODULE_DIR / 'config'
SENSOR_CONFIG_FILE = CONFIG_DIR / 'sensors.json'
GEOMETRY_CONFIG_FILE = CONFIG_DIR / 'vehicle_geometry.json'
RVIZ_TEMPLATE_FILE = CONFIG_DIR / 'lincoln.rviz'
VEHICLE_MESH_FILE = MODULE_DIR / 'assets' / 'lincoln.dae'
RUNTIME_DIR = PROJECT_DIR / '.runtime' / 'lincoln'
CAMERA_RUNTIME_DIR = RUNTIME_DIR / 'camera'
PROC_DIR = Path('/proc')
RVIZ_BINARY = 'rviz2'
