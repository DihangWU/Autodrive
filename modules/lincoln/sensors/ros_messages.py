"""CARLA 传感器数据到 ROS 2 消息的转换；复用 f511f80。"""
import math
import cv2
import numpy as np
from builtin_interfaces.msg import Time
from sensor_msgs.msg import CameraInfo

# Match Demo's camera order; optional extra rows remain explicitly empty.
CAMERA_ROWS = (('roof_sensor_rig/standard', 'roof_sensor_rig/tele'),
               ('surround_camera_rig/rear', 'roof_sensor_rig/wide'),
               ('surround_camera_rig/b_left', 'surround_camera_rig/b_right'),
               ('surround_camera_rig/left', 'surround_camera_rig/right'))

def camera_mosaic(images, specs, rows):
    tile_height, tile_width, title_height = 392, 640, 32
    canvas = np.zeros((rows * tile_height, 2 * tile_width, 3), dtype=np.uint8)
    for row in range(rows):
        for col in range(2):
            tile = canvas[row * tile_height:(row + 1) * tile_height,
                          col * tile_width:(col + 1) * tile_width]
            name = CAMERA_ROWS[row][col] if row < len(CAMERA_ROWS) else None
            title = 'UNUSED - no camera' if name is None else name.split('/')[-1].upper()
            if name in specs:
                title += f' | {math.degrees(specs[name]["fov"]):.0f} deg'
            if name in images:
                tile[title_height:] = images[name]
            elif name is not None:
                title += ' | waiting for data'
            cv2.putText(tile, title, (12, 23), cv2.FONT_HERSHEY_SIMPLEX, .6, (255, 255, 255), 1)
    return canvas


def stamp(seconds):
    ns = round(seconds * 1e9)
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def set_rotation(message, angles):
    message.x, message.y, message.z, message.w = quaternion(*angles)


def local_pose(spec, geometry):
    pose = list(spec['pose'])
    cx, cy, cz = geometry['center']
    pose[0] += cx
    pose[1] += cy
    pose[2] += cz - geometry['height'] / 2
    return pose


def camera_info(spec, header):
    info = CameraInfo(header=header, width=spec['width'], height=spec['height'])
    focal = spec['width'] / (2 * math.tan(spec['fov'] / 2))
    cx, cy = spec['width'] / 2, spec['height'] / 2
    info.distortion_model = 'plumb_bob'
    info.d = [0.] * 5
    info.k = [focal, 0., cx, 0., focal, cy, 0., 0., 1.]
    info.r = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    info.p = [focal, 0., cx, 0., 0., focal, cy, 0., 0., 0., 1., 0.]
    return info


def sensor_points(data, kind):
    raw = np.frombuffer(data.raw_data, dtype=np.float32).reshape(-1, 4)
    if kind == 'lidar':
        result = raw.copy()
        result[:, 1] *= -1
    else:
        # CARLA radar raw fields: velocity, azimuth, altitude, depth.
        velocity, azimuth, altitude, depth = raw.T
        horizontal = depth * np.cos(altitude)
        result = np.column_stack((horizontal * np.cos(azimuth),
                                  -horizontal * np.sin(azimuth),
                                  depth * np.sin(altitude), velocity))
    return np.asarray(result, dtype='<f4')
