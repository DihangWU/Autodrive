"""从现有 Lincoln 网格提取可转动车轮，缓存派生车身和四轮 DAE。

只处理模型中断开的完整几何组件，不用空间平面切割车身。
原模型和材质不改写；车轮资源以各自骨骼中心为原点。
"""
import copy
import fcntl
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from modules.lincoln.runtime.paths import VEHICLE_MESH_FILE, WHEEL_MESH_CACHE_DIR

COLLADA_NAMESPACE = 'http://www.collada.org/2005/11/COLLADASchema'
NS = {'c': COLLADA_NAMESPACE}
WHEEL_GEOMETRY_ID = 'part2'  # 当前模型的轮胎、轮毂和底盘共用此材质网格。
WHEEL_NAMES = ('front_left', 'front_right', 'rear_left', 'rear_right')
BUILDER_FILE = Path(__file__).resolve()
BODY_MESH_NAME = 'body.dae'
MANIFEST_NAME = 'meshes.json'
BUILD_LOCK_NAME = 'build.lock'
MESH_SUFFIX = '.dae'
TEMPORARY_SUFFIX = '.tmp'


def read_geometry(mesh):
    """当前模型的位置和法线共享索引；不支持的格式明确报错。"""
    triangles = mesh.find('c:triangles', NS)
    inputs = triangles.findall('c:input', NS)
    if any(int(item.get('offset', '0')) != 0 for item in inputs):
        raise ValueError('Lincoln 网格的顶点/法线不是共享索引，不能直接拆分。')
    sources = mesh.findall('c:source', NS)
    arrays = [np.fromstring(source.find('c:float_array', NS).text, sep=' ').reshape(-1, 3)
              for source in sources]
    if len(arrays) != 2 or arrays[0].shape != arrays[1].shape:
        raise ValueError('Lincoln 网格的位置和法线格式不符。')
    faces = np.fromstring(triangles.find('c:p', NS).text, sep=' ', dtype=np.int64).reshape(-1, 3)
    return arrays[0], arrays[1], faces


def connected_components(vertices, faces):
    """按重合顶点连接三角形，跨法线接缝也归为同一组件。"""
    unique, welded = np.unique(np.round(vertices, 5), axis=0, return_inverse=True)
    parents = list(range(len(unique)))
    def find(index):
        while parents[index] != index:
            parents[index] = parents[parents[index]]
            index = parents[index]
        return index
    for a, b, c in welded[faces]:
        root = find(int(a))
        parents[find(int(b))] = root
        parents[find(int(c))] = root
    return np.array([find(int(index)) for index in welded])


def select_wheel_faces(vertices, faces, geometry):
    """提取轮胎、轮毂、轮栓和制动盘，保留车身与固定制动钳。"""
    if len(geometry['wheels']) != len(WHEEL_NAMES):
        raise ValueError('当前 Lincoln 可视化需要四个车轮的几何配置。')
    labels = connected_components(vertices, faces)
    center = np.asarray(geometry['center'], dtype=float)
    center[2] -= geometry['height'] / 2
    wheels = []
    used = np.zeros(len(faces), dtype=bool)
    for name, wheel in zip(WHEEL_NAMES, geometry['wheels']):
        pivot = np.array([wheel['x'], wheel['y'], wheel['z']]) - center
        radius = wheel['radius']
        chosen = []
        for label in np.unique(labels):
            points = vertices[labels == label]
            relative = points - pivot
            if (np.max(np.linalg.norm(relative[:, [0, 2]], axis=1)) > radius * 1.12
                    or np.max(np.abs(relative[:, 1])) > radius * .5):
                continue
            span = np.ptp(points, axis=0)
            middle = (points.max(0) + points.min(0)) / 2
            outer_face = np.sign(pivot[1]) * (middle[1] - pivot[1]) > .045
            tire = min(span[0], span[2]) > radius * 1.3
            disc = (min(span[0], span[2]) > .25
                    and np.linalg.norm((middle - pivot)[[0, 2]]) < .025)
            if outer_face or tire or disc:
                chosen.append(label)
        mask = np.isin(labels[faces[:, 0]], chosen)
        if mask.sum() < 500 or np.any(used & mask):
            raise ValueError(f'{name} 车轮提取不完整或与其他轮重叠，停止生成。')
        used |= mask
        wheels.append(dict(name=name, pivot=pivot.tolist(), faces=faces[mask]))
    return faces[~used], wheels


def replace_geometry(mesh, vertices, normals, faces, pivot):
    """压缩引用的顶点并平移原点，法线及材质绑定保持原样。"""
    used, indices = np.unique(faces.ravel(), return_inverse=True)
    for source, values in zip(mesh.findall('c:source', NS), (vertices[used] - pivot, normals[used])):
        array = source.find('c:float_array', NS)
        array.text = ' '.join(format(float(value), '.9g') for value in values.flat)
        array.set('count', str(values.size))
        source.find('c:technique_common/c:accessor', NS).set('count', str(len(used)))
    triangles = mesh.find('c:triangles', NS)
    triangles.set('count', str(len(faces)))
    triangles.find('c:p', NS).text = ' '.join(map(str, indices))


def write_mesh(template, path, vertices, normals, faces, pivot, wheel_only):
    """写派生模型；轮子只保留对应网格，车身保留其他门窗灯具。"""
    root = copy.deepcopy(template)
    library = root.find('c:library_geometries', NS)
    for item in list(library):
        if item.get('id') == WHEEL_GEOMETRY_ID:
            replace_geometry(item.find('c:mesh', NS), vertices, normals, faces, np.asarray(pivot))
        elif wheel_only:
            library.remove(item)
    if wheel_only:
        scene = root.find('c:library_visual_scenes/c:visual_scene', NS)
        for item in list(scene):
            instance = item.find('c:instance_geometry', NS)
            if instance is None or instance.get('url') != '#' + WHEEL_GEOMETRY_ID:
                scene.remove(item)
    ET.ElementTree(root).write(path, encoding='utf-8', xml_declaration=True)


def prepare_wheel_meshes(geometry):
    """首次生成，后续按模型、几何配置和生成代码哈希复用；多车共享锁。"""
    source = VEHICLE_MESH_FILE.read_bytes()
    digest = hashlib.sha256(source + json.dumps(geometry, sort_keys=True).encode()
                            + BUILDER_FILE.read_bytes()).hexdigest()[:16]
    folder = WHEEL_MESH_CACHE_DIR / digest
    folder.mkdir(parents=True, exist_ok=True)
    manifest = folder / MANIFEST_NAME
    with (WHEEL_MESH_CACHE_DIR / BUILD_LOCK_NAME).open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if manifest.exists():
            result = json.loads(manifest.read_text())
            if all((folder / name).is_file() for name in [result['body']] + [w['file'] for w in result['wheels']]):
                return folder, result
        ET.register_namespace('', COLLADA_NAMESPACE)
        root = ET.fromstring(source)
        mesh = root.find(f'c:library_geometries/c:geometry[@id="{WHEEL_GEOMETRY_ID}"]/c:mesh', NS)
        vertices, normals, faces = read_geometry(mesh)
        body_faces, wheels = select_wheel_faces(vertices, faces, geometry)
        write_mesh(root, folder / BODY_MESH_NAME, vertices, normals, body_faces, [0, 0, 0], False)
        result = dict(body=BODY_MESH_NAME, wheels=[])
        for wheel in wheels:
            name = wheel['name'] + MESH_SUFFIX
            write_mesh(root, folder / name, vertices, normals, wheel['faces'], wheel['pivot'], True)
            result['wheels'].append(dict(name=wheel['name'], file=name, pivot=wheel['pivot']))
        temporary = manifest.with_suffix(TEMPORARY_SUFFIX)
        temporary.write_text(json.dumps(result, indent=2))
        temporary.replace(manifest)
        return folder, result
