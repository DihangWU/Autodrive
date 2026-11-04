"""SUMO 路网转为 Qtenv 向下为正的画布；通信半径只读统一实验参数。"""
import json
import math
import xml.etree.ElementTree as ET
from vanet.project_paths import (SUMO_TOWN04_NETWORK_FILE,EXPERIMENT_DEFAULT_CONFIGURATION_FILE,
    OMNET_SCENE_CONFIGURATION_RELATIVE_FILE,OMNET_COMMUNICATION_RANGES_RELATIVE_FILE)


def communication_ranges():
    ranges=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())['communication_ranges']
    for key in ('v2v_metres','rsu_metres'):
        value=ranges.get(key)
        if isinstance(value,bool) or not isinstance(value,(int,float)) or not math.isfinite(value) or value<=0:
            raise ValueError('Invalid communication range: '+key)
    return ranges


def distance_to_lane(point,shape,width):
    distances=[]
    for a,b in zip(shape,shape[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length=dx*dx+dy*dy
        t=0. if length<1e-12 else max(0.,min(1.,((point[0]-a[0])*dx+(point[1]-a[1])*dy)/length))
        distances.append(math.hypot(point[0]-a[0]-t*dx,point[1]-a[1]-t*dy)-width/2.)
    return min(distances) if distances else math.inf


def generate_scene(directory):
    root=ET.parse(SUMO_TOWN04_NETWORK_FILE).getroot()
    offset_x,offset_y=map(float,root.find('location').get('netOffset').split(','))
    lanes=[];all_points=[];highway=[]
    for edge in root.findall('edge'):
        for lane in edge.findall('lane'):
            shape=[tuple(map(float,point.split(',')[:2])) for point in lane.get('shape','').split()]
            if len(shape)<2:continue
            carla_points=[(x-offset_x,offset_y-y) for x,y in shape]
            all_points.extend(carla_points)
            width=float(lane.get('width','3.5'))
            if edge.get('id','').lstrip('-') in {str(i) for i in range(35,51)}:
                highway.append((lane.get('id'),carla_points,width))
            allowed=lane.get('allow','').split();disallowed=lane.get('disallow','').split()
            if 'all' in disallowed or 'passenger' in disallowed or (allowed and 'passenger' not in allowed and 'all' not in allowed):continue
            lanes.append(dict(id=lane.get('id'),internal=edge.get('function')=='internal',width=width,points=carla_points))
    if not all_points:raise ValueError('SUMO network contains no lane geometry.')
    min_x,min_y=min(p[0] for p in all_points),min(p[1] for p in all_points)
    max_x,max_y=max(p[0] for p in all_points),max(p[1] for p in all_points)
    x_shift,y_shift=60.-min_x,60.-min_y
    for lane in lanes:lane['points']=[[x+x_shift,y+y_shift] for x,y in lane['points']]
    parameters=json.loads(EXPERIMENT_DEFAULT_CONFIGURATION_FILE.read_text())
    rsu=parameters['rsu'];ranges=communication_ranges()
    clearance,lane_id=min((distance_to_lane((rsu['x'],rsu['y']),shape,width),name) for name,shape,width in highway)
    if ranges['rsu_metres']>=clearance:
        raise ValueError(f'RSU radius {ranges["rsu_metres"]}m reaches highway lane {lane_id} at {clearance:.3f}m.')
    scene=dict(map='Town04',coordinate_offset_x=x_shift,coordinate_offset_y=y_shift,
        width=max_x-min_x+120.,height=max_y-min_y+120.,lanes=lanes,
        rsu=rsu,communication_ranges=ranges,
        coordinate_convention='Canvas X=CARLA X+offset; canvas Y=CARLA Y+offset (downwards). SUMO Y is inverted once.',
        highway_clearance_metres=clearance,nearest_highway_lane=lane_id,
        highway_coverage_margin_metres=clearance-ranges['rsu_metres'])
    path=directory/OMNET_SCENE_CONFIGURATION_RELATIVE_FILE
    path.write_text(json.dumps(scene,ensure_ascii=False,separators=(',',':'))+'\n')
    (directory/OMNET_COMMUNICATION_RANGES_RELATIVE_FILE).write_text(json.dumps(ranges,indent=2)+'\n')
    return scene,path
