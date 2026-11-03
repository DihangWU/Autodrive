"""同一交通边界采样：CARLA 坐标供无线物理层，应用接收仍经过无线链路。"""
import math


def collect_mobility(bridge, rsu):
    snapshot=bridge.carla.world.get_snapshot()
    states=[]
    for record in bridge.lincoln.records.values():
        actor=record.get('actor') or bridge.carla.world.get_actor(record['actor_id'])
        state=snapshot.find(record['actor_id'])
        if actor is None or state is None:continue
        transform=state.get_transform();box=actor.bounding_box
        # 包围盒中心用于碰撞；无线节点采用同一中心的天线高度。
        center=transform.transform(box.location)
        # 唯一实验车的无线身份固定；SUMO 投影重连后编号可能变化。
        states.append(dict(id='lincoln0',sumo_id=record['sumo_id'],role='lincoln',x=center.x,y=center.y,z=center.z,
            heading=transform.rotation.yaw,speed=state.get_velocity().length(),
            length=2*box.extent.x,width=2*box.extent.y,height=2*box.extent.z))
    connection=bridge.sumo.connection
    offset=bridge.sumo.network['offset']
    present=set(connection.vehicle.getIDList())
    for batch in bridge.npc.batches.values():
        for name in batch['traffic'].created:
            if name not in present:continue
            point=connection.vehicle.getPosition3D(name)
            angle=connection.vehicle.getAngle(name)
            length=connection.vehicle.getLength(name)
            heading=angle-90.;radians=math.radians(heading)
            # SUMO 坐标在车头，转回车身中心；Y 轴翻转。
            x=point[0]-offset[0]-length/2*math.cos(radians)
            y=offset[1]-point[1]-length/2*math.sin(radians)
            states.append(dict(id=name,role='npc',x=x,y=y,z=point[2]+.75,
                heading=heading,speed=connection.vehicle.getSpeed(name),length=length,
                width=connection.vehicle.getWidth(name),height=1.5,
                lane_id=connection.vehicle.getLaneID(name),road_id=connection.vehicle.getRoadID(name),
                route=list(connection.vehicle.getRoute(name))))
    nodes=[{k:v for k,v in state.items() if k in ('id','role','x','y','z','heading','speed')} for state in states]
    nodes.append(dict(rsu,role='rsu',speed=0.,heading=0.))
    return states,nodes
