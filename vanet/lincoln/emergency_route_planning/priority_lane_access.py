"""最高路权的车道访问：允许跨标线及逆向行驶，保留实体隔离检查。"""
import math
import carla


def physically_connected(world,first,second):
    a,b=first.transform.location,second.transform.location
    if abs(a.z-b.z)>.5 or a.distance(b)>first.lane_width+1.5:return False
    source=carla.Location(x=a.x,y=a.y,z=a.z+.5)
    target=carla.Location(x=b.x,y=b.y,z=b.z+.5)
    return not any(str(hit.label).split('.')[-1] not in ('Roads','RoadLines','Ground')
                   for hit in world.cast_ray(source,target))


class PriorityWaypoint:
    def __init__(self,waypoint,direction=1):
        self.waypoint,self.direction=waypoint,direction

    def __getattr__(self,name):return getattr(self.waypoint,name)

    @property
    def transform(self):
        transform=self.waypoint.transform
        rotation=transform.rotation
        return carla.Transform(transform.location,carla.Rotation(
            pitch=rotation.pitch*self.direction,yaw=rotation.yaw+(180. if self.direction<0 else 0.),
            roll=rotation.roll*self.direction))

    def next(self,distance):
        following=self.waypoint.next(distance) if self.direction>0 else self.waypoint.previous(distance)
        return [PriorityWaypoint(waypoint,self.direction) for waypoint in following]

    def orient_sample(self,waypoint):
        return PriorityWaypoint(waypoint,self.direction)

    def adjacent(self,world):
        current=self.transform.get_forward_vector()
        for lane in (self.waypoint.get_right_lane(),self.waypoint.get_left_lane()):
            if lane is None or lane.lane_type!=carla.LaneType.Driving:continue
            if lane.road_id!=self.road_id or not physically_connected(world,self.waypoint,lane):continue
            forward=lane.transform.get_forward_vector()
            direction=1 if current.x*forward.x+current.y*forward.y>=0. else -1
            yield PriorityWaypoint(lane,direction)


def priority_key(waypoint):
    return waypoint.road_id,waypoint.section_id,waypoint.lane_id,int(waypoint.s/4.),waypoint.direction
