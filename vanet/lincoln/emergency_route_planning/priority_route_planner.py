"""最高路权 A*：搜索正向/逆向车道连接，不依赖换道标线许可。"""
import heapq
import itertools
import math
import time
import carla
from modules.lincoln.driving.navigation import PlanningCancelled,ROUTE_SPACING,MAX_SEARCH_NODES,PLANNING_TIMEOUT,TARGET_SNAP_LIMIT,LANE_CHANGE_DISTANCE,LANE_CHANGE_PENALTY
from .priority_lane_access import PriorityWaypoint,priority_key


def plan_priority_route(road_map,world,origin,target,cancelled=lambda:False,heading=None):
    start=road_map.get_waypoint(origin,project_to_road=True,lane_type=carla.LaneType.Driving)
    goal=road_map.get_waypoint(target,project_to_road=True,lane_type=carla.LaneType.Driving)
    if start is None or goal is None or goal.transform.location.distance(target)>TARGET_SNAP_LIMIT:
        raise ValueError('No matching driving surface at the target position and height.')
    direction=1
    if heading is not None and math.cos(math.radians(start.transform.rotation.yaw-heading))<0.:direction=-1
    start=PriorityWaypoint(start,direction)
    goal_lane=(goal.road_id,goal.section_id,goal.lane_id)
    counter=itertools.count();pending=[(0.,next(counter),0.,start)]
    key=priority_key(start);costs={key:0.};parents={};points={key:start}
    deadline=time.monotonic()+PLANNING_TIMEOUT
    while pending:
        if cancelled():raise PlanningCancelled()
        if len(costs)>MAX_SEARCH_NODES or time.monotonic()>deadline:raise ValueError('Highest-priority route search limit reached.')
        _,_,cost,current=heapq.heappop(pending);key=priority_key(current)
        if cost>costs[key]:continue
        location=current.transform.location
        if (current.road_id,current.section_id,current.lane_id)==goal_lane and location.distance(goal.transform.location)<=ROUTE_SPACING*1.5:
            destination=PriorityWaypoint(goal,current.direction)
            route=[destination]
            while key in parents:
                route.append(points[key]);key=parents[key]
            route.append(start)
            return list(reversed(route)),destination
        successors=[(point,0.) for point in current.next(ROUTE_SPACING)]
        if not current.is_junction:
            for adjacent in current.adjacent(world):
                for point in adjacent.next(LANE_CHANGE_DISTANCE):
                    if point.road_id==adjacent.road_id and point.lane_id==adjacent.lane_id and not point.is_junction:
                        successors.append((point,LANE_CHANGE_PENALTY))
        for following,penalty in successors:
            candidate=priority_key(following)
            new_cost=cost+location.distance(following.transform.location)+penalty
            if new_cost>=costs.get(candidate,math.inf):continue
            costs[candidate]=new_cost;parents[candidate]=key;points[candidate]=following
            heapq.heappush(pending,(new_cost+following.transform.location.distance(goal.transform.location),next(counter),new_cost,following))
    raise ValueError('No physically connected driving route reaches the destination.')
