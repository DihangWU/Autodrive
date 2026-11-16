"""对同向慢车主动评估对向借道；跨层、实体隔离及来车冲突时拒绝。"""
import bisect
import logging
import carla
from .borrowing_geometry import blend_borrowing_segment, trajectory_clear
from .priority_lane_access import physically_connected
from vanet.npc.stationary_traffic_obstacles.carla_obstacle_cache import CarlaObstacleCache

LOG = logging.getLogger('vanet.lincoln.borrowing')


class BorrowingPlanner:
    def __init__(self, node, road_map, configuration=None,highest_right_of_way=False):
        self.node, self.road_map = node, road_map
        self.highest_right_of_way=highest_right_of_way
        from vanet.lincoln.emergency_driving_control.control_configuration import DEFAULT_BORROWING
        self.configuration=dict(DEFAULT_BORROWING)
        self.configuration.update(configuration or {})
        self.obstacle_cache = CarlaObstacleCache(node)
        self.last_evaluation = -1.
        self.last_reason = None
        self.predicted_conflict = False

    def report(self, reason, **details):
        if reason != self.last_reason:
            LOG.info('BORROWING reason=%s details=%s', reason, details)
            self.last_reason = reason

    def obstacles(self, snapshot):
        return self.obstacle_cache.collect(snapshot)

    def evaluate(self, controller, snapshot):
        obstacles = self.obstacles(snapshot)
        controller.collision_obstacles=obstacles
        now = snapshot.timestamp.elapsed_seconds
        self.predicted_conflict=False
        if controller.borrowing:
            if controller.progress >= controller.distances[min(controller.borrow_until,len(controller.distances)-1)]:
                self.report('returned_to_original_lane')
                controller.replace_samples(controller.original_samples)
                return True
            begin=max(0,bisect.bisect_left(controller.distances,controller.progress))
            ego=snapshot.find(self.node.car.id)
            speed=ego.get_velocity().length() if ego else 0.
            self.predicted_conflict=not trajectory_clear(
                controller.samples[begin:controller.borrow_until+1],obstacles,max(speed,2.),
                controller.half_width,controller.front)
            if self.predicted_conflict:self.report('new_conflict_while_borrowing_brake')
            # 当前借道轨迹仍持续做物理减速检查，不在车身占对向道时突然切回旧轨迹。
            return False
        if now-self.last_evaluation < self.configuration['evaluation_interval']:
            return False
        self.last_evaluation = now
        state = snapshot.find(self.node.car.id)
        if state is None:
            return False
        speed = state.get_velocity().length()
        begin = bisect.bisect_left(controller.distances,controller.progress+max(6.,controller.front+2.))
        end = min(len(controller.samples)-1,bisect.bisect_left(controller.distances,controller.progress+self.configuration['lookahead_metres']))
        if end-begin < 20:
            return False
        corridor = controller.samples[begin:end+1]
        prediction_speed=max(speed,self.configuration['prediction_speed_mps'])
        if trajectory_clear(corridor,obstacles,prediction_speed,controller.half_width,controller.front):
            self.report('normal_corridor_clear')
            return False
        targets, finish = [], begin-1
        selected_side=None
        for index in range(begin,end+1):
            sample=controller.samples[index]
            waypoint=self.road_map.get_waypoint(carla.Location(*sample.center),project_to_road=True,lane_type=carla.LaneType.Driving)
            if self.highest_right_of_way and waypoint is not None:
                sides=('right','left') if selected_side is None else (selected_side,)
                adjacent=None
                for side in sides:
                    candidate=getattr(waypoint,'get_'+side+'_lane')()
                    if (candidate is not None and candidate.lane_type==carla.LaneType.Driving
                            and candidate.road_id==waypoint.road_id
                            and physically_connected(self.node.world,waypoint,candidate)):
                        adjacent=candidate;selected_side=side;break
            else:adjacent=waypoint.get_left_lane() if waypoint is not None else None
            if (waypoint is None or waypoint.is_junction or adjacent is None or adjacent.is_junction
                    or adjacent.lane_type != carla.LaneType.Driving or adjacent.road_id != waypoint.road_id
                    or (not self.highest_right_of_way and adjacent.lane_id*waypoint.lane_id >= 0)):
                break
            target=adjacent.transform.location
            if abs(target.z-sample.center[2])>.5 or target.distance(waypoint.transform.location)>waypoint.lane_width+1.:
                break
            if waypoint.lane_width < controller.half_width*2.+.5:
                break
            # Unreal 静态场景的围墙/隔离设施不一定是 actor；横向射线检查实体。
            source=carla.Location(*sample.center);source.z+=.5
            destination=carla.Location(target.x,target.y,target.z+.5)
            hits=self.node.world.cast_ray(source,destination)
            if any(str(hit.label).split('.')[-1] not in ('Roads','RoadLines','Ground') for hit in hits):
                self.report('physical_barrier')
                return False
            targets.append((target.x,target.y,target.z));finish=index
        # 预留进入、完整占道和返回的空间，避免把路口当成返回通道。
        if finish<begin or controller.distances[finish]-controller.distances[begin] < self.configuration['minimum_length_metres']:
            self.report('insufficient_opposite_lane_length')
            return False
        candidate=blend_borrowing_segment(controller.samples,targets,begin,finish,transition=self.configuration['transition_metres'])
        if not trajectory_clear(candidate[begin:finish+1],obstacles,prediction_speed,controller.half_width,controller.front):
            self.report('predicted_opposing_or_other_conflict')
            return False
        controller.replace_samples(candidate,True,finish)
        self.report('priority_lane_change_to_pass' if self.highest_right_of_way else 'borrow_to_pass_slower_traffic',start=begin,finish=finish)
        return True
