from math import fabs
from os import O_TRUNC
import pickle

from pylot.perception.messages import ObstaclesMessage
from pylot.perception.tracking.obstacle_trajectory import ObstacleTrajectory
from pylot.planning import rrt_star
from pylot.planning.messages import WaypointsMessage
from pylot.planning.utils import BehaviorPlannerState
from pylot.planning.world import World
from pylot.prediction.messages import PredictionMessage
from pylot.prediction.obstacle_prediction import ObstaclePrediction
from pylot.simulation.utils import map_from_opendrive
from pylot.map.hd_map import HDMap
from pylot.simulation.utils import get_map
from zenoh_flow import Inputs, Operator, Outputs

class PlanningState:
    def __init__(self, cfg):
        self.execution_mode = cfg['execution_mode']
        self.planning_type = cfg['planning_type']
        self.target_speed = cfg['target_speed']

        world_params = {
            'tracking_num_steps': cfg['tracking_num_steps'],
            'static_obstacle_distance_threshold': cfg['static_obstacle_distance_threshold'],
            'dynamic_obstacle_distance_threshold': cfg['dynamic_obstacle_distance_threshold'],
            'num_waypoints_ahead': cfg['num_waypoints_ahead'],
            'obstacle_filtering_distance': float(cfg['obstacle_filtering_distance']),
            'obstacle_radius': float(cfg['obstacle_radius']),
            'min_pid_steer_waypoint_distance': cfg['min_pid_steer_waypoint_distance'],
            'traffic_light_min_distance': cfg['traffic_light_min_distance'],
            'traffic_light_max_distance': cfg['traffic_light_max_distance'],
            'traffic_light_max_angle': float(cfg['traffic_light_max_angle']),
            'coast_factor': float(cfg['coast_factor']),
            'stop_for_people': cfg['stop_for_people'],
            'stop_for_vehicles': cfg['stop_for_vehicles'],
            'stop_for_traffic_lights': cfg['stop_for_traffic_lights'],
            'stop_at_uncontrolled_junctions': cfg['stop_at_uncontrolled_junctions'],
            'person_angle_hit_zone': float(cfg['person_angle_hit_zone']),
            'person_distance_hit_zone': cfg['person_distance_hit_zone'],
            'person_angle_emergency_zone': float(cfg['person_angle_emergency_zone']),
            'person_distance_emergency_zone': cfg['person_distance_emergency_zone']
        }

        FOTparameters = {
            'max_speed': cfg['max_speed'],
            'max_accel': cfg['max_accel'],
            'max_curvature': cfg['max_curvature'],
            'max_road_width_l': cfg['max_road_width_l'],
            'max_road_width_r': cfg['max_road_width_r'],
            'd_road_w': cfg['d_road_w'],
            'dt': cfg['dt'],
            'maxt': cfg['maxt'],
            'mint': cfg['mint'],
            'd_t_s': cfg['d_t_s'],
            'n_s_sample': cfg['n_s_sample'],
            'obstacle_clearance_fot': cfg['obstacle_clearance_fot'],
            'kd': cfg['kd'],
            'kv': cfg['kv'],
            'ka': cfg['ka'],
            'kj': cfg['kj'],
            'kt': cfg['kt'],
            'ko': cfg['ko'],
            'klat': cfg['klat'],
            'klon': cfg['klon']
        }

        hybrid_parameters = {
            "step_size": cfg['step_size_hybrid_astar'],
            "max_iterations": cfg['max_iterations_hybrid_astar'],
            "completion_threshold": cfg['completion_threshold'],
            "angle_completion_threshold": cfg['angle_completion_threshold'],
            "rad_step": cfg['rad_step'],
            "rad_upper_range": cfg['rad_upper_range'],
            "rad_lower_range": cfg['rad_lower_range'],
            "obstacle_clearance": cfg['obstacle_clearance_hybrid_astar'],
            "lane_width": cfg['lane_width_hybrid_astar'],
            "radius": cfg['radius'],
            "car_length": cfg['car_length'],
            "car_width": cfg['car_width']
        }

        rrt_star_parameters = {
            'step_size': cfg['step_size'],
            'max_iterations': cfg['max_iterations'],
            'end_dist_threshold': cfg['end_dist_threshold'],
            'obstacle_clearance': cfg['obstacle_clearance'],
            'lane_width': cfg['lane_width']
        }
        
        self._world = World(world_params)
        if self.planning_type == 'waypoint':
            # Use the FOT planner for overtaking.
            from pylot.planning.frenet_optimal_trajectory.fot_planner \
                import FOTPlanner
            self._planner = FOTPlanner(self._world, FOTparameters)
        elif self.planning_type == 'frenet_optimal_trajectory':
            from pylot.planning.frenet_optimal_trajectory.fot_planner \
                import FOTPlanner
            self._planner = FOTPlanner(self._world, FOTparameters)
        elif self.planning_type == 'hybrid_astar':
            from pylot.planning.hybrid_astar.hybrid_astar_planner \
                import HybridAStarPlanner
            self._planner = HybridAStarPlanner(self._world, hybrid_parameters)
        elif self.planning_type == 'rrt_star':
            from pylot.planning.rrt_star.rrt_star_planner import RRTStarPlanner
            self._planner = RRTStarPlanner(self._world, rrt_star_parameters)
        else:
            raise ValueError('Unexpected planning type: {}'.format(
                self.planning_type))
        self._state = BehaviorPlannerState.FOLLOW_WAYPOINTS

        self._map = HDMap(
            get_map(cfg['simulator_host'], cfg['simulator_port'],
                    cfg['simulator_timeout']))


class PlanningOperator(Operator):
    def initialize(self, configuration):
        return PlanningState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        pose_token = tokens.get("pose_stream")
        trajectory_token = tokens.get("trajectory_stream")
        traffic_lights_token = tokens.get("traffic_lights_stream")
        prediction_token = tokens.get("prediction_stream")
        carla_token = tokens.get("carla_stream")

        if pose_token.is_pending():
            pose_token.set_action_keep()
            return False
        if trajectory_token.is_pending():
            trajectory_token.set_action_keep()
            return False
        if traffic_lights_token.is_pending():
            traffic_lights_token.set_action_keep()
            return False
        if prediction_token.is_pending():
            prediction_token.set_action_keep()
            return False
        if carla_token.is_pending():
            carla_token.set_action_keep()
            return False

        if not pose_token.is_pending() and not trajectory_token.is_pending() and not traffic_lights_token.is_pending() and \
                not carla_token.is_pending() and not prediction_token.is_pending():
            pose_msg = pickle.loads(bytes(pose_token.get_data()))
            trajectory_msg = pickle.loads(bytes(trajectory_token.get_data()))
            traffic_lights_msg = pickle.loads(bytes(traffic_lights_token.get_data()))
            carla_msg = pickle.loads(bytes(carla_token.get_data()))
            prediction_msg = pickle.loads(bytes(prediction_token.get_data()))
            if pose_msg is None or trajectory_msg is None or traffic_lights_msg is None or carla_msg is None or prediction_msg is None:
                pose_token.set_action_drop()
                trajectory_token.set_action_drop()
                traffic_lights_token.set_action_drop()
                carla_token.set_action_drop()
                prediction_token.set_action_drop()
                return False
            state.pose_msg = pose_msg
            state.linear_prediction_msg = prediction_msg
            state.trajectory_msg = trajectory_msg
            state.traffic_light_msg = traffic_lights_msg
            state.open_drive_msg = carla_msg["open_drive_stream"]
            state.timestamp = carla_msg["timestamp"]

        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):

        timestamp = _state.timestamp
        lane_msg = None
        linear_prediction_msg = _state.linear_prediction_msg
        open_drive_msg = _state.open_drive_msg
        trajectory_msg = _state.trajectory_msg
        vehicle_transform = _state.pose_msg.post.transform

        traffic_light_msg = TrafficLightsMessage(timestamp, [])
        print('@{}: received watermark'.format(timestamp))

        if trajectory_msg.agent_state:
            print('@{}: updating planner state to {}'.format(
                timestamp, trajectory_msg.agent_state))
            _state._state = trajectory_msg.agent_state
        if trajectory_msg.waypoints:
            print('@{}: route has {} waypoints'.format(
                timestamp, len(trajectory_msg.waypoints.waypoints)))
            # The last waypoint is the goal location.
            _state._world.update_waypoints(trajectory_msg.waypoints.waypoints[-1].location,
                                           trajectory_msg.waypoints)

        _state._map = map_from_opendrive(open_drive_msg.data)

        self.update_world(self, timestamp, vehicle_transform,
                          linear_prediction_msg, lane_msg, traffic_light_msg, _state._world, _state._map)
        # Total ttd - time spent up to now
        ttd = 0
        # if self._state == BehaviorPlannerState.OVERTAKE:
        #     # Ignore traffic lights and obstacle.
        #     output_wps = self._planner.run(timestamp, ttd)
        # else:
        (speed_factor, _, _, speed_factor_tl,
         speed_factor_stop) = _state._world.stop_for_agents(timestamp)
        if _state.planning_type == 'waypoint':
            target_speed = speed_factor * _state.target_speed
            print(
                '@{}: speed factor: {}, target speed: {}'.format(
                    timestamp, speed_factor, target_speed))
            output_wps = _state._world.follow_waypoints(target_speed)
        else:
            output_wps = _state._planner.run(timestamp, ttd)
            speed_factor = min(speed_factor_stop, speed_factor_tl)
            print('@{}: speed factor: {}'.format(
                timestamp, speed_factor))
            output_wps.apply_speed_factor(speed_factor)
        return {'waypoints_stream': pickle.dumps(WaypointsMessage(timestamp, output_wps))}

    def get_predictions(self, prediction_msg, ego_transform):
        """Extracts obstacle predictions out of the message.

        This method is useful to build obstacle predictions when
        the operator directly receives detections instead of predictions.
        The method assumes that the obstacles are static.
        """
        predictions = []
        if isinstance(prediction_msg, ObstaclesMessage):
            # Transform the obstacle into a prediction.
            print(
                'Planner received obstacles instead of predictions.')
            predictions = []
            for obstacle in prediction_msg.obstacles:
                obstacle_trajectory = ObstacleTrajectory(obstacle, [])
                prediction = ObstaclePrediction(
                    obstacle_trajectory, obstacle.transform, 1.0,
                    [ego_transform.inverse_transform() * obstacle.transform])
                predictions.append(prediction)
        elif isinstance(prediction_msg, PredictionMessage):
            predictions = prediction_msg.predictions
        else:
            raise ValueError('Unexpected obstacles msg type {}'.format(
                type(prediction_msg)))
        return predictions

    def update_world(self, timestamp, vehicle_transform, linear_prediction_msg, lane_msg, traffic_light_msg, world, map):
        ego_transform = vehicle_transform
        prediction_msg = linear_prediction_msg
        predictions = self.get_predictions(self, prediction_msg, ego_transform)
        static_obstacles_msg = traffic_light_msg

        # if lane_msg.data != None:
        #     lanes = lane_msg.data
        # else:
        #     lanes = None
        lanes = None

        # Update the representation of the world.
        world.update(timestamp,
                           ego_transform,
                           predictions,
                           static_obstacles_msg.obstacles,
                           hd_map=map,
                           lanes=lanes)


class TrafficLightsMessage():
    def __init__(self, timestamp, traffic_lights):
        self.obstacles = traffic_lights
        self.timestamp = timestamp

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return 'TrafficLightsMessage(timestamp: {}, ' \
            'traffic lights: {})'.format(
                self.timestamp, self.obstacles)


def register():
    return PlanningOperator
