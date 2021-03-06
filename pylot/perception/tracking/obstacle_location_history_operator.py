from collections import defaultdict, deque

import pickle

import pylot.utils
from pylot.perception.detection.utils import get_obstacle_locations
from pylot.perception.messages import ObstacleTrajectoriesMessage
from pylot.perception.tracking.obstacle_trajectory import ObstacleTrajectory


class ObstacleLocationHistoryState:
    def __init__(self, cfg):
        self.cfg = cfg
        self.dynamic_obstacle_distance_threshold = cfg['dynamic_obstacle_distance_threshold']
        self.obstacle_history = []


class ObstacleLocationHistoryOperator:
    def initialize(self, configuration):
        return ObstacleLocationHistoryState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        obstacle_token = tokens.get('obstacles_wo_history_tracking_stream')
        lidar_token = tokens.get('point_cloud_stream')

        if obstacle_token.is_pending():
            obstacle_token.set_action_keep()
            return False
        if lidar_token.is_pending():
            lidar_token.set_action_keep()
            return False

        if not obstacle_token.is_pending() and not lidar_token.is_pending():
            obstacles_msg = pickle.loads(bytes(obstacle_token.get_data()))
            lidar_msg = pickle.loads(bytes(lidar_token.get_data()))
            if obstacles_msg is None or lidar_msg['lidar_stream'] is None:
                obstacle_token.set_action_drop()
                lidar_token.set_action_drop()
                return False
            state.obstacles_msg = obstacles_msg
            state.point_cloud_msg = lidar_msg['lidar_stream']
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):
        timestamp = _state.obstacles_msg.timestamp
        depth_msg = _state.point_cloud_msg
        obstacles_msg = _state.obstacles_msg
        vehicle_transform = _state.point_cloud_msg.point_cloud.transform
        camera_setup = _state.obstacles_msg.camera_setup
        print("camera setup: {}".format(camera_setup))
        print('@{}: received watermark'.format(timestamp))

        obstacles_with_location = get_obstacle_locations(
            obstacles_msg.obstacles, depth_msg, vehicle_transform,
            camera_setup)

        ids_cur_timestamp = []
        obstacle_trajectories = []
        for obstacle in obstacles_with_location:
            # Ignore obstacles that are far away.
            if (vehicle_transform.location.distance(
                    obstacle.transform.location) >
                    _state.dynamic_obstacle_distance_threshold):
                continue
            ids_cur_timestamp.append(obstacle.id)
            _state.obstacle_history[obstacle.id].append(obstacle)
            # Transform obstacle location from global world coordinates to
            # ego-centric coordinates.
            cur_obstacle_trajectory = []
            for obstacle in _state.obstacle_history[obstacle.id]:
                new_location = \
                    vehicle_transform.inverse_transform_locations(
                        [obstacle.transform.location])[0]
                cur_obstacle_trajectory.append(
                    pylot.utils.Transform(new_location,
                                          pylot.utils.Rotation()))
            # The trajectory is relative to the current location.
            obstacle_trajectories.append(
                ObstacleTrajectory(obstacle, cur_obstacle_trajectory))

        for obstacle in obstacles_with_location:
            obstacle_location = obstacle.transform.location
            x = obstacle_location.x
            y = obstacle_location.y
            z = obstacle_location.z
            print('{},{},obstacle,{},{}'.format(
                pylot.utils.time_epoch_ms(), timestamp.coordinates[0],
                "[{} {}]".format(obstacle.id, obstacle.label),
                "[{:.4f} {:.4f} {:.4f}]".format(x, y, z)))

        return {
            'obstacles_tracking_stream': pickle.dumps(ObstacleTrajectoriesMessage(timestamp, obstacle_trajectories))}


def register():
    return ObstacleLocationHistoryOperator
