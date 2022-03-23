from collections import defaultdict, deque

import pickle

import pylot.utils
from pylot.perception.detection.utils import get_obstacle_locations
from pylot.perception.messages import ObstacleTrajectoriesMessage
from pylot.perception.tracking.obstacle_trajectory import ObstacleTrajectory

class ObstacleLocationHistoryState:
    def __init__(self, cfg):
        self.cfg = cfg

class ObstacleLocationHistoryOperator:
    def initialize(self, configuration):
         return ObstacleLocationHistoryState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):
        print("operator starts running")
        msg = inputs.get("ObstaclesMsg").data
        msg = pickle.loads(msg)
        timestamp = msg[0]
        depth_msg = msg[1]
        obstacles_msg = msg[2]
        vehicle_transform = msg[3]
        camera_setup = msg[4]

        print('@{}: received watermark'.format(timestamp))

        obstacles_with_location = get_obstacle_locations(
            obstacles_msg.obstacles, depth_msg, vehicle_transform,
            camera_setup, None)

        ids_cur_timestamp = []
        obstacle_trajectories = []
        for obstacle in obstacles_with_location:
            # Ignore obstacles that are far away.
            if (vehicle_transform.location.distance(
                    obstacle.transform.location) >
                    self._flags.dynamic_obstacle_distance_threshold):
                continue
            ids_cur_timestamp.append(obstacle.id)
            self._obstacle_history[obstacle.id].append(obstacle)
            # Transform obstacle location from global world coordinates to
            # ego-centric coordinates.
            cur_obstacle_trajectory = []
            for obstacle in self._obstacle_history[obstacle.id]:
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
        

        return {'ObstacleTrajectoriesMessage': pickle.dumps(ObstacleTrajectoriesMessage(timestamp, obstacle_trajectories))}
        

        # self._timestamp_history.append(timestamp)
        # self._timestamp_to_id[timestamp] = ids_cur_timestamp
        # if len(self._timestamp_history) >= self._flags.tracking_num_steps:
        #     gc_timestamp = self._timestamp_history.popleft()
        #     for obstacle_id in self._timestamp_to_id[gc_timestamp]:
        #         self._obstacle_history[obstacle_id].popleft()
        #         if len(self._obstacle_history[obstacle_id]) == 0:
        #             del self._obstacle_history[obstacle_id]
        #     del self._timestamp_to_id[gc_timestamp]

def register():
    return ObstacleLocationHistoryOperator