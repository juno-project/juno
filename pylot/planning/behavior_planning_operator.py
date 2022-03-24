import pickle
import time
from collections import deque

import numpy as np

import pylot.planning.cost_functions
import pylot.utils
from pylot.planning.messages import WaypointsMessage
from pylot.planning.utils import BehaviorPlannerState
from pylot.planning.waypoints import Waypoints
from zenoh_flow import Inputs, Operator, Outputs


class BehaviorPlanningState:
    """Behavior planning operator.

    Args:
        pose_stream (:py:class:`erdos.ReadStream`): Stream on which pose
            info is received.
        open_drive_stream (:py:class:`erdos.ReadStream`): Stream on which open
            drive string representations are received. The operator can
            construct HDMaps out of the open drive strings.
        route_stream (:py:class:`erdos.ReadStream`): Stream on which the
            scenario runner publishes waypoints.
        trajectory_stream (:py:class:`erdos.WriteStream`): Stream on which the
            operator sends waypoints the ego vehicle must follow.
        flags (absl.flags): Object to be used to access absl flags.
        goal_location (:py:class:`~pylot.utils.Location`): The goal location of
            the ego vehicle.
    """
    def __init__(self, cfg):

        # pose_stream.add_callback(self.on_pose_update)
        # open_drive_stream.add_callback(self.on_opendrive_map)
        # route_stream.add_callback(self.on_route_msg)
        # erdos.add_watermark_callback(
        #     [pose_stream, open_drive_stream, route_stream],
        #     [trajectory_stream], self.on_watermark)
        # flags.DEFINE_list('goal_location', '234, 59, 39', 'Ego-vehicle goal location')
        print("operator init......................")
        self.cfg = cfg
        goal_location = pylot.utils.Location(cfg['goal_location_x'],
                                             cfg['goal_location_y'],
                                             cfg['goal_location_z'])
        # Do not set the goal location here so that the behavior planner
        # issues an initial message.
        self._goal_location = None
        # Initialize the state of the behaviour planner.
        self.__initialize_behaviour_planner()
        self._pose_msgs = deque()
        self._ego_info = EgoInfo()
        if goal_location:
            self._route = Waypoints(
                deque([
                    pylot.utils.Transform(goal_location,
                                          pylot.utils.Rotation())
                ]))
        else:
            self._route = None
        self._map = None

    def on_opendrive_map(self, msg, timestamp):
        """Invoked whenever a message is received on the open drive stream.

        Args:
            msg (:py:class:`~erdos.message.Message`): Message that contains
                the open drive string.
        """
        print("on_opendrive_map....................")
        print('@{}: received open drive message'.format(timestamp))
        from pylot.simulation.utils import map_from_opendrive
        self._map = map_from_opendrive(msg.data, self.cfg['log_file_name'])

    def on_route_msg(self, msg):
        """Invoked whenever a message is received on the trajectory stream.

        Args:
            msg (:py:class:`~erdos.message.Message`): Message that contains
                a list of waypoints to the goal location.
        """
        print('@{}: global trajectory has {} waypoints'.format(
            msg.timestamp, len(msg.waypoints.waypoints)))
        print('111', msg.waypoints.waypoints)
        self._route = msg.waypoints

    def on_pose_update(self, msg, timestamp):
        """Invoked whenever a message is received on the pose stream.

        Args:
            msg (:py:class:`~erdos.message.Message`): Message that contains
                info about the ego vehicle.
        """
        print("on_pose_update.....................")
        print('@{}: received pose message'.format(timestamp))
        self._pose_msgs.append(msg)


    def __initialize_behaviour_planner(self):
        # State the planner is in.
        self._state = BehaviorPlannerState.FOLLOW_WAYPOINTS
        # Cost functions. Output between 0 and 1.
        self._cost_functions = [
            pylot.planning.cost_functions.cost_overtake,
        ]
        # How important a cost function is.
        self._function_weights = [1]

    def __successor_states(self):
        """ Returns possible state transitions from current state."""
        if self._state == BehaviorPlannerState.FOLLOW_WAYPOINTS:
            # Can transition to OVERTAKE if the ego vehicle has been stuck
            # behind an obstacle for a while.
            return [
                BehaviorPlannerState.FOLLOW_WAYPOINTS,
                BehaviorPlannerState.OVERTAKE
            ]
        elif self._state == BehaviorPlannerState.OVERTAKE:
            return [
                BehaviorPlannerState.OVERTAKE,
                BehaviorPlannerState.FOLLOW_WAYPOINTS
            ]
        elif self._state == BehaviorPlannerState.KEEP_LANE:
            # 1) keep_lane -> prepare_lane_change_left
            # 2) keep_lane -> prepare_lane_change_right
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT
            ]
        elif self._state == BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT:
            # 1) prepare_lane_change_left -> keep_lane
            # 2) prepare_lane_change_left -> lange_change_left
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_LEFT,
                BehaviorPlannerState.LANE_CHANGE_LEFT
            ]
        elif self._state == BehaviorPlannerState.LANE_CHANGE_LEFT:
            # 1) lange_change_left -> keep_lane
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.LANE_CHANGE_LEFT
            ]
        elif self._state == BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT:
            # 1) prepare_lane_change_right -> keep_lane
            # 2) prepare_lane_change_right -> lange_change_right
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.PREPARE_LANE_CHANGE_RIGHT,
                BehaviorPlannerState.LANE_CHANGE_RIGHT
            ]
        elif self._state == BehaviorPlannerState.LANE_CHANGE_RIGHT:
            # 1) lane_change_right -> keep_lane
            return [
                BehaviorPlannerState.KEEP_LANE,
                BehaviorPlannerState.LANE_CHANGE_RIGHT
            ]
        else:
            raise ValueError('Unexpected vehicle state {}'.format(self._state))

    def best_state_transition(self, ego_info):
        """ Computes most likely state transition from current state."""
        # Get possible next state machine states.
        possible_next_states = self.__successor_states()
        best_next_state = None
        min_state_cost = np.infty
        for state in possible_next_states:
            state_cost = 0
            # Compute the cost of the trajectory.
            for i in range(len(self._cost_functions)):
                cost_func = self._cost_functions[i](self._state, state,
                                                    ego_info)
                state_cost += self._function_weights[i] * cost_func
            # Check if it's the best trajectory.
            if state_cost < min_state_cost:
                best_next_state = state
                min_state_cost = state_cost
        return best_next_state

    def get_goal_location(self, ego_transform: pylot.utils.Transform):
        if len(self._route.waypoints) > 1:
            dist = ego_transform.location.distance(
                self._route.waypoints[0].location)
            if dist < 5:
                new_goal_location = self._route.waypoints[1].location
            else:
                new_goal_location = self._route.waypoints[0].location
        elif len(self._route.waypoints) == 1:
            new_goal_location = self._route.waypoints[0].location
            print("get_goal_location  new_goal_location : {}".format(new_goal_location))
        else:
            new_goal_location = ego_transform.location
        return new_goal_location



class BehaviorPlanningOperator(Operator):
    def initialize(self, configuration):
         return BehaviorPlanningState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        # Using input rules

        token = tokens.get('behaviorMsg').get_data()
        msg = pickle.loads(bytes(token))

        open_drive = msg['open_drive']
        vehicle_transform = msg['vehicle_transform']
        timestamp = msg['timestamp']

        state.on_pose_update(vehicle_transform, timestamp)
        # print(open_drive)
        state.on_opendrive_map(open_drive, timestamp)
        return True

    def output_rule(self, _ctx, state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, state, inputs):
        msg = inputs.get("behaviorMsg").data
        # print("@: inputs:  {}".format(msg))
        msg = pickle.loads(msg)

        # # Retrieve the messages for this timestamp
        timestamp = msg['timestamp']

        print('@{}: received watermark'.format(timestamp))

        # return {'behaviorPlanningMsg': pickle.dumps("test")}
        # if timestamp.is_top:
        #     return
        pose_msg = state._pose_msgs.popleft()
        print("pose_msg: {}".format(pose_msg))
        ego_transform = pose_msg

        state._ego_info.update(state._state, pose_msg)
        old_state = state._state
        state._state = state.best_state_transition(state._ego_info)
        print('@{}: agent transitioned from {} to {}'.format(
            timestamp, old_state, state._state))
        # Remove the waypoint from the route if we're close to it.
        if (old_state != state._state
                and state._state == BehaviorPlannerState.OVERTAKE):
            state._route.remove_waypoint_if_close(ego_transform.location, 10)
        else:
            if not state._map.is_intersection(ego_transform.location):
                state._route.remove_waypoint_if_close(ego_transform.location,
                                                     10)
            else:
                state._route.remove_waypoint_if_close(ego_transform.location, 3)

        new_goal_location = state.get_goal_location(ego_transform)
        if new_goal_location != state._goal_location:
            state._goal_location = new_goal_location
            if state._map:
                # Use the map to compute more fine-grained waypoints.
                waypoints = state._map.compute_waypoints(
                    ego_transform.location, state._goal_location)
                road_options = deque([
                    pylot.utils.RoadOption.LANE_FOLLOW
                    for _ in range(len(waypoints))
                ])
                waypoints = Waypoints(waypoints, road_options=road_options)
            else:
                # Map is not available, send the route.
                waypoints = state._route
            if not waypoints or waypoints.is_empty():
                # If waypoints are empty (e.g., reached destination), set
                # waypoints to current vehicle location.
                waypoints = Waypoints(
                    deque([ego_transform]),
                    road_options=deque([pylot.utils.RoadOption.LANE_FOLLOW]))
            return {"behaviorPlanningMsg": pickle.dumps(WaypointsMessage(timestamp, waypoints, state._state))}
        elif old_state != state._state:
            # Send the state update.
            return {"behaviorPlanningMsg": pickle.dumps(WaypointsMessage(timestamp, None, state._state))}

        return {"behaviorPlanningMsg": pickle.dumps(None)}





class EgoInfo(object):
    def __init__(self):
        self.last_time_moving = 0
        self.last_time_stopped = 0
        self.current_time = 0

    def update(self, state, pose_msg):
        # self.current_time = pose_msg.timestamp.coordinates[0]
        state.current_time = time.time()
        # if pose_msg.data.forward_speed >= 0.7:
        if 1 >= 0.7:
            state.last_time_moving = state.current_time
        else:
            state.last_time_stopped = state.current_time


def register():
    return BehaviorPlanningOperator