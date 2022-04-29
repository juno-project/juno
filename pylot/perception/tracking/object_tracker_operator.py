import pickle
import time

from pylot.perception.messages import ObstaclesMessage


class ObjectTrackerState:
    def __init__(self, cfg):
        self.cfg = cfg
        self._last_tracker_run_completion_time = 0
        self.detection_update_count = -1
        self.track_every_nth_detection = 1
        min_matching_iou = float(cfg['min_matching_iou'])
        obstacle_track_max_age = cfg['obstacle_track_max_age']
        da_siam_rpn_model_path = cfg['da_siam_rpn_model_path']
        deep_sort_tracker_weights_path = cfg['deep_sort_tracker_weights_path']
        tracker_type = cfg['tracker_type']
        if tracker_type == 'da_siam_rpn':
            from pylot.perception.tracking.da_siam_rpn_tracker import \
                MultiObjectDaSiamRPNTracker
            self.tracker = MultiObjectDaSiamRPNTracker(da_siam_rpn_model_path, min_matching_iou, obstacle_track_max_age)
        elif tracker_type == 'deep_sort':
            from pylot.perception.tracking.deep_sort_tracker import \
                MultiObjectDeepSORTTracker
            self.tracker = MultiObjectDeepSORTTracker(deep_sort_tracker_weights_path, obstacle_track_max_age, min_matching_iou)
        elif tracker_type == 'sort':
            from pylot.perception.tracking.sort_tracker import \
                MultiObjectSORTTracker
            self.tracker = MultiObjectSORTTracker(obstacle_track_max_age, min_matching_iou)


class ObjectTrackerOperator:
    def initialize(self, configuration):
        return ObjectTrackerState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        obstacle_token = tokens.get('ObstaclesStream')
        camera_token = tokens.get('carlaCameraDriverMsg')
        if camera_token.is_pending():
            camera_token.set_action_keep()
            return False
        if obstacle_token.is_pending():
            obstacle_token.set_action_keep()
            return False

        if not obstacle_token.is_pending() and not camera_token.is_pending():
            obstacles_msg = pickle.loads(bytes(obstacle_token.get_data()))
            camera_msg = pickle.loads(bytes(camera_token.get_data()))

            camera_stream = camera_msg['camera_stream']

            if obstacles_msg is None or camera_stream is None:
                obstacle_token.set_action_drop()
                camera_token.set_action_drop()
                return False
            state.obstacles_msg = obstacles_msg
            state.camera_stream = camera_stream
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):
        frame_msg = _state.camera_stream
        camera_frame = frame_msg.frame
        obstacles_msg = _state.obstacles_msg
        camera_setup = obstacles_msg.camera_setup
        timestamp = frame_msg.timestamp
        detector_runtime = 0
        reinit_runtime = 0
        print("object tracker input obstacles: {}".format(obstacles_msg.obstacles))
        # Check if the most recent obstacle message has this timestamp.
        # If it doesn't, then the detector might have skipped sending
        # an obstacle message.
        if (len(obstacles_msg.obstacles) > 0
                and obstacles_msg.timestamp == timestamp):
            _state.detection_update_count += 1
            if _state.detection_update_count % _state.track_every_nth_detection == 0:
                # Reinitialize the tracker with new detections.
                print(
                    'Restarting trackers at frame {}'.format(timestamp))
                detected_obstacles = []
                for obstacle in obstacles_msg.obstacles:
                    if obstacle.is_vehicle() or obstacle.is_person():
                        detected_obstacles.append(obstacle)
                reinit_runtime, _ = self._reinit_tracker(
                   _state, camera_frame, detected_obstacles)
                detector_runtime = obstacles_msg.runtime
        tracker_runtime, (ok, tracked_obstacles) = \
            self._run_tracker(_state, camera_frame)
        assert ok, 'Tracker failed at timestamp {}'.format(timestamp)
        tracker_runtime = tracker_runtime + reinit_runtime
        tracker_delay = self.__compute_tracker_delay(timestamp,
                                                     detector_runtime,
                                                     tracker_runtime,
                                                     _state)
        print("object tracker output obstacles: {}".format(tracked_obstacles))
        return {'ObstaclesHistoryTrackingStream': pickle.dumps(ObstaclesMessage(timestamp, tracked_obstacles, camera_setup, tracker_delay))}

    @staticmethod
    def __compute_tracker_delay(world_time, detector_runtime,
                                tracker_runtime, _state):
        # If the tracker runtime does not fit within the frame gap, then
        # the tracker will fall behind. We need a scheduler to better
        # handle such situations.
        if (world_time + detector_runtime >
                _state._last_tracker_run_completion_time):
            # The detector finished after the previous tracker invocation
            # completed. Therefore, the tracker is already sequenced.
            tracker_runtime = detector_runtime + tracker_runtime
            _state._last_tracker_run_completion_time = \
                world_time + tracker_runtime
        else:
            # The detector finished before the previous tracker invocation
            # completed. The tracker can only run after the previous
            # invocation completes.
            _state._last_tracker_run_completion_time += tracker_runtime
            tracker_runtime = \
                _state._last_tracker_run_completion_time - world_time
        return tracker_runtime

    @staticmethod
    def _reinit_tracker(state, camera_frame, detected_obstacles):
        start = time.time()
        result = state.tracker.reinitialize(camera_frame, detected_obstacles)
        return (time.time() - start) * 1000, result

    @staticmethod
    def _run_tracker(state, camera_frame):
        start = time.time()
        result = state.tracker.track(camera_frame)
        return (time.time() - start) * 1000, result


def register():
    return ObjectTrackerOperator
