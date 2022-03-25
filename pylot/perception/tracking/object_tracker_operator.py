import time

import pickle
from matplotlib.pyplot import cla

from pylot.perception.messages import ObstaclesMessage


class ObjectTrackerState:
    def __init__(self, cfg):
        self.cfg = cfg
        self._last_tracker_run_completion_time = 0
        self._detection_update_count = -1
        self.tracker_type = cfg['tracker_type']
        self.min_matching_iou = float(cfg['min_matching_iou'])
        self.obstacle_track_max_age = cfg['obstacle_track_max_age']
        self.da_siam_rpn_model_path = cfg['da_siam_rpn_model_path']
        self.deep_sort_tracker_weights_path = cfg['deep_sort_tracker_weights_path']


class ObjectTrackerOperator:
    def initialize(self, configuration):
        return ObjectTrackerState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):
        print("operator starts running")
        msg = inputs.get("ObjectMsg").data
        msg = pickle.loads(msg)
        timestamp = msg[0]
        print('@{}: received watermark'.format(timestamp))
        frame_msg = msg[2]
        camera_frame = frame_msg.frame
        tracked_obstacles = []
        detector_runtime = 0
        reinit_runtime = 0
        # Check if the most recent obstacle message has this timestamp.
        # If it doesn't, then the detector might have skipped sending
        # an obstacle message.
        if (len(self._obstacles_msgs) > 0
                and self._obstacles_msgs[0].timestamp == timestamp):
            obstacles_msg = msg[1]
            _state._detection_update_count += 1
            if (_state._detection_update_count %
                    self._flags.track_every_nth_detection == 0):
                # Reinitialize the tracker with new detections.
                print(
                    'Restarting trackers at frame {}'.format(timestamp))
                detected_obstacles = []
                for obstacle in obstacles_msg.obstacles:
                    if obstacle.is_vehicle() or obstacle.is_person():
                        detected_obstacles.append(obstacle)
                reinit_runtime, _ = self._reinit_tracker(
                    camera_frame, detected_obstacles)
                detector_runtime = obstacles_msg.runtime
        tracker_runtime, (ok, tracked_obstacles) = \
            self._run_tracker(camera_frame)
        assert ok, 'Tracker failed at timestamp {}'.format(timestamp)
        tracker_runtime = tracker_runtime + reinit_runtime
        tracker_delay = self.__compute_tracker_delay(timestamp.coordinates[0],
                                                     detector_runtime,
                                                     tracker_runtime,
                                                     _state)
        return {'ObstaclesMessage': pickle.dumps(ObstaclesMessage(timestamp, tracked_obstacles, tracker_delay))}

    def __compute_tracker_delay(self, world_time, detector_runtime,
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

    def _reinit_tracker(self, camera_frame, detected_obstacles):
        start = time.time()
        result = self._tracker.reinitialize(camera_frame, detected_obstacles)
        return (time.time() - start) * 1000, result

    def _run_tracker(self, camera_frame):
        start = time.time()
        result = self._tracker.track(camera_frame)
        return (time.time() - start) * 1000, result


def register():
    return ObjectTrackerOperator
