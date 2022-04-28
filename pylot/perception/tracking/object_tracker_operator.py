import time

import pickle
from matplotlib.pyplot import cla

from pylot.perception.messages import ObstaclesMessage


class ObjectTrackerState:
    def __init__(self, cfg):
        self.cfg = cfg
        self._last_tracker_run_completion_time = 0
        self.detection_update_count = -1
        self.track_every_nth_detection = 1
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
        print("object tracker operator received msgs.")
        obstacle_token = tokens.get('ObstaclesWithLocationMsg')
        camera_token = tokens.get('carlaCameraDriverMsg')
        if camera_token.is_pending():
            print("camera pending")
            camera_token.set_action_keep()
            return False
        if obstacle_token.is_pending():
            print("obstacle pending")
            obstacle_token.set_action_drop()
            return False

        if not obstacle_token.is_pending() and not camera_token.is_pending():
            print("neither pending")
            obstacles_msg = pickle.loads(bytes(obstacle_token.get_data()))
            camera_msg = pickle.loads(bytes(camera_token.get_data()))

            camera_stream = camera_msg['camera_stream']

            if obstacles_msg is None or camera_stream is None:
                print("either none")
                obstacle_token.set_action_drop()
                camera_token.set_action_drop()
                return False
            state.obstacles_msg = obstacles_msg
            state.camera_stream = camera_stream
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):
        print("erererere")
        # frame_msg = _state.camera_stream
        # camera_frame = frame_msg.frame
        # obstacles_msg = _state.obstacles_msg
        # timestamp = frame_msg.timestamp
        # print('@{}: received watermark'.format(timestamp))
        #
        # tracked_obstacles = []
        # detector_runtime = 0
        # reinit_runtime = 0
        # # Check if the most recent obstacle message has this timestamp.
        # # If it doesn't, then the detector might have skipped sending
        # # an obstacle message.
        # if (len(obstacles_msg.obstacles) > 0
        #         and obstacles_msg.timestamp == timestamp):
        #     _state.detection_update_count += 1
        #     if _state.detection_update_count % _state.track_every_nth_detection == 0:
        #         # Reinitialize the tracker with new detections.
        #         print(
        #             'Restarting trackers at frame {}'.format(timestamp))
        #         detected_obstacles = []
        #         for obstacle in obstacles_msg.obstacles:
        #             if obstacle.is_vehicle() or obstacle.is_person():
        #                 detected_obstacles.append(obstacle)
        #         reinit_runtime, _ = self._reinit_tracker(
        #             camera_frame, detected_obstacles)
        #         detector_runtime = obstacles_msg.runtime
        # tracker_runtime, (ok, tracked_obstacles) = \
        #     self._run_tracker(camera_frame)
        # assert ok, 'Tracker failed at timestamp {}'.format(timestamp)
        # tracker_runtime = tracker_runtime + reinit_runtime
        # tracker_delay = self.__compute_tracker_delay(timestamp.coordinates[0],
        #                                              detector_runtime,
        #                                              tracker_runtime,
        #                                              _state)
        # return {'ObstaclesMessage': pickle.dumps(ObstaclesMessage(timestamp, tracked_obstacles, None, tracker_delay))}
        return {'ObstaclesMessage': pickle.dumps(None)}

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
