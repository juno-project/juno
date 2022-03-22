"""Implements an operator that detects obstacles."""
import os
import pickle
import time

import numpy as np
import tensorflow as tf

from pylot.perception.detection.obstacle import Obstacle
from pylot.perception.detection.utils import BoundingBox2D, \
    OBSTACLE_LABELS, load_coco_bbox_colors, load_coco_labels
from pylot.perception.messages import ObstaclesMessage


class DetectionState:
    def __init__(self, cfg):
        # Only sets memory growth for flagged GPU
        # physical_devices = tf.config.experimental.list_physical_devices('GPU')
        # tf.config.experimental.set_visible_devices(
        #     [physical_devices[cfg['obstacle_detection_gpu_index']]],
        #     'GPU')
        # tf.config.experimental.set_memory_growth(
        #     physical_devices[cfg['obstacle_detection_gpu_index']], False)

        # Load the model from the saved_model format file.

        self.cfg = cfg
        self._model = tf.saved_model.load(cfg['model_path'])

        self._coco_labels = load_coco_labels(cfg['path_coco_labels'])
        self._bbox_colors = load_coco_bbox_colors(self._coco_labels)
        # Unique bounding box id. Incremented for each bounding box.
        self._unique_id = 0

    def run_model(self, image_np):
        # Expand dimensions since the model expects images to have
        # shape: [1, None, None, 3]
        image_np_expanded = np.expand_dims(image_np, axis=0)

        infer = self._model.signatures['serving_default']
        result = infer(tf.convert_to_tensor(value=image_np_expanded))

        boxes = result['boxes']
        scores = result['scores']
        classes = result['classes']
        num_detections = result['detections']

        num_detections = int(num_detections[0])
        res_classes = [int(cls) for cls in classes[0][:num_detections]]
        res_boxes = boxes[0][:num_detections]
        res_scores = scores[0][:num_detections]
        return num_detections, res_boxes, res_scores, res_classes


class DetectionOperator():
    """Detects obstacles using a TensorFlow model.

    The operator receives frames on a camera stream, and runs a model for each
    frame.

    """

    def initialize(self, configuration):
        return DetectionState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, inputs):
        msg = inputs.get("FrameMsg").data
        msg = pickle.loads(msg)
        print('@{}: {} received message'.format(
            msg.timestamp, 'DetectionOperator'))
        start_time = time.time()
        # The models expect BGR images.
        assert msg.frame.encoding == 'BGR', 'Expects BGR frames'
        num_detections, res_boxes, res_scores, res_classes = _state.run_model(
            msg.frame.frame)
        obstacles = []
        for i in range(0, num_detections):
            if res_classes[i] in _state._coco_labels:
                if (res_scores[i] >=
                        _state.cfg['obstacle_detection_min_score_threshold'] / 10):
                    if (_state._coco_labels[res_classes[i]] in OBSTACLE_LABELS):
                        obstacles.append(
                            Obstacle(BoundingBox2D(
                                int(res_boxes[i][1] *
                                    msg.frame.camera_setup.width),
                                int(res_boxes[i][3] *
                                    msg.frame.camera_setup.width),
                                int(res_boxes[i][0] *
                                    msg.frame.camera_setup.height),
                                int(res_boxes[i][2] *
                                    msg.frame.camera_setup.height)),
                                res_scores[i],
                                _state._coco_labels[res_classes[i]],
                                id=_state._unique_id))
                        _state._unique_id += 1
                    else:
                        print('Ignoring non essential detection {}'.format(
                            _state._coco_labels[res_classes[i]]))
            else:
                print('Filtering unknown class: {}'.format(
                    res_classes[i]))

        print('@{}: {} obstacles: {}'.format(
            msg.timestamp, 'DetectionOperator', obstacles))

        # Get runtime in ms.
        runtime = (time.time() - start_time) * 1000
        # Send out obstacles.

        if _state.cfg['log_detector_output'] and obstacles:
            msg.frame.annotate_with_bounding_boxes(msg.timestamp, obstacles,
                                                   None, _state._bbox_colors)
            os.makedirs(_state.cfg['out_path'], exist_ok=True)
            msg.frame.save(msg.timestamp, _state.cfg['out_path'],
                           'detector-{}'.format('DetectionOperator'))

        return {'ObstaclesMsg': pickle.dumps(ObstaclesMessage(msg.timestamp, obstacles, runtime))}


def register():
    return DetectionOperator
