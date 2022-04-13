"""Implements an operator that detects traffic lights."""
import logging
import os

import numpy as np
import pickle
from pylot.perception.detection.traffic_light import TrafficLight, \
    TrafficLightColor
from pylot.perception.detection.utils import BoundingBox2D
from pylot.perception.messages import TrafficLightsMessage
from zenoh_flow import Inputs, Operator, Outputs

import tensorflow as tf

class TrafficLightDetState:
    def __init__(self, cfg):
        # Only sets memory growth for flagged GPU
        # physical_devices = tf.config.experimental.list_physical_devices('GPU')
        # tf.config.experimental.set_visible_devices(
        #     [physical_devices[cfg['traffic_light_det_gpu_index']], 'GPU')
        # tf.config.experimental.set_memory_growth(
        #     physical_devices[cfg['traffic_light_det_gpu_index']], True)

        # Load the model from the saved_model format file.
        self.cfg = cfg
        model_path = os.path.abspath(os.getenv("PYLOT_HOME") + cfg['model_path'])
        print("model_path : {}".format(model_path))
        self._model = tf.saved_model.load(model_path)

            #self._flags.traffic_light_det_model_path)

        self._labels = {
            1: TrafficLightColor.GREEN,
            2: TrafficLightColor.YELLOW,
            3: TrafficLightColor.RED,
            4: TrafficLightColor.OFF
        }
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
        res_labels = [
            self._labels[int(label)] for label in classes[0][:num_detections]
        ]
        res_boxes = boxes[0][:num_detections]
        res_scores = scores[0][:num_detections]
        return res_boxes, res_scores, res_labels

    def convert_to_detected_tl(self, boxes, scores, labels, height, width):
        traffic_lights = []

        for index in range(len(scores)):
            if scores[
                    index] > self.cfg['traffic_light_det_min_score_threshold']/10:
                bbox = BoundingBox2D(
                    int(boxes[index][1] * width),  # x_min
                    int(boxes[index][3] * width),  # x_max
                    int(boxes[index][0] * height),  # y_min
                    int(boxes[index][2] * height)  # y_max
                )
                traffic_lights.append(
                    TrafficLight(scores[index],
                                 labels[index],
                                 id=self._unique_id,
                                 bounding_box=bbox))
                self._unique_id += 1
        return traffic_lights

class TrafficLightDetOperator(Operator):
    """Detects traffic lights using a TensorFlow model.

    The operator receives frames on a camera stream, and runs a model for each
    frame."""
    def initialize(self, configuration):
        return TrafficLightDetState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        # token = tokens.get('carlaCameraDriverMsg').get_data()
        token = tokens.get('carlaCameraDriverMsg').get_data()
        msg = pickle.loads(bytes(token))
        state.camera_stream = msg['camera_stream']
        print("TrafficLightDetOperator state.camera_stream : {}".format(state.camera_stream))
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, inputs):
        # msg = inputs.get("FrameMsg").data
        if _state.camera_stream == None:
            return {"TrafficLightsMsg": pickle.dumps(None)}

        msg = _state.camera_stream

        # print('@{}: {} received message'.format(
        #     msg.timestamp, 'TrafficLightDetOperator'))

        print('@{}: {} received message : {}'.format(
            msg.timestamp, 'TrafficLightDetOperator', msg))

        assert msg.frame.encoding == 'BGR', 'Expects BGR frames'
        boxes, scores, labels = _state.run_model(msg.frame.as_rgb_numpy_array())

        traffic_lights = _state.convert_to_detected_tl(
            boxes, scores, labels, msg.frame.camera_setup.height, msg.frame.camera_setup.width)

        print('@{}: {} detected traffic lights {}'.format(
            msg.timestamp, 'TrafficLightDetOperator', traffic_lights))

        if _state.cfg['log_traffic_light_detector_output'] and traffic_lights:
            msg.frame.annotate_with_bounding_boxes(msg.timestamp,
                                                   traffic_lights)
            os.makedirs(_state.cfg['out_path'], exist_ok=True)
            msg.frame.save(msg.timestamp, _state.cfg['out_path'],
                           'tl-detector-{}'.format('TrafficLightDetOperator'))

        return {"TrafficLightsMsg": pickle.dumps(TrafficLightsMessage(msg.timestamp, traffic_lights))}

def register():
    return TrafficLightDetOperator