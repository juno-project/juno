import copy
import logging
import math
import os
import pickle

import cv2

from lanenet.lanenet_model import lanenet  # noqa: I100 E402
from lanenet.lanenet_model import lanenet_postprocess  # noqa: I100 E402

import numpy as np

import pylot.utils
from pylot.perception.detection.lane import Lane

import tensorflow as tf

from pylot.perception.messages import LanesMessage


class LanenetDetectionState:
    def __init__(self, cfg):
        self.cfg = cfg
        self.camera_stream = None
        self._logger = pylot.utils.get_logger(cfg["log_file_name"])
        tf.compat.v1.disable_eager_execution()
        pylot.utils.set_tf_loglevel(logging.ERROR)
        self._input_tensor = tf.compat.v1.placeholder(dtype=tf.float32,
                                                      shape=[1, 256, 512, 3],
                                                      name='input_tensor')
        net = lanenet.LaneNet(phase='test')
        self._binary_seg_ret, self._instance_seg_ret = net.inference(
            input_tensor=self._input_tensor, name='LaneNet')
        self._gpu_options = tf.compat.v1.GPUOptions(
            allow_growth=True,
            visible_device_list=str(cfg["lane_detection_gpu_index"]),
            per_process_gpu_memory_fraction=cfg["lane_detection_gpu_memory_fraction"] / 10,
            allocator_type='BFC')
        self._tf_session = tf.compat.v1.Session(
            config=tf.compat.v1.ConfigProto(gpu_options=self._gpu_options,
                                            allow_soft_placement=True))
        with tf.compat.v1.variable_scope(name_or_scope='moving_avg'):
            variable_averages = tf.train.ExponentialMovingAverage(0.9995)
            variables_to_restore = variable_averages.variables_to_restore()

        self._postprocessor = lanenet_postprocess.LaneNetPostProcessor()
        saver = tf.compat.v1.train.Saver(variables_to_restore)

        lanenet_detection_model_path = os.path.abspath(os.getenv("PYLOT_HOME") + cfg['lanenet_detection_model_path'])
        print("lanenet_detection_model_path : {}".format(lanenet_detection_model_path))

        with self._tf_session.as_default():
            saver.restore(sess=self._tf_session, save_path=lanenet_detection_model_path)

    def lane_to_ego_coordinates(self, lane, camera_setup):
        """Transforms a lane represented as a pixel locations into 3D locations
        relative to the ego vehicle."""
        inverse_intrinsic_matrix = np.linalg.inv(
            camera_setup.get_intrinsic_matrix())
        camera_ground_height = camera_setup.get_transform().location.z
        pitch = -math.radians(camera_setup.get_transform().rotation.pitch)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        pitch_matrix = np.array([[1, 0, 0], [0, cos_pitch, sin_pitch],
                                 [0, -sin_pitch, cos_pitch]])
        ego_lane = []
        locations = []
        for x, y in lane:
            # Project the 2D pixel location into 3D space, onto the z=1 plane.
            p3d = np.dot(inverse_intrinsic_matrix, np.array([[x], [y], [1.0]]))
            rotate_point = np.dot(pitch_matrix, p3d)
            scale = camera_ground_height / rotate_point[1][0]
            loc = pylot.utils.Location(rotate_point[0][0] * scale,
                                       rotate_point[1][0] * scale,
                                       rotate_point[2][0] * scale)
            locations.append(loc)
        to_world_transform = camera_setup.get_unreal_transform()
        ego_lane = to_world_transform.transform_locations(locations)
        # Reset z = ground.
        for loc in ego_lane:
            loc.z = 0
        return ego_lane


class LanenetDetectionOperator():
    def initialize(self, configuration):
        return LanenetDetectionState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        # Using input rules
        token = tokens.get('center_camera_stream')
        msg = pickle.loads(bytes(token.get_data()))
        if msg['camera_stream'] == None:
            token.set_action_drop()
            return False
        state.camera_stream = msg['camera_stream']

        # camera collection frame  output
        # out_path = "/home/erdos/workspace/zenoh-flow-auto-driving/test_out"
        # os.makedirs(out_path, exist_ok=True)
        # state.camera_stream.frame.save(state.camera_stream.timestamp, out_path,
        #                                'tl-detector-{}'.format('LanenetDetOperator_input'))

        print("LanenetDetectionOperator state.camera_stream : {}".format(state.camera_stream))
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, inputs):
        """Invoked whenever a frame message is received on the stream.

                Args:
                    msg: A :py:class:`~pylot.perception.messages.FrameMessage`.
                    detected_lanes_stream (:py:class:`erdos.WriteStream`): Stream on
                        which the operator sends
                        :py:class:`~pylot.perception.messages.LanesMessage` messages.
                """
        # msg = _state.camera_stream
        msg = copy.deepcopy(_state.camera_stream)
        print('@{}: {} received message'.format(
            msg, 'LanenetDetectionOperator'))

        assert msg.frame.encoding == 'BGR', 'Expects BGR frames'
        image = cv2.resize(msg.frame.as_rgb_numpy_array(), (512, 256),
                           interpolation=cv2.INTER_LINEAR)

        image = image / 127.5 - 1.0
        binary_seg_image, instance_seg_image = _state._tf_session.run(
            [_state._binary_seg_ret, _state._instance_seg_ret],
            feed_dict={_state._input_tensor: [image]})

        postprocess_result = _state._postprocessor.postprocess(
            binary_seg_result=binary_seg_image[0],
            instance_seg_result=instance_seg_image[0],
            source_image=msg.frame.frame)
        # mask_image = postprocess_result['mask_image']
        # for i in range(4):
        #     instance_seg_image[0][:, :, i] = \
        #         minmax_scale(instance_seg_image[0][:, :, i])
        # embedding_image = np.array(instance_seg_image[0], np.uint8)

        lanes = postprocess_result['lanes']
        ego_lane_markings = []
        for lane in lanes:
            ego_markings = _state.lane_to_ego_coordinates(
                lane, msg.frame.camera_setup)
            ego_markings.reverse()
            ego_lane_markings.append([
                pylot.utils.Transform(loc, pylot.utils.Rotation())
                for loc in ego_markings
            ])
        # Sort the lane markings from left to right.
        ego_lane_markings = sorted(ego_lane_markings,
                                   key=lambda lane: lane[0].location.y)
        # Get the index of the ego lane.
        ego_lane_index = None
        all_lanes_to_the_left = True
        all_lanes_to_the_right = True
        for index, lane in enumerate(ego_lane_markings):
            if index > 0 and ego_lane_markings[
                index - 1][0].location.y < 0 and lane[0].location.y > 0:
                ego_lane_index = index
            if ego_lane_markings[index][0].location.y < 0:
                all_lanes_to_the_right = False
            elif ego_lane_markings[index][0].location.y > 0:
                all_lanes_to_the_left = False

        if ego_lane_index is None:
            # No ego lane found.
            if all_lanes_to_the_left:
                # All left lanes have negative ids.
                ego_lane_index = len(ego_lane_markings)
            if all_lanes_to_the_right:
                # All right lanes have positive ids.
                ego_lane_index = 0

        detected_lanes = []
        for index, lane in enumerate(ego_lane_markings):
            if index > 0:
                lane = Lane(index - ego_lane_index,
                            ego_lane_markings[index - 1], lane)
                detected_lanes.append(lane)

        print('@{}: Detected {} lanes'.format(
            msg.timestamp, len(detected_lanes)))

        os.makedirs(_state.cfg['out_path'], exist_ok=True)
        msg.frame.save(msg.timestamp, _state.cfg['out_path'],
                                       'tl-detector-{}'.format('LanenetDetectionOperator'))
        return {'lane_detection_stream': pickle.dumps(LanesMessage(msg.timestamp, detected_lanes))}
        # return {"LanesMessage": pickle.dumps(msg)}
        # plt.figure('binary_image')
        # plt.imshow(binary_seg_image[0] * 255, cmap='gray')
        # plt.figure('instance_image')
        # plt.imshow(embedding_image[:, :, (2, 1, 0)])
        # plt.figure('mask_image')
        # plt.imshow(msg.frame.frame[:, :, (2, 1, 0)])
        # plt.show()


def register():
    return LanenetDetectionOperator
