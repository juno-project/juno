import pickle
from collections import deque

import erdos

from pylot.perception.detection.utils import get_obstacle_locations
from pylot.perception.messages import ObstaclesMessage
import tensorflow as tf

"""Computes the world location of the obstacle.

The operator uses a point cloud, which may come from a depth frame to
compute the world location of an obstacle. It populates the location
attribute in each obstacle object.

Warning:
    An obstacle will be ignored if the operator cannot find its location.

Args:
    obstacles_stream (:py:class:`erdos.ReadStream`): Stream on which
        detected obstacles are received.
    depth_stream (:py:class:`erdos.ReadStream`): Stream on which
        either point cloud messages or depth frames are received. The
        message type differs dependening on how data-flow operators are
        connected.
    pose_stream (:py:class:`erdos.ReadStream`): Stream on which pose
        info is received.
    obstacles_output_stream (:py:class:`erdos.WriteStream`): Stream on
        which the operator sends detected obstacles with their world
        location set.
    flags (absl.flags): Object to be used to access absl flags.
    camera_setup (:py:class:`~pylot.drivers.sensor_setup.CameraSetup`):
        The setup of the center camera. This setup is used to calculate the
        real-world location of the camera, which in turn is used to convert
        detected obstacles from camera coordinates to real-world
        coordinates.
"""


# def on_obstacles_update(self, msg: erdos.Message):
#     self._logger.debug('@{}: obstacles update'.format(msg.timestamp))
#     self._obstacles_msgs.append(msg)


# def on_depth_update(self, msg: erdos.Message):
#     self._logger.debug('@{}: depth update'.format(msg.timestamp))
#     self._depth_msgs.append(msg)


# def on_pose_update(self, msg: erdos.Message):
#     self._logger.debug('@{}: pose update'.format(msg.timestamp))
#     self._pose_msgs.append(msg)


class ObstacleLocationFinderState:
    def __init__(self, cfg):
        # Only sets memory growth for flagged GPU
        # physical_devices = tf.config.experimental.list_physical_devices('GPU')
        # tf.config.experimental.set_visible_devices(
        #     [physical_devices[cfg['obstacle_detection_gpu_index']]],
        #     'GPU')
        # tf.config.experimental.set_memory_growth(
        #     physical_devices[cfg['obstacle_detection_gpu_index']], False)

        # Load the model from the saved_model format file.
        print("init operator state")
        self.cfg = cfg


class ObstacleLocationFinderOperator:

    def initialize(self, configuration):
        return ObstacleLocationFinderState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def run(self, _ctx, _state, inputs):
        msg = inputs.get("ObstaclesMsg").data
        msg = pickle.loads(msg)
        timestamp = msg[0]
        depth_msg = msg[1]
        obstacles_msg = msg[2]
        vehicle_transform = msg[3]
        camera_setup = msg[4]
        obstacles_with_location = get_obstacle_locations(
            obstacles_msg.obstacles, depth_msg, vehicle_transform,
            camera_setup, None)
        print('@{}: {}'.format(timestamp,
                               obstacles_with_location))
        return {'ObstaclesMsg': pickle.dumps(ObstaclesMessage(timestamp, obstacles_with_location))}


def register():
    return ObstacleLocationFinderOperator
