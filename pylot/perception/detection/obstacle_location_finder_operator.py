import pickle
from collections import deque

import erdos

from pylot.perception.detection.utils import get_obstacle_locations
from pylot.perception.messages import ObstaclesMessage
import tensorflow as tf
from zenoh_flow import Inputs, Operator, Outputs

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
        self.cfg = cfg


class ObstacleLocationFinderOperator(Operator):

    def initialize(self, configuration):
        return ObstacleLocationFinderState(configuration)

    def finalize(self, state):
        return None

    def input_rule(self, _ctx, state, tokens):
        obstacle_token = tokens.get('obstacles_stream_wo_depth')
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
        # print("depth_msg: {}".format(depth_msg))
        obstacles_msg = _state.obstacles_msg
        # print("obstacles_msg: {}".format(obstacles_msg))
        vehicle_transform = _state.point_cloud_msg.point_cloud.transform
        # print("vehicle_transform: {}".format(vehicle_transform))
        camera_setup = _state.obstacles_msg.camera_setup
        # print("obstacle location finder camera_setup: {}".format(camera_setup))
        obstacles_with_location = get_obstacle_locations(
            obstacles_msg.obstacles, depth_msg, vehicle_transform,
            camera_setup)
        # print('@{}, obstacles with location: {}'.format(timestamp,
        #                        obstacles_with_location))
        return {'obstacles_stream': pickle.dumps(ObstaclesMessage(timestamp, obstacles_with_location, camera_setup))}


def register():
    return ObstacleLocationFinderOperator
