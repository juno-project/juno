"""This module implements EKF localization using GNSS and IMU."""
import pickle
from collections import deque
import numpy as np

import pylot
from pylot.localization.messages import PoseMessage
from pylot.utils import Location, Pose, Quaternion, Rotation, Transform, \
    Vector3D
from zenoh_flow import Inputs, Operator, Outputs

class LocalizationState:
    def  __init__(self, cfg):
        # Register callbacks on read streams.
        self._imu_updates = deque()

        self._gnss_updates = deque()

        self._ground_pose_updates = deque()

        # Initialize a logger.
        self.cfg = cfg
        self._logger = pylot.utils.get_logger(cfg["log_file_name"])

        # Gravity vector.
        self._g = np.array([0, 0, -9.81])

        # Previous timestamp values.
        self._last_pose_estimate = None
        self._last_timestamp = None

        # NOTE: At the start of the simulation, the vehicle drops down from
        # the sky, during which the IMU values screw up the calculations.
        # This boolean flag takes care to start the prediction only when the
        # values have stabilized.
        self._is_started = False

        # Constants required for the Kalman filtering.
        var_imu_f, var_imu_w, var_gnss = 0.5, 0.5, 0.1
        self.__Q = np.identity(6)
        self.__Q[0:3, 0:3] = self.__Q[0:3, 0:3] * var_imu_f
        self.__Q[3:6, 3:6] = self.__Q[3:6, 3:6] * var_imu_w

        self.__F = np.identity(9)

        self.__L = np.zeros([9, 6])
        self.__L[3:9, :] = np.identity(6)

        self.__R_GNSS = np.identity(3) * var_gnss

        self._last_covariance = np.zeros((9, 9))

    def update_using_gnss(self, location_estimate, velocity_estimate,
                            rotation_estimate, gnss_reading, delta_t):
        # Construct H_k = [I, 0, 0] (shape=(3, 9))
        H_k = np.zeros((3, 9))
        H_k[:, :3] = np.identity(3)

        # Propogate uncertainty.
        Q = self.__Q * delta_t * delta_t
        self._last_covariance = (self.__F.dot(self._last_covariance).dot(
            self.__F.T)) + (self.__L.dot(Q).dot(self.__L.T))

        # Compute Kalman gain. (shape=(9, 3))
        K_k = self._last_covariance.dot(
            H_k.T.dot(
                np.linalg.inv(
                    H_k.dot(self._last_covariance.dot(H_k.T)) +
                    self.__R_GNSS)))

        # Compute error state. (9x3) x ((3x1) - (3x1)) = shape(9, 1)
        delta_x_k = K_k.dot(gnss_reading - location_estimate)

        # Correct predicted state.
        corrected_location_estimate = location_estimate + delta_x_k[0:3]
        corrected_velocity_estimate = velocity_estimate + delta_x_k[3:6]
        roll, pitch, yaw = delta_x_k[6:]
        corrected_rotation_estimate = Quaternion.from_rotation(
            Rotation(roll=roll, pitch=pitch, yaw=yaw)) * rotation_estimate

        # Fix the covariance.
        self._last_covariance = (np.identity(9) - K_k.dot(H_k)).dot(
            self._last_covariance)

        return (
            corrected_location_estimate,
            corrected_velocity_estimate,
            corrected_rotation_estimate,
        )

    def buffer_msg(self, msg, msg_type: str, queue: deque):
        """Callback which buffers the received message."""
        print("@{}: received {} message.".format(
            msg.timestamp, msg_type))
        queue.append(msg)

    def get_message(self, queue: deque, timestamp, name: str):
        msg = None
        if queue:
            while len(queue) > 0:
                retrieved_msg = queue.popleft()
                if retrieved_msg.timestamp == timestamp:
                    msg = retrieved_msg
                    break
            if not msg:
                raise ValueError(
                    "The message for {} with timestamp {} was not found".
                    format(name, timestamp))
        return msg

    def __skew_symmetric(self, v):
        """Skew symmetric form of a 3x1 vector."""
        return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]],
                        dtype=np.float64)

class LocalizationOperator(Operator):
    def initialize(self, configuration):
        return LocalizationState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        imu_token = tokens.get("imu_stream")
        gnss_token = tokens.get("gnss_stream")
        carla_token = tokens.get("carla_stream")

        # print("-------------------------------------------------------------")
        # print(" imu_token.is_pending() : {}".format(imu_token.is_pending()))
        # print(" gnss_token.is_pending() : {}".format(gnss_token.is_pending()))
        # print(" carla_token.is_pending() : {}".format(carla_token.is_pending()))
        # print("-------------------------------------------------------------")
        if imu_token.is_pending():
            imu_token.set_action_keep()
            return False

        if gnss_token.is_pending():
            gnss_token.set_action_keep()
            return False

        if carla_token.is_pending():
            carla_token.set_action_keep()
            return False

        if not imu_token.is_pending() and not gnss_token.is_pending() and not gnss_token.is_pending():
            imu_msg = pickle.loads(bytes(imu_token.get_data()))
            gnss_msg = pickle.loads(bytes(gnss_token.get_data()))
            carla_msg = pickle.loads(bytes(carla_token.get_data()))

            pose_stream = carla_msg['pose_stream']
            gnss_stream = gnss_msg['gnss_stream']
            imu_stream = imu_msg['imu_stream']

            if pose_stream == None or gnss_stream == None or imu_stream == None:
                imu_token.set_action_drop()
                gnss_token.set_action_drop()
                carla_token.set_action_drop()
                return False

            state.timestamp = carla_msg['timestamp']
            state.pose_stream = pose_stream
            state.gnss_stream = gnss_stream
            state.imu_stream = imu_stream

        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, inputs):
        # # Retrieve the messages for this timestamp
        timestamp = _state.timestamp
        print("@{}: received watermark.".format(timestamp))
        pose_msg = _state.pose_stream
        gnss_msg = _state.gnss_stream
        imu_msg = _state.imu_stream

        if _state._last_pose_estimate is None or \
                (abs(imu_msg.acceleration.y) > 100 and not _state._is_started):
            print("@{}: The initial pose estimate is not initialized.".format(timestamp))
            # If this is the first update or values have not stabilized,
            # save the estimates.
            if pose_msg:
                _state._last_pose_estimate = pose_msg.data
                _state._last_timestamp = _state.timestamp
            else:
                raise NotImplementedError("Need pose message to initialize the estimates.")
        else:
            _state._is_started = True

            # Initialize the delta_t
            current_ts = _state.timestamp
            delta_t = (current_ts - _state._last_timestamp) / 1000.0

            # Estimate the rotation.
            last_rotation_estimate = Quaternion.from_rotation(
                _state._last_pose_estimate.transform.rotation)
            rotation_estimate = (
                    last_rotation_estimate *
                    Quaternion.from_angular_velocity(imu_msg.gyro, delta_t))

            # Transform the IMU accelerometer data from the body frame to the
            # world frame, and retrieve location and velocity estimates.
            accelerometer_data = last_rotation_estimate.matrix.dot(
                imu_msg.acceleration.as_numpy_array()) + _state._g
            last_location_estimate = \
                _state._last_pose_estimate.transform.location.as_numpy_array()
            last_velocity_estimate = \
                _state._last_pose_estimate.velocity_vector.as_numpy_array()

            # Estimate the location.
            location_estimate = last_location_estimate + (
                    delta_t * last_velocity_estimate) + ((
                                                                 (delta_t ** 2) / 2.0) * accelerometer_data)

            # Estimate the velocity.
            velocity_estimate = last_velocity_estimate + (delta_t *
                                                          accelerometer_data)

            # Fuse the GNSS values using an EKF to fix drifts and noise in
            # the estimates.

            # Linearize the motion model and compute Jacobians.
            _state._LocalizationState__F[0:3, 3:6] = np.identity(3) * delta_t
            _state._LocalizationState__F[3:6, 6:9] = last_rotation_estimate.matrix.dot(
                -_state._LocalizationState__skew_symmetric(accelerometer_data.reshape(
                    (3, 1)))) * delta_t

            # Fix estimates using GNSS

            print("gnss_stream----- latitude:{},longitude:{}, altitude:{}".format(gnss_msg.latitude, gnss_msg.longitude, gnss_msg.altitude))
            gnss_reading = Location.from_gps(
                gnss_msg.latitude, gnss_msg.longitude,
                gnss_msg.altitude).as_numpy_array()
            (
                location_estimate,
                velocity_estimate,
                rotation_estimate,
            ) = _state.update_using_gnss(location_estimate, velocity_estimate,
                                         rotation_estimate, gnss_reading,
                                         delta_t)

            # Create the PoseMessage and send it downstream.
            current_pose = Pose(
                transform=Transform(location=Location(*location_estimate),
                                    rotation=rotation_estimate.as_rotation()),
                forward_speed=Vector3D(*velocity_estimate).magnitude(),
                velocity_vector=Vector3D(*velocity_estimate),
                localization_time=current_ts,
            )
            _state._logger.info("@{}: Predicted pose: {}".format(
                timestamp, current_pose))

            # Set the estimates for the next iteration.
            _state._last_timestamp = current_ts
            _state._last_pose_estimate = current_pose

        return {'pose_stream': pickle.dumps(PoseMessage(_state._last_timestamp, _state._last_pose_estimate))}


def register():
    return LocalizationOperator