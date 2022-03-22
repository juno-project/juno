"""This module implements EKF localization using GNSS and IMU."""
import pickle
import time
from collections import deque
import numpy as np
from pylot.localization.messages import PoseMessage
from pylot.utils import Location, Pose, Quaternion, Rotation, Transform, \
    Vector3D


class LocalizationState:
    def  __init__(self, cfg):
        # Register callbacks on read streams.
        self._imu_updates = deque()

        self._gnss_updates = deque()

        self._ground_pose_updates = deque()

        # Initialize a logger.
        self.cfg = cfg

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

class LocalizationOperator():
    def initialize(self, configuration):
        return LocalizationState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        token = tokens.get('localizationMsg').get_data()
        msg = pickle.loads(bytes(token))

        pose_msg = msg['pose']
        gnss_msg = msg['gnss']
        imu_msg = msg['imu']

        state.buffer_msg(pose_msg, msg_type="pose", queue=state._ground_pose_updates)
        state.buffer_msg(gnss_msg, msg_type="GNSS", queue=state._gnss_updates)
        state.buffer_msg(imu_msg, msg_type="IMU", queue=state._imu_updates)

        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, inputs):
        msg = inputs.get("localizationMsg").data
        # print("@: inputs:  {}".format(msg))
        msg = pickle.loads(msg)

        # # Retrieve the messages for this timestamp
        # pose_msg = msg['pose']
        # gnss_msg = msg['gnss']
        # imu_msg = msg['imu']
        timestamp = msg['timestamp']

        print("@{}: received watermark.".format(timestamp))
        # if timestamp.is_top:
        #     self._pose_stream.send(erdos.WatermarkMessage(timestamp))
        #     return
        pose_msg = _state.get_message(_state._ground_pose_updates, msg['pose'].timestamp,"pose")
        gnss_msg = _state.get_message(_state._gnss_updates, msg['gnss'].timestamp, "GNSS")
        imu_msg = _state.get_message(_state._imu_updates, msg['imu'].timestamp, "IMU")

        if _state._last_pose_estimate is None or \
                (abs(imu_msg.acceleration.y) > 100 and not _state._is_started):
            print("@{}: The initial pose estimate is not initialized.".format(timestamp))
            # If this is the first update or values have not stabilized,
            # save the estimates.
            if pose_msg:
                _state._last_pose_estimate = pose_msg.data
                _state._last_timestamp = time.time()
            else:
                raise NotImplementedError("Need pose message to initialize the estimates.")
        else:
            _state._is_started = True

            # Initialize the delta_t
            current_ts = time.time()
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
            print("@{}: Predicted pose: {}".format(
                timestamp, current_pose))

            # Set the estimates for the next iteration.
            _state._last_timestamp = current_ts
            _state._last_pose_estimate = current_pose

        return {'PoseMessage': pickle.dumps(PoseMessage(_state._last_timestamp, _state._last_pose_estimate))}


def register():
    return LocalizationOperator