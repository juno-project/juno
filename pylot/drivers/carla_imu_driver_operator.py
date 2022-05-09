"""This module implements an operator acts like a IMU driver when
using the simulator.

The operator attaches an IMU sensor to the ego vehicle, receives
IMU measurements from the simulator, and sends them on its output stream.
"""
import pickle
import threading

import erdos
import pylot
import pylot.utils
import pylot.drivers.sensor_setup

from pylot.localization.messages import IMUMessage
from pylot.simulation.utils import get_vehicle_handle, get_world, \
    set_simulation_mode
from pylot.utils import Transform, Vector3D

class CarlaIMUDriverState:
    def __init__(self, cfg):
        self._imu_stream = None
        self.msg_timestamp = 0
        # The operator does not pass watermarks by defaults.
        self.cfg = cfg
        # self._logger = erdos.utils.setup_logging(self.config.name,
        #                                          self.config.log_file_name)

        self._logger = pylot.utils.get_logger(cfg["log_file_name"])
        transform = pylot.utils.Transform(location=pylot.utils.Location(),
                                          rotation=pylot.utils.Rotation())
        imu_setup = pylot.drivers.sensor_setup.IMUSetup('imu', transform)
        self._imu_setup = imu_setup
        # The hero vehicle actor object we obtain from the simulator.
        self._vehicle = None
        # The IMU sensor actor object we obtain from the simulator.
        self._imu = None
        # Lock to ensure that the callbacks do not execute simultaneously.
        self._lock = threading.Lock()

    def process_imu(self, imu_msg):
        """Invoked when an IMU measurement is received from the simulator.

        Sends IMU measurements to downstream operators.
        """
        game_time = int(imu_msg.timestamp * 1000)
        # timestamp = erdos.Timestamp(coordinates=[game_time])
        # watermark_msg = erdos.WatermarkMessage(timestamp)
        timestamp = game_time
        watermark_msg = game_time
        self.msg_timestamp = game_time
        # with erdos.profile(self.cfg["name"] + '.process_imu',
        #                    self,
        #                    event_data={'timestamp': str(timestamp)}):
        with self._lock:
            msg = IMUMessage(
                timestamp,
                Transform.from_simulator_transform(imu_msg.transform),
                Vector3D.from_simulator_vector(imu_msg.accelerometer),
                Vector3D.from_simulator_vector(imu_msg.gyroscope),
                imu_msg.compass)
            # self._imu_stream.send(msg)
            self._imu_stream = msg
            # Note: The operator is set not to automatically propagate
            # watermarks received on input streams. Thus, we can issue
            # watermarks only after the simulator callback is invoked.
            # self._imu_stream.send(watermark_msg)



class CarlaIMUDriverOperator():
    """Publishes IMU mesurements (transform, acceleration, gyro and
    compass) from IMU (inertial measurement unit) sensor.

    This operator attaches to a vehicle at the required position with respect
    to the vehicle, registers callback functions to retrieve the IMU
    measurements and publishes it to downstream operators.

    Args:
        ego_vehicle_id_stream (:py:class:`erdos.ReadStream`): Stream on
            which the operator receives the id of the ego vehicle. It uses this
            id to get a simulator handle to the vehicle.
        imu_stream (:py:class:`erdos.WriteStream`): Stream on which the
            operator sends IMU info.
        imu_setup (:py:class:`pylot.drivers.sensor_setup.IMUSetup`):
            Setup of the IMU sensor.
        flags (absl.flags): Object to be used to access absl flags.
    """

    def initialize(self, configuration):
        return CarlaIMUDriverState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        token = tokens.get('carla_stream').get_data()
        msg = pickle.loads(bytes(token))

        state.vehicle_id_msg = msg['vehicle_id_stream']
        # timestamp = msg['timestamp']
        # state.release_data(timestamp)

        # state.vehicle_id_msg = pickle.load(
        #     open("/home/erdos/workspace/zenoh-flow-auto-driving/test_data/CarlaCameraDriverOperator/input/vehicle_id_msg.pkl",
        #          "rb"))
        # print(state.vehicle_id_msg)
        # state.release_data(time.time())

        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, inputs):
        # Read the vehicle id from the vehicle id stream
        vehicle_id = _state.vehicle_id_msg.data
        _state._logger.debug(
            "The IMUDriverOperator received the vehicle id: {}".format(
                vehicle_id))

        # Connect to the world. We connect here instead of in the constructor
        # to ensure we're connected to the latest world.
        _, world = get_world(_state.cfg["simulator_host"],
                             _state.cfg["simulator_port"],
                             _state.cfg["simulator_timeout"])
        set_simulation_mode(world, _state.cfg)

        _state._vehicle = get_vehicle_handle(world, vehicle_id)

        # Install the IMU.
        imu_blueprint = world.get_blueprint_library().find('sensor.other.imu')

        # Set noise attributes.
        imu_blueprint.set_attribute('noise_accel_stddev_x',
                                    str(_state.cfg["accel_noise_stddev_x"]))
        imu_blueprint.set_attribute('noise_accel_stddev_y',
                                    str(_state.cfg["accel_noise_stddev_y"]))
        imu_blueprint.set_attribute('noise_accel_stddev_z',
                                    str(_state.cfg["accel_noise_stddev_z"]))
        imu_blueprint.set_attribute('noise_gyro_stddev_x',
                                    str(_state.cfg["gyro_noise_stddev_x"]))
        imu_blueprint.set_attribute('noise_gyro_stddev_y',
                                    str(_state.cfg["gyro_noise_stddev_y"]))
        imu_blueprint.set_attribute('noise_gyro_stddev_z',
                                    str(_state.cfg["gyro_noise_stddev_z"]))

        if _state.cfg["simulator_imu_frequency"] == -1:
            imu_blueprint.set_attribute('sensor_tick', '0.0')
        else:
            imu_blueprint.set_attribute(
                'sensor_tick', str(1.0 / _state.cfg["simulator_imu_frequency"]))

        transform = _state._imu_setup.get_transform().as_simulator_transform()

        _state._logger.debug("Spawning an IMU: {}".format(_state._imu_setup))

        _state._imu = world.spawn_actor(imu_blueprint,
                                      transform,
                                      attach_to=_state._vehicle)

        # Register the callback on the IMU.
        _state._imu.listen(_state.process_imu)

        result = {"imu_stream": _state._imu_stream,
                  "timestamp": _state.msg_timestamp
                  }
        return {'imu_stream': pickle.dumps(result)}


def register():
    return CarlaIMUDriverOperator


if __name__ == '__main__':
    config = {
        "log_file_name": 'pylot.log',
        "name": "center_camera_operator",
        "simulator_host": 'localhost',
        "simulator_port": 2000,
        "simulator_timeout": 10,
        "simulator_mode": 'synchronous',
        "simulator_fps": 20,
        "accel_noise_stddev_x": 0.0,
        "accel_noise_stddev_y": 0.0,
        "accel_noise_stddev_z": 0.0,
        "gyro_noise_stddev_x": 0.0,
        "gyro_noise_stddev_y": 0.0,
        "gyro_noise_stddev_z": 0.0,
        "simulator_imu_frequency": 10
    }
    operator = CarlaIMUDriverOperator()
    state = operator.initialize(config)
    operator.input_rule(None, state, None)
    operator.run(None, state, None)
