"""This module implements an operator acts like a GNSS driver when
using the simulator.

The operator attaches a GNSS sensor to the ego vehicle, receives GNSS
 measurements from the simulator, and sends them on its output stream.
"""
import pickle
import threading

import erdos
import pylot

from pylot.localization.messages import GNSSMessage
from pylot.simulation.utils import get_vehicle_handle, get_world, \
    set_simulation_mode
from pylot.utils import Transform
import pylot.drivers.sensor_setup

class CarlaGNSSDriverState:
    def __init__(self, cfg):
        self._gnss_stream = None
        self.msg_timestamp = 0

        # Save the flags and initialize logging.
        self.cfg = cfg
        # self._logger = erdos.utils.setup_logging(self.config.name,
        #                                          self.config.log_file_name)
        self._logger = pylot.utils.get_logger(cfg["log_file_name"])

        # Save the setup, the vehicle and the sensor.
        transform = pylot.utils.Transform(location=pylot.utils.Location(),
                              rotation=pylot.utils.Rotation())
        gnss_setup = pylot.drivers.sensor_setup.GNSSSetup('gnss', transform)

        self._gnss_setup = gnss_setup
        self._vehicle = None
        self._gnss = None
        self._lock = threading.Lock()

    def process_gnss(self, gnss_msg):
        """Invoked when a GNSS measurement is received from the simulator.

        Sends GNSS measurements to downstream operators.
        """
        game_time = int(gnss_msg.timestamp * 1000)
        timestamp = game_time
        watermark_msg = game_time
        self.msg_timestamp = game_time
        # with erdos.profile(self.config.name + '.process_gnss',
        #                    self,
        #                    event_data={'timestamp': str(timestamp)}):
        with self._lock:
            msg = GNSSMessage(
                timestamp,
                Transform.from_simulator_transform(gnss_msg.transform),
                gnss_msg.altitude, gnss_msg.latitude, gnss_msg.longitude)
            # self._gnss_stream.send(msg)
            # self._gnss_stream.send(watermark_msg)
            self._gnss_stream = msg

class CarlaGNSSDriverOperator():
    """Publishes GNSSMessages (transform, altitude, latitude, longitude) from
    the GNSS sensor.

    This operator attaches to a vehicle at the required position with respect
    to the vehicle, registers callback functions to retrieve the GNSS
    measurements and publishes it to downstream operators.

    Args:
        ground_vehicle_id_stream (:py:class:`erdos.ReadStream`): Stream on
            which the operator receives the id of the ego vehicle. It uses this
            id to get a simulator handle to the vehicle.
        gnss_stream (:py:class:`erdos.WriteStream`): Stream on which the
            operator sends GNSS info.
        gnss_setup (:py:class:`pylot.drivers.sensor_setup.GNSSSetup`):
            Setup of the GNSS sensor.
        flags (absl.flags): Object to be used to access absl flags.
    """
    def initialize(self, configuration):
        return CarlaGNSSDriverState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        token = tokens.get('carla_stream').get_data()
        msg = pickle.loads(bytes(token))

        state.vehicle_id_msg = msg['vehicle_id_stream']

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
        # Read the vehicle ID from the vehicle ID stream.
        vehicle_id = _state.vehicle_id_msg.data
        _state._logger.debug(
            "The GNSSDriverOperator received the vehicle id: {}".format(
                vehicle_id))

        # Connect to the world.
        _, world = get_world(_state.cfg["simulator_host"],
                             _state.cfg["simulator_port"],
                             _state.cfg["simulator_timeout"])

        set_simulation_mode(world, _state.cfg)

        # Retrieve the vehicle and install the GNSS sensor.
        _state._vehicle = get_vehicle_handle(world, vehicle_id)
        gnss_blueprint = world.get_blueprint_library().find(
            'sensor.other.gnss')

        # Set the noise and bias parameters.
        gnss_blueprint.set_attribute('noise_alt_stddev',
                                     str(_state.cfg["gnss_noise_stddev_alt"]))
        gnss_blueprint.set_attribute('noise_lat_stddev',
                                     str(_state.cfg["gnss_noise_stddev_lat"]))
        gnss_blueprint.set_attribute('noise_lon_stddev',
                                     str(_state.cfg["gnss_noise_stddev_lon"]))
        gnss_blueprint.set_attribute('noise_alt_bias',
                                     str(_state.cfg["gnss_bias_alt"]))
        gnss_blueprint.set_attribute('noise_lat_bias',
                                     str(_state.cfg["gnss_bias_lat"]))
        gnss_blueprint.set_attribute('noise_lon_bias',
                                     str(_state.cfg["gnss_bias_lon"]))

        if _state.cfg["simulator_gnss_frequency"] == -1:
            gnss_blueprint.set_attribute('sensor_tick', '0.0')
        else:
            gnss_blueprint.set_attribute(
                'sensor_tick', str(1.0 / _state.cfg["simulator_gnss_frequency"]))
        transform = _state._gnss_setup.get_transform().as_simulator_transform()
        _state._logger.debug("Spawning a GNSS sensor: {}".format(
            _state._gnss_setup))
        _state._gnss = world.spawn_actor(gnss_blueprint,
                                       transform,
                                       attach_to=_state._vehicle)

        # Register the callback on the GNSS sensor.
        _state._gnss.listen(_state.process_gnss)

        result = {"gnss_stream": _state._gnss_stream,
                  "timestamp": _state.msg_timestamp
                  }
        return {'gnss_stream': pickle.dumps(result)}




def register():
    return CarlaGNSSDriverOperator


if __name__ == '__main__':
    config = {
        "log_file_name": 'pylot.log',
        "name": "center_camera_operator",
        "simulator_host": 'localhost',
        "simulator_port": 2000,
        "simulator_timeout": 10,
        "simulator_mode": 'synchronous',
        "simulator_fps": 20,
        "gnss_noise_stddev_alt": 0.0,
        "gnss_noise_stddev_lat": 0.0,
        "gnss_noise_stddev_lon": 0.0,
        "gnss_bias_alt": 0.0,
        "gnss_bias_lat": 0.0,
        "gnss_bias_lon": 0.0,
        "simulator_gnss_frequency": 10
    }
    operator = CarlaGNSSDriverOperator()
    state = operator.initialize(config)
    operator.input_rule(None, state, None)
    operator.run(None, state, None)
