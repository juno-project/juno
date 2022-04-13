"""This module implements an operator acts like a camera driver when
using the simulator.

The operator attaches a camera to the ego vehicle, receives camera frames from
the simulator, and sends them on its output stream.
"""
import os
import pickle
import threading
import pylot.utils
from pylot.drivers.sensor_setup import RGBCameraSetup
from pylot.perception.camera_frame import CameraFrame
from pylot.perception.depth_frame import DepthFrame
from pylot.perception.messages import DepthFrameMessage, FrameMessage, \
    SegmentedFrameMessage
from pylot.perception.segmentation.segmented_frame import SegmentedFrame
from pylot.simulation.utils import get_vehicle_handle, get_world, \
    set_simulation_mode
from zenoh_flow import  Inputs, Operator, Outputs

class CarlaCameraDriverState:
    """Publishes images onto the desired stream from a camera.

    This operator attaches a vehicle at the required position with respect to
    the vehicle, registers callback functions to retrieve the images and
    publishes it to downstream operators.

    Args:
        ground_vehicle_id_stream (:py:class:`erdos.ReadStream`): Stream on
            which the operator receives the id of the ego vehicle. It uses this
            id to get a simulator handle to the vehicle.
        camera_stream (:py:class:`erdos.WriteStream`): Stream on which the
            operator sends camera frames.
        notify_reading_stream (:py:class:`erdos.WriteStream`): Stream on which
            the operator sends notifications when it receives camera frames.
        camera_setup (:py:class:`pylot.drivers.sensor_setup.RGBCameraSetup`):
            Setup of the camera.
        flags (absl.flags): Object to be used to access absl flags.
    """

    def __init__(self, cfg):
        # erdos.add_watermark_callback([release_sensor_stream], [],
        #                              self.release_data)
        # self._vehicle_id_stream = ground_vehicle_id_stream
        # self._camera_stream = camera_stream
        # self._notify_reading_stream = notify_reading_stream

        self.cfg = cfg
        # self._logger = erdos.utils.setup_logging(self.config.name,
        #                                          self.config.log_file_name)

        self._logger = pylot.utils.get_logger(cfg["log_file_name"])

        # center_camera_setup = RGBCameraSetup('center_camera',
        #                                      FLAGS.camera_image_width,
        #                                      FLAGS.camera_image_height, transform,
        #                                      FLAGS.camera_fov)

        CENTER_CAMERA_LOCATION = pylot.utils.Location(1.3, 0.0, 1.8)
        transform = pylot.utils.Transform(CENTER_CAMERA_LOCATION,
                                          pylot.utils.Rotation(pitch=-15))
        camera_setup = RGBCameraSetup(cfg["camera_name"],
                                      cfg["camera_image_width"],
                                      cfg["camera_image_height"], transform,
                                      cfg["camera_fov"])

        self.notify_reading_msg = None
        self._camera_stream = None
        self._notify_reading_stream = None
        self.msg_timestamp = 0
        self._camera_setup = camera_setup
        # The hero vehicle actor object we obtain from the simulator.
        self._vehicle = None
        # The camera sensor actor object we obtain from the simulator.
        self._camera = None
        self._pickle_lock = threading.Lock()
        self._pickled_messages = {}
        # Lock to ensure that the callbacks do not execute simultaneously.
        self._lock = threading.Lock()
        # If false then the operator does not send data until it receives
        # release data watermark. Otherwise, it sends as soon as it
        # receives it.
        self._release_data = False

    def process_images(self, simulator_image):
        """Invoked when an image is received from the simulator."""
        game_time = int(simulator_image.timestamp * 1000)
        # timestamp = erdos.Timestamp(coordinates=[game_time])
        # watermark_msg = erdos.WatermarkMessage(timestamp)
        timestamp = game_time
        watermark_msg = timestamp
        self.msg_timestamp = game_time
        # with erdos.profile(self.config.name + '.process_images',
        #                    self,
        #                    event_data={'timestamp': str(timestamp)}):
        # Ensure that the code executes serially

        with self._lock:
            msg = None
            if self._camera_setup.camera_type == 'sensor.camera.rgb':
                msg = FrameMessage(
                    timestamp,
                    CameraFrame.from_simulator_frame(
                        simulator_image, self._camera_setup))
            elif self._camera_setup.camera_type == 'sensor.camera.depth':
                # Include the transform relative to the vehicle.
                # simulator_image.transform returns the world transform,
                # but we do not use it directly.
                msg = DepthFrameMessage(
                    timestamp,
                    DepthFrame.from_simulator_frame(
                        simulator_image,
                        self._camera_setup,
                        save_original_frame=self.cfg["visualize_depth_camera"] == "True"))
            elif (self._camera_setup.camera_type ==
                  'sensor.camera.semantic_segmentation'):
                msg = SegmentedFrameMessage(
                    timestamp,
                    SegmentedFrame.from_simulator_image(
                        simulator_image, self._camera_setup))

            if self._release_data:
                # self.cfg._camera_stream.send(msg)
                # self.cfg._camera_stream.send(watermark_msg)
                self._camera_stream = msg
            else:
                # Pickle the data, and release it upon release msg receipt.
                pickled_msg = pickle.dumps(
                    msg, protocol=pickle.HIGHEST_PROTOCOL)
                with self._pickle_lock:
                    self._pickled_messages[msg.timestamp] = pickled_msg
                # self._notify_reading_stream.send(watermark_msg)
                self._notify_reading_stream = watermark_msg

    def release_data(self, timestamp):
        self._release_data = True
        # if timestamp.is_top:
        #     # The operator can always send data ASAP.
        #     self._release_data = True
        # else:
        #     self._logger.debug("@{}: {} releasing sensor data".format(
        #         timestamp, self.config.name))
        #     watermark_msg = erdos.WatermarkMessage(timestamp)
        #     self._camera_stream.send_pickled(timestamp,
        #                                      self._pickled_messages[timestamp])
        #     # Note: The operator is set not to automatically propagate
        #     # watermark messages received on input streams. Thus, we can
        #     # issue watermarks only after the simulator callback is invoked.
        #     self._camera_stream.send(watermark_msg)
        #     with self._pickle_lock:
        #         del self._pickled_messages[timestamp]


class CarlaCameraDriverOperator(Operator):

    def initialize(self, configuration):
        return CarlaCameraDriverState(configuration)

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        token = tokens.get('carlaOperatorMsg').get_data()
        msg = pickle.loads(bytes(token))

        state.vehicle_id_msg = msg['vehicle_id_stream']
        timestamp = msg['timestamp']
        state.release_data(timestamp)

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
            "The CameraDriverOperator received the vehicle id: {}".format(
                vehicle_id))

        # Connect to the world. We connect here instead of in the constructor
        # to ensure we're connected to the latest world.
        _, world = get_world(_state.cfg["simulator_host"],
                             _state.cfg["simulator_port"],
                             _state.cfg["simulator_timeout"])

        set_simulation_mode(world, _state.cfg)

        self._vehicle = get_vehicle_handle(world, vehicle_id)
        print("self._vehicle :{}".format(self._vehicle))

        # Install the camera.
        camera_blueprint = world.get_blueprint_library().find(
            _state._camera_setup.camera_type)
        camera_blueprint.set_attribute('image_size_x',
                                       str(_state._camera_setup.width))
        camera_blueprint.set_attribute('image_size_y',
                                       str(_state._camera_setup.height))
        camera_blueprint.set_attribute('fov', str(_state._camera_setup.fov))
        if _state.cfg["simulator_camera_frequency"] == -1:
            camera_blueprint.set_attribute('sensor_tick', '0.0')
        else:
            camera_blueprint.set_attribute(
                'sensor_tick',
                str(1.0 / _state.cfg["simulator_camera_frequency"]))

        transform = _state._camera_setup.get_transform().as_simulator_transform()

        _state._logger.debug("Spawning a camera: {}".format(_state._camera_setup))
        _state._camera = world.spawn_actor(camera_blueprint,
                                           transform,
                                           attach_to=self._vehicle)

        # Register the callback on the camera.
        _state._camera.listen(_state.process_images)

        result = {"camera_stream": _state._camera_stream,
                  "notify_reading_stream": _state._notify_reading_stream,
                  "timestamp": _state.msg_timestamp
                  }

        if _state._camera_stream != None:
            print("carlaCameraDriverMsg : {}".format(_state._camera_stream))
            msg = _state._camera_stream
            out_path = "/home/erdos/workspace/zenoh-flow-auto-driving/test_out"
            os.makedirs(out_path, exist_ok=True)
            msg.frame.save(msg.timestamp, out_path,
                           'carla_camera_driver_operator-test_dtector-{}'.format(msg.timestamp))

        print("carlaCameraDriverMsg : {}".format(result))
        return {'carlaCameraDriverMsg': pickle.dumps(result)}


def register():
    return CarlaCameraDriverOperator


if __name__ == '__main__':

    config = {
        "log_file_name": 'pylot.log',
        "name": "center_camera_operator",
        "simulator_host": 'localhost',
        "simulator_port": 2000,
        "simulator_timeout": 10,
        "simulator_mode": 'synchronous',
        "simulator_fps": 20
    }
    operator = CarlaCameraDriverOperator()
    state = operator.initialize(config)
    operator.input_rule(None, state, None)
    operator.run(None, state, None)
