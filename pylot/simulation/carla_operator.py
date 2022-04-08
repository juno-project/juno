import enum
import heapq
import logging
import pickle
import sys
import threading
import time
# sys.path.remove("/home/erdos/workspace/pylot")
# sys.path.append('/home/erdos/workspace/zenoh-flow-auto-driving')
# sys.path.append("/home/erdos/workspace/pylot/dependencies/CARLA_0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg")
from functools import total_ordering
from carla import Location, VehicleControl, command
import erdos
import pylot.simulation.utils
import pylot.utils
from pylot.control.messages import ControlMessage
from pylot.perception.messages import ObstaclesMessage, SpeedSignsMessage, \
    StopSignsMessage, TrafficLightsMessage, Message

class CarlaState:
    """Initializes and controls a CARLA simulation.

    This operator connects to the simulation, sets the required weather in the
    simulation world, initializes the required number of actors, and the
    vehicle that the rest of the pipeline drives.

    Args:
        flags: A handle to the global flags instance to retrieve the
            configuration.

    Attributes:
        _client: A connection to the simulator.
        _world: A handle to the world running inside the simulation.
        _vehicles: A list of identifiers of the vehicles inside the simulation.
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.pose_msg = None
        self.pose_msg_for_control = None
        self.ground_traffic_lights_msg = None
        self.ground_obstacles_msg = None
        self.ground_speed_limit_signs_msg = None
        self.ground_stop_signs_msg= None
        self.vehicle_id_msg = None
        self.open_drive_msg = None
        self.global_trajectory_msg = None

        # logger
        logger = logging.getLogger(__name__)
        logger.setLevel(level=logging.INFO)
        handler = logging.FileHandler(cfg["log_file_name"])
        handler.setLevel(logging.INFO)
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        handler.setFormatter(formatter)
        logger.addHandler(handler)

        # self._logger = erdos.utils.setup_logging(cfg["name"],
        #                                          cfg["log_file_name"])
        self._logger = logger
        self._csv_logger = erdos.utils.setup_csv_logging(
            cfg["name"] + '-csv', cfg["csv_log_file_name"])
        # Connect to simulator and retrieve the world running.
        self._client, self._world = pylot.simulation.utils.get_world(
            cfg["simulator_host"], cfg["simulator_port"],
            cfg["simulator_timeout"])
        self._simulator_version = self._client.get_client_version()

        if not cfg["scenario_runner"] and \
                cfg["control"] != "manual":
            # Load the appropriate town.
            self._initialize_world()

        # Save the spectator handle so that we don't have to repeteadly get the
        # handle (which is slow).
        self._spectator = self._world.get_spectator()

        if pylot.simulation.utils.check_simulator_version(
                self._simulator_version, required_minor=9, required_patch=8):
            # Any simulator version after 0.9.7.
            # Create a traffic manager to that auto pilot works.
            self._traffic_manager = self._client.get_trafficmanager(
                cfg["carla_traffic_manager_port"])
            self._traffic_manager.set_synchronous_mode(
                cfg["simulator_mode"] == 'synchronous')

        if cfg["scenario_runner"]:
            # Tick until 4.0 seconds time so that all synchronous scenario runs
            # start at exactly the same game time.
            pylot.simulation.utils.set_synchronous_mode(self._world, 1000)
            self._tick_simulator_until(4000)

        pylot.simulation.utils.set_simulation_mode(self._world, self.cfg)

        if cfg["scenario_runner"] == "True" or cfg["control"] == "manual":
            # # Wait until the ego vehicle is spawned by the scenario runner.
            self._logger.info("Waiting for the scenario to be ready ...")
            self._ego_vehicle = pylot.simulation.utils.wait_for_ego_vehicle(
                self._world)
            self._logger.info("Found ego vehicle")
            # Spawn ego vehicle, people and vehicles.

        else:
            # Spawn ego vehicle, people and vehicles.
            (self._ego_vehicle, self._vehicle_ids,
             self._people) = pylot.simulation.utils.spawn_actors(
                self._client, self._world,
                cfg["carla_traffic_manager_port"],
                self._simulator_version,
                cfg["simulator_spawn_point_index"],
                cfg["control"] == 'simulator_auto_pilot',
                cfg["simulator_num_people"],
                cfg["simulator_num_vehicles"], self._logger)

        pylot.simulation.utils.set_vehicle_physics(
            self._ego_vehicle, cfg["simulator_vehicle_moi"]/10,
            cfg["simulator_vehicle_mass"])

        # Lock used to ensure that simulator callbacks are not executed
        # concurrently.
        self._lock = threading.Lock()

        # Dictionary that stores the processing times when sensors are ready
        # to realease data. This info is used to calculate the real processing
        # time of our pipeline without including simulator-induced sensor
        # delays.
        self._next_localization_sensor_reading = None
        self._next_control_sensor_reading = None
        self._simulator_in_sync = False
        self._tick_events = []
        self._control_msgs = {}

    def on_control_msg(self, msg):
        """ Invoked when a ControlMessage is received.

        Args:
            msg: A control.messages.ControlMessage message.
        """
        self._logger.debug('@{}: received control message'.format(
            msg.timestamp))
        if self.cfg["simulator_mode"] == 'pseudo-asynchronous':
            heapq.heappush(
                self._tick_events,
                (msg.timestamp, TickEvent.CONTROL_CMD))
            self._control_msgs[msg.timestamp] = msg
            # Tick until the next sensor read game time to ensure that the
            # data-flow has a new round of sensor inputs. Apply control
            # commands if they must be applied before the next sensor read.
            self._consume_next_event()
        else:
            # If auto pilot or manual mode is enabled then we do not apply the
            # control, but we still want to tick in this method to ensure that
            # all operators finished work before the world ticks.
            if self.cfg["control"] not in ['simulator_auto_pilot', 'manual']:
                self._apply_control_msg(msg)
            # Tick the world after the operator received a control command.
            # This usually indicates that all the operators have completed
            # processing the previous timestamp (with the exception of logging
            # operators that are not part of the main loop).
            self._tick_simulator()

    def _consume_next_event(self):
        while True:
            (sim_time, event_type) = heapq.heappop(self._tick_events)
            if event_type == TickEvent.SENSOR_READ:
                self._tick_simulator_until(sim_time)
                break
            elif event_type == TickEvent.CONTROL_CMD:
                self._tick_simulator_until(sim_time)
                control_msg = self._control_msgs[sim_time]
                self._apply_control_msg(control_msg)

    def on_pipeline_finish(self, timestamp):
        self._logger.debug("@{}: Received pipeline finish.".format(timestamp))
        game_time = timestamp
        if (self.cfg["simulator_control_frequency"] == -1
                or self._next_control_sensor_reading is None
                or game_time == self._next_control_sensor_reading):
            # There was supposed to be a control message for this timestamp
            # too. Send the Pose message and continue after the control message
            # is received.
            self._update_next_control_pseudo_asynchronous_ticks(timestamp)
            self._send_hero_vehicle_data(self.pose_msg_for_control,
                                          timestamp)
            self._update_spectactor_pose()
        else:
            # No pose message was supposed to be sent for this timestamp, we
            # need to consume the next event to move the dataflow forward.
            self._consume_next_event()

    def on_sensor_ready(self, timestamp):
        # The first sensor reading needs to be discarded because it might
        # not be correctly spaced out.
        if not self._simulator_in_sync:
            self._simulator_in_sync = True

    def send_actor_data(self, msg):
        """ Callback function that gets called when the world is ticked.
        This function sends a WatermarkMessage to the downstream operators as
        a signal that they need to release data to the rest of the pipeline.

        Args:
            msg: Data recieved from the simulation at a tick.
        """
        # Ensure that the callback executes serially.
        with self._lock:
            game_time = int(msg.elapsed_seconds * 1000)
            self._logger.info(
                'The world is at the timestamp {}'.format(game_time))
            self.msg_timestamp = game_time
            timestamp = game_time
            # with erdos.profile(self.cfg["name"] + '.send_actor_data',
            #                    self,
            #                    event_data={'timestamp': str(timestamp)}):
            if (self.cfg["simulator_localization_frequency"] == -1
                    or self._next_localization_sensor_reading is None or
                    game_time == self._next_localization_sensor_reading):
                if self.cfg["simulator_mode"] == 'pseudo-asynchronous':
                    self._update_next_localization_pseudo_async_ticks(
                        game_time)
                self._send_hero_vehicle_data(self.pose_msg, timestamp)
                self._send_ground_actors_data(timestamp)
                self._update_spectactor_pose()

            if self.cfg["simulator_mode"] == "pseudo-asynchronous" and (
                    self.cfg["simulator_control_frequency"] == -1
                    or self._next_control_sensor_reading is None
                    or game_time == self._next_control_sensor_reading):
                self._update_next_control_pseudo_asynchronous_ticks(
                    game_time)
                self._send_hero_vehicle_data(self.pose_msg_for_control,
                                              timestamp)
                self._update_spectactor_pose()

    def _update_next_localization_pseudo_async_ticks(self, game_time: int):
        if self.cfg["simulator_localization_frequency"] > -1:
            self._next_localization_sensor_reading = (
                game_time +
                int(1000 / self.cfg["simulator_localization_frequency"]))
            if not self._simulator_in_sync:
                # If this is the first sensor reading, then tick
                # one more time because the second sensor reading
                # is sometimes delayed by 1 tick.
                self._next_localization_sensor_reading += int(
                    1000 / self.cfg["simulator_fps"])
        else:
            self._next_localization_sensor_reading = (
                game_time + int(1000 / self.cfg["simulator_fps"]))
        heapq.heappush(
            self._tick_events,
            (self._next_localization_sensor_reading, TickEvent.SENSOR_READ))

    def _update_next_control_pseudo_asynchronous_ticks(self, game_time: int):
        if self.cfg["simulator_control_frequency"] > -1:
            self._next_control_sensor_reading = (
                game_time +
                int(1000 / self.cfg["simulator_control_frequency"]))
        else:
            self._next_control_sensor_reading = (
                game_time + int(1000 / self.cfg["simulator_fps"]))
        if (self._next_control_sensor_reading !=
                self._next_localization_sensor_reading):
            heapq.heappush(
                self._tick_events,
                (self._next_control_sensor_reading, TickEvent.SENSOR_READ))

    def _initialize_world(self):
        """ Setups the world town, and activates the desired weather."""
        if self._simulator_version == '0.9.5':
            # TODO (Sukrit) :: ERDOS provides no way to retrieve handles to the
            # class objects to do garbage collection. Hence, objects from
            # previous runs of the simulation may persist. We need to clean
            # them up right now. In future, move this logic to a seperate
            # destroy function.
            pylot.simulation.utils.reset_world(self._world)
        else:
            self._world = self._client.load_world('Town{:02d}'.format(
                self.cfg["simulator_town"]))
        self._logger.info('Setting the weather to {}'.format(
            self.cfg["simulator_weather"]))
        pylot.simulation.utils.set_weather(self._world,
                                           self.cfg["simulator_weather"])

    def _tick_simulator(self):
        if (self.cfg["simulator_mode"] == 'asynchronous-fixed-time-step'
                or self.cfg["simulator_mode"] == 'asynchronous'):
            # No need to tick when running in these modes.
            return
        self._world.tick()

    def _tick_simulator_until(self, goal_time: int):
        while True:
            snapshot = self._world.get_snapshot()
            sim_time = int(snapshot.timestamp.elapsed_seconds * 1000)
            if sim_time < goal_time:
                self._world.tick()
            else:
                return

    def _apply_control_msg(self, msg: ControlMessage):
        # Transform the message to a simulator control cmd.
        vec_control = VehicleControl(throttle=msg.throttle,
                                     steer=msg.steer,
                                     brake=msg.brake,
                                     hand_brake=msg.hand_brake,
                                     reverse=msg.reverse)
        self._client.apply_batch_sync(
            [command.ApplyVehicleControl(self._ego_vehicle.id, vec_control)])

    def _send_hero_vehicle_data(self, msg, timestamp):
        vec_transform = pylot.utils.Transform.from_simulator_transform(
            self._ego_vehicle.get_transform())
        velocity_vector = pylot.utils.Vector3D.from_simulator_vector(
            self._ego_vehicle.get_velocity())
        forward_speed = velocity_vector.magnitude()
        pose = pylot.utils.Pose(vec_transform, forward_speed, velocity_vector,
                                timestamp)
        msg = Message(timestamp, pose)
        # stream.send(erdos.Message(timestamp, pose))
        # stream.send(erdos.WatermarkMessage(timestamp))

    def _send_ground_actors_data(self, timestamp):
        # Get all the actors in the simulation.
        actor_list = self._world.get_actors()

        (vehicles, people, traffic_lights, speed_limits, traffic_stops
         ) = pylot.simulation.utils.extract_data_in_pylot_format(actor_list)

        # Send ground people and vehicles.
        # self.ground_obstacles_stream.send(
        #     ObstaclesMessage(timestamp, vehicles + people))
        self.ground_obstacles_msg = ObstaclesMessage(timestamp, vehicles + people)
        # print(ObstaclesMessage(timestamp, vehicles + people))
        # self.ground_obstacles_stream.send(erdos.WatermarkMessage(timestamp))

        # Send ground traffic lights.
        # self.ground_traffic_lights_stream.send(
        #     TrafficLightsMessage(timestamp, traffic_lights))
        # self.ground_traffic_lights_stream.send(
        #     erdos.WatermarkMessage(timestamp))

        self.ground_traffic_lights_msg = TrafficLightsMessage(timestamp, traffic_lights)
        # Send ground speed signs.
        # self.ground_speed_limit_signs_stream.send(
        #     SpeedSignsMessage(timestamp, speed_limits))
        # self.ground_speed_limit_signs_stream.send(
        #     erdos.WatermarkMessage(timestamp))
        #
        self.ground_speed_limit_signs_msg = SpeedSignsMessage(timestamp, speed_limits)

        # Send stop signs.
        # self.ground_stop_signs_stream.send(
        #     StopSignsMessage(timestamp, traffic_stops))
        # self.ground_stop_signs_stream.send(erdos.WatermarkMessage(timestamp))

        self.ground_stop_signs_msg = StopSignsMessage(timestamp, traffic_stops)

    def _send_world_data(self):
        """ Sends ego vehicle id, open drive and trajectory messages."""
        # Send the id of the ego vehicle. This id is used by the driver
        # operators to get a handle to the ego vehicle, which they use to
        # attach sensors.
        # self.vehicle_id_stream.send(
        #     erdos.Message(erdos.Timestamp(coordinates=[0]),
        #                   self._ego_vehicle.id))
        # self.vehicle_id_stream.send(
        #     erdos.WatermarkMessage(erdos.Timestamp(is_top=True)))
        timestamp = 0
        self.vehicle_id_msg = Message(timestamp, self._ego_vehicle.id)


        # Send open drive string.
        # self.open_drive_stream.send(
        #     erdos.Message(erdos.Timestamp(coordinates=[0]),
        #                   self._world.get_map().to_opendrive()))
        # top_watermark = erdos.WatermarkMessage(erdos.Timestamp(is_top=True))

        self.open_drive_msg = Message(timestamp, self._world.get_map().to_opendrive())

        # self.open_drive_stream.send(top_watermark)
        # self.global_trajectory_stream.send(top_watermark)

    def _update_spectactor_pose(self):
        # Set the world simulation view with respect to the vehicle.
        v_pose = self._ego_vehicle.get_transform()
        v_pose.location -= 10 * Location(v_pose.get_forward_vector())
        v_pose.location.z = 5
        self._spectator.set_transform(v_pose)


class CarlaOperator():
    def initialize(self, configuration):
        return CarlaState(configuration)


    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        token = tokens.get('carlaMsg').get_data()
        carlaMsg = pickle.loads(bytes(token))
        #
        control_msg = carlaMsg['control']
        timestamp = carlaMsg['timestamp']

        state.on_control_msg(control_msg)
        state.on_sensor_ready(timestamp)
        state.on_pipeline_finish(timestamp)
        return True


    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs


    def finalize(self, state):
        return None


    def run(self, _ctx, _state, inputs):
        _state._send_world_data()
        # Tick here once to ensure that the driver operators can get a handle
        # to the ego vehicle.
        # XXX(ionel): Hack to fix a race condition. Driver operators
        # register a simulator listen callback only after they've received
        # the vehicle id value. We miss frames if we tick before
        # they register a listener. Thus, we sleep here a bit to
        # give them sufficient time to register a callback.
        time.sleep(4)
        _state._tick_simulator()
        time.sleep(4)
        # The older CARLA versions require an additional tick to sync
        # sensors.
        _state._world.on_tick(_state.send_actor_data)
        _state._tick_simulator()

        result = {"pose_msg": _state.pose_msg,
                  "pose_msg_for_control": _state.pose_msg_for_control,
                  "ground_traffic_lights_msg": _state.ground_traffic_lights_msg,
                  "ground_obstacles_msg": _state.ground_obstacles_msg,
                  "ground_speed_limit_signs_msg": _state.ground_speed_limit_signs_msg,
                  "ground_stop_signs_msg": _state.ground_stop_signs_msg,
                  "vehicle_id_msg": _state.vehicle_id_msg,
                  "open_drive_msg": _state.open_drive_msg,
                  "global_trajectory_msg": _state.global_trajectory_msg,
                  "timestamp": _state.msg_timestamp
                  }
        return {'carlaOperatorMsg': pickle.dumps(result)}


@total_ordering
class TickEvent(enum.Enum):
    CONTROL_CMD = 1
    SENSOR_READ = 2

    def __lt__(self, other):
        if self.__class__ is other.__class__:
            return self.value < other.value
        return NotImplemented

def register():
    return CarlaOperator

if __name__=='__main__':
    # project_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + "..")
    # control_path = os.path.join(project_path, "test_data/CarlaOperator/input/ControlMessage.pkl")
    control_msg = pickle.load(open("/home/erdos/workspace/zenoh-flow-auto-driving/test_data/CarlaOperator/input/ControlMessage.pkl", "rb"))
    config = {
        'name': 'simulator_bridge_operator',
        'log_file_name': 'pylot.log',
        'csv_log_file_name': 'pylot.csv',
        'simulator_host': 'localhost',
        'simulator_port': 2000,
        'simulator_timeout': 10,
        'scenario_runner': 'False',
        'control': 'simulator_auto_pilot',
        'carla_traffic_manager_port': 8000,
        'simulator_mode': 'synchronous',
        'simulator_control_frequency': -1,
        'simulator_localization_frequency': -1,
        'simulator_fps': 20,
        'simulator_town': 1,
        "simulator_weather": 'ClearNoon',
        "simulator_spawn_point_index": -1,
        "simulator_num_people": 250,
        "simulator_num_vehicles": 20,
        "simulator_vehicle_moi": 0.1,
        "simulator_vehicle_mass": 100
    }
    carla = CarlaOperator()
    state = carla.initialize(config)
    carla.run(None, state, None)
