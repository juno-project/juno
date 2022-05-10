import pickle

import pylot.control.utils
import pylot.planning.utils
from pylot.control.messages import ControlMessage
from pylot.control.pid import PIDLongitudinalController


class PIDControlState:
    def __init__(self, cfg):
        print("operator state initializing")
        self.cfg = cfg
        self.min_pid_steer_waypoint_distance = cfg['min_pid_steer_waypoint_distance']
        self.min_pid_speed_waypoint_distance = cfg['min_pid_speed_waypoint_distance']
        self.steer_gain = float(cfg['steer_gain'])
        self.throttle_max = cfg['throttle_max']
        self.brake_max = cfg['brake_max']
        
        self.execution_mode = cfg['execution_mode']
        self.simulator_control_frequency = float(cfg['simulator_control_frequency'])
        self.simulator_fps = cfg['simulator_fps']
        pid_p = float(cfg['pid_p'])
        pid_d = float(cfg['pid_d'])
        pid_i = float(cfg['pid_i'])
        pid_use_real_time = False
        if self.execution_mode == 'real-world':
            # The PID is executing on a real car. Use the real time delta
            # between two control commands.
            pid_use_real_time = True
        if self.simulator_control_frequency == -1:
            dt = 1.0 / self.simulator_fps
        else:
            dt = 1.0 / self.simulator_control_frequency
        self._pid = PIDLongitudinalController(pid_p, pid_d,
                                              pid_i, dt,
                                              pid_use_real_time)
        # Queues in which received messages are stored.

class PIDControlOperator:
    def initialize(self, configuration):
        return PIDControlState(configuration)

    def finalize(self, state):
        print("finalize")
        return None

    def input_rule(self, _ctx, state, tokens):
        # Using input rules
        print("input")
        return True

    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        print("output")
        return outputs

    def run(self, _ctx, _state, inputs):
        """Computes and sends the control command on the control stream.

        Invoked when all input streams have received a watermark.

        Args:
            timestamp (:py:class:`erdos.timestamp.Timestamp`): The timestamp of
                the watermark.
        """
        print("operator starts running")
        msg = inputs.get("PIDMsg").data
        msg = pickle.loads(msg)
        timestamp = msg[0]
        print('@{}: received watermark'.format(timestamp))
        ego_transform = msg[1]
        # Vehicle speed in m/s.
        current_speed = 1.468
        waypoints = msg[2]
        try:
            angle_steer = waypoints.get_angle(
                ego_transform, _state.min_pid_steer_waypoint_distance)
            target_speed = waypoints.get_target_speed(
                ego_transform, _state.min_pid_speed_waypoint_distance)
            throttle, brake = pylot.control.utils.compute_throttle_and_brake(
                _state._pid, current_speed, target_speed, _state.throttle_max, _state.brake_max)
            steer = pylot.control.utils.radians_to_steer(
                angle_steer, _state.steer_gain)
        except ValueError:
            print('Braking! No more waypoints to follow.')
            throttle, brake = 0.0, 0.5
            steer = 0.0
        print(
            '@{}: speed {}, location {}, steer {}, throttle {}, brake {}'.
            format(timestamp, current_speed, ego_transform, steer, throttle,
                   brake))
        return {'ControlMessage': pickle.dumps(ControlMessage(steer, throttle, brake, False, False, timestamp))}

def register():
    return PIDControlOperator