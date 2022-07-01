import enum
import logging
import pickle
import sys
import threading
import time
# sys.path.remove("/home/erdos/workspace/pylot")
# sys.path.append('/home/erdos/workspace/zenoh-flow-auto-driving')
# sys.path.append("/home/erdos/workspace/pylot/dependencies/CARLA_0.9.10.1/PythonAPI/carla/dist/carla-0.9.10-py3.7-linux-x86_64.egg")
from functools import total_ordering
import pylot.utils
from pylot.control.messages import ControlMessage
from zenoh_flow import  Inputs, Operator


class EndState:
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

class EndOperator(Operator):
    def initialize(self, configuration):
        return EndState(configuration)


    def input_rule(self, _ctx, state, tokens):
        token = tokens.get("headOperatorMsg")
        imu_msg = pickle.loads(bytes(token.get_data()))
        if imu_msg["head_stream"] is None:
            token.set_action_drop()
            return  False
        state.count = imu_msg["head_stream"]
        return True


    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs


    def finalize(self, state):
        return None


    def run(self, _ctx, _state, inputs):
        time.sleep(1)
        _state.count = _state.count + 1
        result = {"data": _state.count}
        print("end....................{}".format(result))
        return {'endOperatorMsg': pickle.dumps(result), "feedback-AC" : pickle.dumps(result)}


def register():
    return EndOperator

