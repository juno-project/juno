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


class HeadState:
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
        self.count = 0
        self.source_count = 0


class HeadOperator(Operator):
    def initialize(self, configuration):
        return HeadState(configuration)


    def input_rule(self, _ctx, state, tokens):
        test_token = tokens.get("testMsg")
        if not test_token.is_pending() :
            end_msg = pickle.loads(bytes(test_token.get_data()))
            state.source_count = end_msg["data"]
        return True


    def output_rule(self, _ctx, _state, outputs, _deadline_miss):
        return outputs


    def finalize(self, state):
      return None


    def run(self, _ctx, _state, inputs):
        try:
            feedback = inputs.get('feedback-AC').data
            data = pickle.loads(feedback)
            _state.count =  data["data"]
            result = {"head_stream": _state.count}
        except Exception as e:
            result = {"head_stream": None}
            if _state.source_count == 1:
                result = {"head_stream": _state.count}

        print("head....................{}".format(result))
        return {'headOperatorMsg': pickle.dumps(result)}


def register():
    return HeadOperator

