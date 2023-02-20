import pickle

from zenoh_flow import Sink

from pylot import constant


class MySink(Sink):
    def initialize(self, configuration):
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        msg = pickle.loads(input.data)
        constant.zenoh_deque.put(msg)
        print("control sink -------------------------> {} ".format(msg))

def register():
    return MySink
