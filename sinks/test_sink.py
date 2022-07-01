import os
import pickle

from zenoh_flow import Sink

from pylot import constant


class MySink(Sink):
    def initialize(self, configuration):
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        data = pickle.loads(input.data)
        constant.zenoh_deque.put(data)
        print("test sink : {}".format(data))
def register():
    return MySink




