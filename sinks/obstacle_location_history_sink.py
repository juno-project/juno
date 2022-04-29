import pickle

from zenoh_flow import Sink

class MySink(Sink):
    def initialize(self, configuration):
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        msg = pickle.loads(input.data)
        print("obstacle location history sink {}".format(msg))

def register():
    return MySink
