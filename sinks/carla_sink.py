import os
import pickle

from zenoh_flow import Sink


class MySink(Sink):
    def initialize(self, configuration):
        print("sink init.............")
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        data = pickle.loads(input.data)
        if data is not None:
            print("sink vehicle_id_msg : {}".format(data["vehicle_id_stream"]))


def register():
    return MySink
