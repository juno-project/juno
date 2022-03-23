import os
import pickle

from zenoh_flow import Sink

import numpy as np
import cv2

class MySink(Sink):
    def initialize(self, configuration):
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        msg = pickle.loads(input.data)
        print(f"LanesMessage sink: {msg}")

        out_poss_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".")
        out_poss_path = os.path.join(out_poss_path, "test_data/LaneDetectionOperator/output/lane_msg-6871.pkl")
        out_data = pickle.load(open(out_poss_path, "rb"))
        print("LanesMessage  sink out_data : {}".format(out_data))



def register():
    return MySink
