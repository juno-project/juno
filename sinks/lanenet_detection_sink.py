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



def register():
    return MySink
