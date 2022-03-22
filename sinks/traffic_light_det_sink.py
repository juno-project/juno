
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
        traffic_lights = msg.obstacles
        for traffic_light in traffic_lights:
            print(f"traffic light color {traffic_light.state}")
            print(f"traffic light score {traffic_light.confidence}")

def register():
    return MySink
