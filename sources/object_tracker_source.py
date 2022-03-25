import sys

sys.path.append('/home/zenoh-flow-auto-driving/')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/test2/.local/lib/python3.8/site-packages')

import pylot.utils
from zenoh_flow import Source
import time
import pickle


if not hasattr(sys, 'argv'):
    sys.argv = ['']

CENTER_CAMERA_LOCATION = pylot.utils.Location(1.3, 0.0, 1.8)
transform = pylot.utils.Transform(CENTER_CAMERA_LOCATION,
                                  pylot.utils.Rotation(pitch=-15))


class MyState:
    def __init__(self, cfg):
        print("init state")
        self.timestamp = 0
        self.obstacles_msg = cfg['obstacles_msg']
        self.frame_msg = cfg['frame_msg']
        print("timestamp: {}".format(self.timestamp))


class MySrc(Source):
    def initialize(self, configuration):
        print("initialize")
        return MyState(configuration)

    def finalize(self, state):
        print("finalize")
        return None

    def run(self, _ctx, state):
        print("run")
        print('state.timestamp', state.timestamp)
        obstacles_msg = pickle.load(open(state.obstacles_msg, 'rb'))
        frame_msg = pickle.load(open(state.frame_msg, 'rb'))
        msg = (
            state.timestamp, obstacles_msg, frame_msg)
        print("msg: {}".format(msg))
        state.timestamp += 1
        time.sleep(1)
        return pickle.dumps(msg)


def register():
    print("register")
    return MySrc
