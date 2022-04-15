import sys

sys.path.append('/home/zenoh-flow-auto-driving/')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/test2/.local/lib/python3.8/site-packages')

from zenoh_flow import Source
import time
import pickle

class MyState:
    def __init__(self, cfg):
        print("init state")
        self.timestamp = 0
        self.vehicle_transform = cfg['vehicle_transform']
        self.waypoints_msg = cfg['waypoints_msg']
        print("timestamp: {}".format(self.timestamp)) 


class MySrc(Source):
    def initialize(self, configuration):
        print("initialize")
        return MyState(configuration)

    def finalize(self, state):
        print("finalize")
        return None

    def run(self, _ctx, state):
        print('state.timestamp', state.timestamp)
        vehicle_transform = pickle.load(open(state.vehicle_transform, 'rb'))
        waypoints_msg = pickle.load(open(state.waypoints_msg, 'rb'))
        msg = (
            state.timestamp, vehicle_transform, waypoints_msg)
        print("msg: {}".format(msg))
        state.timestamp += 1
        time.sleep(1)
        return pickle.dumps(msg)
        


def register():
    print("register")
    return MySrc
