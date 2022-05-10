import sys

sys.path.remove("/home/erdos/workspace/pylot")
# sys.path.append('/home/erdos/workspace/zenoh-flow/zenoh-flow-auto-driving-xf')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/erdos/.local/lib/python3.8/site-packages')

import pickle
import time
from zenoh_flow import Source

class MyState:
    def __init__(self, cfg):
        print("init state")
        self.timestamp = 0
        self.lane_msg = cfg['lane_msg']
        self.linear_prediction_msg = cfg['linear_prediction_msg']
        self.open_drive_msg = cfg['open_drive_msg']
        self.trajectory_msg = cfg['trajectory_msg']
        self.vehicle_transform = cfg['vehicle_transform']
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
        lane_msg = pickle.load(open(state.lane_msg, 'rb'))
        linear_prediction_msg = pickle.load(
            open(state.linear_prediction_msg, 'rb'))
        open_drive_msg = pickle.load(open(state.open_drive_msg, 'rb'))
        trajectory_msg = pickle.load(open(state.trajectory_msg, 'rb'))
        vehicle_transform = pickle.load(open(state.vehicle_transform, 'rb'))
        msg = (
            state.timestamp, lane_msg, linear_prediction_msg, open_drive_msg, trajectory_msg, vehicle_transform)
        print("msg: {}".format(msg))
        state.timestamp += 1
        time.sleep(1)
        return pickle.dumps(msg)


def register():
    print("register")
    return MySrc
