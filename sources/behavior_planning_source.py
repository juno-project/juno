import os
import sys
sys.path.append('/home/erdos/zenoh-flow-auto-driving/')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/test2/.local/lib/python3.8/site-packages')
from zenoh_flow import Source
import time
import pickle

if not hasattr(sys, 'argv'):
    sys.argv = ['']


class MyState:
    def __init__(self, cfg):
        self.open_drive_path = cfg['open_drive_path']
        self.vehicle_transform_path = cfg['vehicle_transform_path']
        self.timestamp = 0

class MySrc(Source):
    def initialize(self, configuration):
        return MyState(configuration)

    def finalize(self, state):
        return None

    def run(self, _ctx, state):
        print('state.timestamp', state.timestamp)
        open_drive = pickle.load(open(state.open_drive_path, "rb"))
        vehicle_transform = pickle.load(open(state.vehicle_transform_path, "rb"))
        state.timestamp += 1
        time.sleep(1)
        result = {"open_drive": open_drive, "vehicle_transform": vehicle_transform, "timestamp": state.timestamp}
        return pickle.dumps(result)

def register():
    return MySrc

if __name__=='__main__':
    project_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + "..")
    # project_path = os.path.abspath(os.path.dirname(project_path) + os.path.sep + ".")
    open_drive_path = os.path.join(project_path, "test_data/BehaviorPlanningOperator/input/open_drive_msg-0.pkl")
    vehicle_transform_path = os.path.join(project_path, "test_data/BehaviorPlanningOperator/input/vehicle_transform-5855.pkl")
    print("open_drive_path:" + open_drive_path)
    print("vehicle_transform_path:" + vehicle_transform_path)

    config = {'open_drive_path': open_drive_path, 'vehicle_transform_path': vehicle_transform_path}
    camera = MySrc()
    state = camera.initialize(config)
    camera.run(None, state)


