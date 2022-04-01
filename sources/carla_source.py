import os
import sys
# sys.path.append('/home/erdos/zenoh-flow-auto-driving/')
# sys.path.append('/usr/lib/python3.8')
# sys.path.append('/home/test2/.local/lib/python3.8/site-packages')
sys.path.remove("/home/erdos/workspace/pylot")
from zenoh_flow import Source
import time
import pickle

if not hasattr(sys, 'argv'):
    sys.argv = ['']


class MyState:
    def __init__(self, cfg):
        print("source init .....................")
        self.control_path = cfg['control_path']
        self.timestamp = 0

class MySrc(Source):
    def initialize(self, configuration):
        return MyState(configuration)

    def finalize(self, state):
        return None

    def run(self, _ctx, state):
        print('state.timestamp', state.timestamp)
        control = pickle.load(open(state.control_path, "rb"))
        state.timestamp += 1
        time.sleep(1)
        return pickle.dumps(control)

def register():
    return MySrc

if __name__=='__main__':
    project_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + "..")
    # project_path = os.path.abspath(os.path.dirname(project_path) + os.path.sep + ".")
    carla_path = os.path.join(project_path, "test_data/CarlaOperator/input/ControlMessage.pkl")
    obj = pickle.load(open(carla_path, "rb"))
    print(obj)

