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
        self.tracked_obstacles_path = cfg['tracked_obstacles_path']
        self.timestamp = 0

class MySrc(Source):
    def initialize(self, configuration):
        return MyState(configuration)

    def finalize(self, state):
        return None

    def run(self, _ctx, state):
        print('state.timestamp', state.timestamp)
        tracked_obstacles = pickle.load(open(state.tracked_obstacles_path, "rb"))
        state.timestamp += 1
        time.sleep(1)
        result = {"tracked_obstacles": tracked_obstacles, "timestamp": state.timestamp}
        return pickle.dumps(result)

def register():
    return MySrc

if __name__=='__main__':
    project_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + "..")
    # project_path = os.path.abspath(os.path.dirname(project_path) + os.path.sep + ".")
    tracked_obstacles_path = os.path.join(project_path, "test_data/LinearPredictorOperator/input/tracked_obstacles_msg-5724.pkl")
    print("tracked_obstacles_path:" + tracked_obstacles_path)

    config = {'tracked_obstacles_path': tracked_obstacles_path}
    camera = MySrc()
    state = camera.initialize(config)
    camera.run(None, state)


