import os
import sys

# sys.path.append('/home/erdos/zenoh-flow-auto-driving/')
# sys.path.append('/usr/lib/python3.8')
# sys.path.append('/home/test2/.local/lib/python3.8/site-packages')

sys.path.remove("/home/erdos/workspace/pylot")
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/erdos/.local/lib/python3.8/site-packages')
sys.path.append('/home/erdos/workspace/zenoh-flow-auto-driving')



from pylot import constant

from zenoh_flow import Source
import time
import pickle

if not hasattr(sys, 'argv'):
    sys.argv = ['']


class MyState:
    def __init__(self, cfg):
        self.timestamp = 0

class MySrc(Source):
    def initialize(self, configuration):
        return MyState(configuration)

    def finalize(self, state):
        return None

    def run(self, _ctx, state):
        state.timestamp = state.timestamp + 1
        time.sleep(1)
        result = {"data": state.timestamp}
        print("source ---------------{}".format(result))
        return pickle.dumps(result)

def register():
    return MySrc


