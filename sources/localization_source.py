import os
import sys
sys.path.append('/home/erdos/zenoh-flow-auto-driving/')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/test2/.local/lib/python3.8/site-packages')
from zenoh_flow import Source
import time
import pickle
import pylot.utils

if not hasattr(sys, 'argv'):
    sys.argv = ['']

CENTER_CAMERA_LOCATION = pylot.utils.Location(1.3, 0.0, 1.8)
transform = pylot.utils.Transform(CENTER_CAMERA_LOCATION,
                                  pylot.utils.Rotation(pitch=-15))
class MyState:
    def __init__(self, cfg):
        self.gnss_path = cfg['gnss_path']
        self.imu_path = cfg['imu_path']
        self.pose_path = cfg['pose_path']
        self.timestamp = 0

class MySrc(Source):
    def initialize(self, configuration):
        return MyState(configuration)

    def finalize(self, state):
        return None

    def run(self, _ctx, state):
        print('state.timestamp', state.timestamp)
        gnss = pickle.load(open(state.gnss_path, "rb"))
        imu = pickle.load(open(state.imu_path, "rb"))
        pose = pickle.load(open(state.pose_path, "rb"))

        state.timestamp += 1
        time.sleep(1)
        result = {"gnss": gnss, "imu": imu, "pose": pose, "timestamp": state.timestamp}
        return pickle.dumps(result)

def register():
    return MySrc

if __name__=='__main__':
    project_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".")
    # project_path = os.path.abspath(os.path.dirname(project_path) + os.path.sep + ".")
    gnss_path = os.path.join(project_path, "test_data/LocalizationOperator/input/gnss/gnss-5643.pkl")
    imu_path = os.path.join(project_path, "test_data/LocalizationOperator/input/imu/imu-5643.pkl")
    pose_path = os.path.join(project_path, "test_data/LocalizationOperator/input/pose/pose-5643.pkl")
    print("gnss_path:" + gnss_path)
    print("imu_path:" + imu_path)
    print("pose_path:" + pose_path)


    config = {'gnss_path': gnss_path, 'imu_path': imu_path, "pose_path": pose_path}
    camera = MySrc()
    state = camera.initialize(config)
    (gnss, imu, poss) = camera.run(None, state)

    print(gnss)
    print(imu)
    print(poss)

