
import sys

sys.path.append('/home/zenoh-flow-auto-driving/')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/test2/.local/lib/python3.8/site-packages')

from pylot.drivers.sensor_setup import RGBCameraSetup
import cv2
import pickle
import time
from zenoh_flow import Source
import pylot.utils

if not hasattr(sys, 'argv'):
    sys.argv = ['']

CENTER_CAMERA_LOCATION = pylot.utils.Location(1.3, 0.0, 1.8)
transform = pylot.utils.Transform(CENTER_CAMERA_LOCATION,
                                  pylot.utils.Rotation(pitch=-15))


class MyState:
    def __init__(self, cfg):
        print("init state")
        self.timestamp = 0
        self.depth_msg_path=cfg['depth_msg']
        self.obstacles_msg_path=cfg['obstacles_msg']
        self.vehicle_transform_path=cfg['vehicle_transform']
        src = cv2.imread(cfg['image_path'])
        self.value = src
        self.center_camera_setup = RGBCameraSetup('center_camera',
                                                  self.value.shape[1],
                                                  self.value.shape[0], transform,
                                                  )
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
        depth_msg=pickle.load(open(state.depth_msg_path,'rb'))
        obstacles_msg=pickle.load(open(state.obstacles_msg_path,'rb'))
        vehicle_transform=pickle.load(open(state.vehicle_transform_path,'rb'))
        msg = (
            state.timestamp, depth_msg, obstacles_msg, vehicle_transform, state.center_camera_setup)
        print("msg: {}".format(msg))
        state.timestamp += 1
        # print(pickle.dumps(msg))
        time.sleep(1)
        return pickle.dumps(msg)


def register():
    print("register")
    return MySrc
