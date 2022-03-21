
import sys
sys.path.append('/home/zenoh-flow-auto-driving/')
sys.path.append('/usr/lib/python3.8')
sys.path.append('/home/test2/.local/lib/python3.8/site-packages')
from zenoh_flow import Inputs, Outputs, Source
import time
import pickle
import cv2
import pylot.utils
from pylot.drivers.sensor_setup import DepthCameraSetup, RGBCameraSetup, \
    SegmentedCameraSetup
from pylot.perception.camera_frame import CameraFrame
from pylot.perception.messages import FrameMessage
if not hasattr(sys, 'argv'):
    sys.argv = ['']

CENTER_CAMERA_LOCATION = pylot.utils.Location(1.3, 0.0, 1.8)
transform = pylot.utils.Transform(CENTER_CAMERA_LOCATION,
                                  pylot.utils.Rotation(pitch=-15))
class MyState:
    def __init__(self, cfg):
        src = cv2.imread(cfg['image_path'])
        # dst = cv2.cvtColor(src, cv2.COLOR_BGR2RGB)
        self.value = src
        self.timestamp = 0
        self.center_camera_setup = RGBCameraSetup('center_camera',
                                                  self.value.shape[1],
                                                  self.value.shape[0], transform,
                                                  )
class MySrc(Source):
    def initialize(self, configuration):
        return MyState(configuration)

    def finalize(self, state):
        return None

    def run(self, _ctx, state):
        print('state.timestamp', state.timestamp)
        rgb_frame = CameraFrame(state.value, 'BGR', state.center_camera_setup)
        msg = FrameMessage(state.timestamp, rgb_frame)
        state.timestamp += 1
        time.sleep(1)
        return pickle.dumps(msg)

def register():
    return MySrc

if __name__=='__main__':
    config = {'fps': 1}
    camera = MySrc()
    state = camera.initialize(config)
    camera.run(None, state)
