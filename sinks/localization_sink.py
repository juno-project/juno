import os
import pickle

from zenoh_flow import Sink
class MySink(Sink):
    def initialize(self, configuration):
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        data = pickle.loads(input.data)
        print("localization_sink : {}".format(data))
        # out_poss_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".")
        # out_poss_path = os.path.join(out_poss_path, "test_data/LocalizationOperator/output/pose/pose-5643.pkl")
        # out_data = pickle.load(open(out_poss_path, "rb"))
        # print("sink out_data : {}".format(out_data))
def register():
    return MySink
