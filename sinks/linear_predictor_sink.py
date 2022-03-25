import os
import pickle

from zenoh_flow import Sink
class MySink(Sink):
    def initialize(self, configuration):
        print("sink init.............")
        return None

    def finalize(self, state):
        return None

    def run(self, _ctx, _state, input):
        data = pickle.loads(input.data)
        print("sink input_data : {}".format(data))
        out_poss_path = os.path.abspath(os.path.dirname(os.path.abspath(__file__)) + os.path.sep + ".")
        out_poss_path = os.path.join(out_poss_path, "test_data/LinearPredictorOperator/output/linear_prediction_msg-5724.pkl")
        out_data = pickle.load(open(out_poss_path, "rb"))
        print("sink out_data : {}".format(out_data))
def register():
    return MySink





