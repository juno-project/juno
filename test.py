import os.path
import pickle

# test_data/LaneDetectionOperator/output/lane_msg-6871.pkl
# control_msg = pickle.load(open("/home/erdos/workspace/zenoh-flow-auto-driving/test_data/CarlaOperator/input/ControlMessage.pkl", "rb"))
# control_msg = pickle.load(open("/home/erdos/workspace/zenoh-flow-auto-driving/test_data/CarlaOperator/input/ControlMessage.pkl", "rb"))
control_msg = pickle.load(open("/home/erdos/workspace/zenoh-flow-auto-driving/test_data/LaneDetectionOperator/output/lane_msg-6871.pkl", "rb"))

print(control_msg)
print(control_msg.data)
print(control_msg.data[0].left_markings)
print(control_msg.data[0].right_markings)


