import pickle

file = pickle.load(open("/home/wangjl/project/universal_manipulation_interface/example_demo_session/dataset_plan.pkl", "rb"))
print(file[0]["episode_timestamps"])