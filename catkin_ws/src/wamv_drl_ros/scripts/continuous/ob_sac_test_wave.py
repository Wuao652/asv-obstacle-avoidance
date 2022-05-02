#!/usr/bin/env python3
import rospy
import torch
import gym
import yaml
import os
from stable_baselines3 import SAC
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 
from datetime import datetime
import numpy
import pickle

#set the script running path to the folder "results"
os.chdir('/home/dennis/catkin_ws/src/wamv_drl_ros/results/')
rospy.init_node('ob_sac_test', anonymous=True, log_level=rospy.INFO)

# load params
with open("/home/dennis/catkin_ws/src/wamv_drl_ros/config/ob_sac_params.yaml", 'r') as stream:
    param = yaml.safe_load(stream)
rospy.set_param('/wamv', param['wamv'])

# init the environment
print('init the Environment ====>>>>>>>')
environment_name = rospy.get_param('/wamv/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)


n_observations = rospy.get_param('/wamv/n_observations')
n_actions = rospy.get_param('/wamv/n_actions')
device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")


# create the model
print("load the model ====>>>>>>>>>>")
# model = SAC.load("trained_models/sac_test", env=env, device=device, print_system_info=True)

model = SAC.load("trained_models/submit_sac_2022-04-15 20:30:35.719727", print_system_info=True)
# del model # remove to demonstrate saving and loading

robot_position = []
rospy.sleep(3)
rate = rospy.Rate(30)
observation = env.reset()
robot_position.append([observation[0], observation[1], observation[4]])

for epoch in range(1):
    for _ in range(100):
        action, _states = model.predict(observation, deterministic=True)
        observation, reward, done, info = env.step(action)
        robot_position.append([observation[0], observation[1], observation[4]])
        if done:
            print('good we finished!')
            observation = env.reset()
            break
        rate.sleep()
   
env.close()

# robot_position = numpy.array(robot_position)
# numpy.save('/home/dennis/asv_vis/sac_train_w=1.npy', robot_position)
# with open('/home/dennis/asv_vis/sac_test_p=0.pkl', 'wb') as f:
#     pickle.dump(robot_dic, f)
