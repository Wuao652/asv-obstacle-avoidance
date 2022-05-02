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

# robot_position = []
rospy.sleep(3)
rate = rospy.Rate(30)
robot_dic = {}
# target_goal = numpy.array([[13.2, -42.0, 0.0], 
#                            [22.6, -33.0, 0.0],
#                            [10.9, -24.3, 0.0],
#                            [23.0, -58.0, 0.0],
#                            [0.0, 0.0, 0.0]])

target_goal = numpy.array([[13.2, -42.0, 0.0], 
                           [22.6, -33.0, 0.0],
                           [10.9, -24.3, 0.0],
                           [23.0, -58.0, 0.0],
                           [18.6, -41.6, 0.0],
                           [4.1, -46.9, 0.0],
                           [-0.5, -33.2, 0.0],
                           [2.4, -58.4, 0.0],
                           [24.4, -61.4, 0.0],
                           [19.2, -23.2, 0.0],
                           [0.0, 0.0, 0.0],
                           [0.0, 0.0, 0.0]])

env.set_target(target_goal[0][0], target_goal[0][1], target_goal[0][2])
observation = env.reset()
# robot_position.append([observation[0], observation[1], observation[4]])

for epoch in range(11):
    r_pose = []
    for _ in range(100):
        action, _states = model.predict(observation, deterministic=True)
        observation, reward, done, info = env.step(action)
        r_pose.append([observation[0], observation[1], observation[4]])
        # robot_position.append([observation[0], observation[1], observation[4]])
        if done:
            print('good we finished!')
            env.set_target(target_goal[epoch+1][0], target_goal[epoch+1][1], target_goal[epoch+1][2])
            observation = env.reset()
            break
        rate.sleep()
    r_pose = numpy.array(r_pose)
    robot_dic[epoch] = r_pose
    
env.close()

# robot_position = numpy.array(robot_position)
# numpy.save('/home/dennis/asv_vis/sac_train_w=1.npy', robot_position)
with open('/home/dennis/asv_vis/sac_test.pkl', 'wb') as f:
    pickle.dump(robot_dic, f)
