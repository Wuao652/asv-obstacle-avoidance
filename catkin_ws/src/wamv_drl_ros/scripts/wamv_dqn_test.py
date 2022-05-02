#!/usr/bin/env python3
import rospy
import torch
import gym
import os
from stable_baselines3 import DQN
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 

#set the script running path to the folder "results"
os.chdir('/home/dennis/catkin_ws/src/wamv_drl_ros/results/')
rospy.init_node('wamv_dqn_test', anonymous=True, log_level=rospy.INFO)


# init the environment
print('init the Environment ====>>>>>>>')
environment_name = rospy.get_param('/wamv/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)


n_observations = rospy.get_param('/wamv/n_observations')
n_actions = rospy.get_param('/wamv/n_actions')

# create the model
print("load the model ====>>>>>>>>>>")

model = DQN.load("trained_models/dqn_2022-02-17 21:59:39.941564")

# del model # remove to demonstrate saving and loading

# model = DQN.load("trained_models/dqn_temp")
rospy.sleep(5)
rate = rospy.Rate(30)

obs = env.reset()
while True:
    # env.render()   #can't render with ros
    action, _states = model.predict(obs, deterministic=True)
    observation, reward, done, info = env.step(action)
    if done:
        print('good we finished!')
        #print("Episode {} finished after {} timesteps".format(i_episode, t+1))
        observation = env.reset()
    rate.sleep()

env.close()
