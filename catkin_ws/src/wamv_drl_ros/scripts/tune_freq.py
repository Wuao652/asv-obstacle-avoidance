#!/usr/bin/env python3
import rospy
import torch
import gym
import os
import numpy
import copy
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 

#set the script running path to the folder "results"
os.chdir('/home/dennis/catkin_ws/src/wamv_drl_ros/results/')
rospy.init_node('wamv_dqn_test', anonymous=True, log_level=rospy.DEBUG)


# init the environment
print('init the Environment ====>>>>>>>')

rospy.set_param('/wamv/task_and_robot_environment_name', 'WamvNavTwoSetsBuoys-v1')
rospy.set_param('/wamv/ros_ws_abspath', '/home/dennis/simulation_ws')
rospy.set_param('/wamv/n_observations', '9')
rospy.set_param('/wamv/n_actions', '4')
rospy.set_param('/wamv/control_type', 'velocity')

n_observations = rospy.get_param('/wamv/n_observations')
n_actions = rospy.get_param('/wamv/n_actions')
environment_name = rospy.get_param('/wamv/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)


# create the model
print("load the model ====>>>>>>>>>>")


rate = rospy.Rate(30)
obs_collection = numpy.empty([0, 9])
observation = env.reset()
obs_collection = numpy.append(obs_collection, observation.reshape((-1, 9)), axis=0)
i = 0
while True:
    i += 1
    # env.render()   #can't render with ros
    # action, _states = model.predict(obs, deterministic=True)
    print('The current step is ', i)
    observation, reward, done, info = env.step(0)
    obs_collection = numpy.append(obs_collection, observation.reshape((-1, 9)), axis=0)
    print('The current action taken is ', i%4)
    if done:
        print("Done!!!!!!!!!!")
        observation = env.reset()
        i = 0
        break
    rate.sleep()
print(obs_collection)
numpy.save('./traj.npy', obs_collection)
env.close()
