#!/usr/bin/env python3
import rospy
import gym
from openai_ros.task_envs.cartpole_stay_up import stay_up
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 

rospy.init_node('cartpole3d_random', anonymous=True, log_level=rospy.INFO)
environment_name = rospy.get_param('/cartpole_v0/task_and_robot_environment_name')
print('init Env')
env = StartOpenAI_ROS_Environment(environment_name)
print('ok Env')
env.reset()

rate = rospy.Rate(30)

for i_episode in range(20):
    observation = env.reset()
    for t in range(100):
        # env.render()   #can't render with ros
        action = env.action_space.sample()
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode {} finished after {} timesteps".format(i_episode, t+1))
            break
        rate.sleep()

env.close()


