#!/usr/bin/env python3
import rospy
import gym
import yaml
# from openai_ros.task_envs.cartpole_stay_up import stay_up
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 
# init the python node 
rospy.init_node('ob_random', anonymous=True, log_level=rospy.INFO)
# load params
with open("/home/dennis/catkin_ws/src/wamv_drl_ros/config/ob_random_params.yaml", 'r') as stream:
    param = yaml.safe_load(stream)
rospy.set_param('/wamv', param['wamv'])

environment_name = rospy.get_param('/wamv/task_and_robot_environment_name')
# 'WamvNavObstacles-v1'
print('init Env')

env = StartOpenAI_ROS_Environment(environment_name)
print('ok Env')
env.reset()

rate = rospy.Rate(30)

for i_episode in range(100):
    observation = env.reset()
    for t in range(100):
        # env.render()   #can't render with ros
        action = env.action_space.sample()
        # action = [0, 0]
        observation, reward, done, info = env.step(action)
        if done:
            print("Episode {} finished after {} timesteps".format(i_episode, t+1))
            break
        rate.sleep()

env.close()


