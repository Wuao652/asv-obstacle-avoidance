#!/usr/bin/env python3
import rospy
import gym
import os

# from stable_baselines3.common.policies import MlpPolicy, MlpLstmPolicy, MlpLnLstmPolicy
# from stable_baselines3.common.policies import MlpPolicy
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3 import A2C

from openai_ros.task_envs.cartpole_stay_up import stay_up
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 


os.chdir('/home/dennis/catkin_ws/src/cartpole3d_ros/results/')

rospy.init_node('cartpole3d_A2C', anonymous=True, log_level=rospy.FATAL)

environment_name = rospy.get_param('/cartpole_v0/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)
env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run

# # Create the model
# model = A2C('MlpPolicy', env, verbose=1, tensorboard_log="tensorboard_logs/A2C_cartpole/")
# print(model)

# # TRAIN
# print('Start Model Training')
# print('---------------------------')
# model.learn(total_timesteps=100000)
# model.save("trained_models/A2C_temp")

# print('Finish training the model!')


# TEST
# model = model.load("trained_models/A2C")

# obs = env.reset()
# rate = rospy.Rate(30)

# for t in range(1000):
#     print(t)
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     rate.sleep()

# env.close()


