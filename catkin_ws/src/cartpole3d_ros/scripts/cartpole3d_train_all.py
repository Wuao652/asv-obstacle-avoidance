#!/usr/bin/env python3
import rospy
import gym
import os
import numpy as np
import pandas as pd
import time

from stable_baselines.common.policies import MlpPolicy, MlpLstmPolicy, MlpLnLstmPolicy
from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import A2C, ACKTR, DDPG, PPO1, PPO2, SAC, TRPO, TD3
from stable_baselines.deepq.policies import MlpPolicy as mlp_dqn
from stable_baselines.sac.policies import MlpPolicy as mlp_sac
from stable_baselines.ddpg.policies import MlpPolicy as mlp_ddpg
from stable_baselines.td3.policies import MlpPolicy as mlp_td3
from stable_baselines.ddpg.noise import NormalActionNoise, OrnsteinUhlenbeckActionNoise

from openai_ros.task_envs.cartpole_stay_up import stay_up
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 


os.chdir('~/catkin_ws/src/openai_examples/cartpole/cartpole3d/scripts/')

rospy.init_node('cartpole3d_train_all', anonymous=True, log_level=rospy.FATAL)

environment_name = rospy.get_param('/cartpole_v0/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)
env = DummyVecEnv([lambda: env])  # The algorithms require a vectorized environment to run


# The noise objects for TD3
n_actions = env.action_space.n
action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))

model_list = [
        A2C(MlpPolicy, env, verbose=1, tensorboard_log="../results/tensorboard_logs/A2C/"), 
        ACKTR(MlpPolicy, env, verbose=1, tensorboard_log="../results/tensorboard_logs/ACKTR/"), 
        PPO2(MlpPolicy, env, verbose=1, tensorboard_log="../results/tensorboard_logs/PPO2/"), 
        TRPO(MlpPolicy, env, verbose=1, tensorboard_log="../results/tensorboard_logs/TRPO/"),
]

algo_list = ['A2C', 'ACKTR', 'PPO2', 'TRPO']

training_time_list = []
for model, algo in zip(model_list, algo_list):
    print(model)

    start = time.time()
    model.learn(total_timesteps=100000)
    end = time.time()
    training_time_list.append((end-start)*1000)
    model.save("../results/trained_models/"+algo)


df = pd.DataFrame(list(zip(algo_list, training_time_list)), columns=['algo', 'train_time (ms)'])
df.to_csv('../results/train_time.csv', index=False)

env.close()

