#!/usr/bin/env python3
import rospy
import torch
import gym
import os
from stable_baselines3 import DQN
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 
from datetime import datetime

#set the script running path to the folder "results"
os.chdir('/home/dennis/catkin_ws/src/wamv_drl_ros/results/')
rospy.init_node('wamv_drl_dqn', anonymous=True, log_level=rospy.INFO)
start_time = datetime.now()

lr = rospy.get_param('/wamv/lr')
batch_size = rospy.get_param('/wamv/batch_size')
buffer_size = rospy.get_param('/wamv/buffer_size')
gamma = rospy.get_param('/wamv/gamma')
# episodes_training = rospy.get_param('/wamv/episodes_training')
target_update_interval = rospy.get_param('/wamv/target_update_interval')

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print("The current device is:")
print(device)
print(torch.cuda.get_device_name(device))


# init the environment
print('init the Environment ====>>>>>>>')
environment_name = rospy.get_param('/wamv/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)

n_observations = rospy.get_param('/wamv/n_observations')
n_actions = rospy.get_param('/wamv/n_actions')

# create the model
print("create the model ====>>>>>>>>>>")

model = DQN(policy="MlpPolicy", 
            env=env, 
            verbose=1,
            learning_rate=1e-3,
            batch_size=128,
            learning_starts=0,
            gamma=0.99,
            target_update_interval=250,
            train_freq=4,
            gradient_steps=1,
            exploration_fraction=0.12,
            exploration_final_eps=0.01,
            policy_kwargs=dict(net_arch=[256, 256]),
            device=device,
            tensorboard_log="tensorboard_logs/drl_dqn/")


model.learn(total_timesteps=10e4, log_interval=4)
model_name = "./trained_models/dqn_"+str(start_time)
model.save(model_name)
print('model saved first time!')
# del model # remove to demonstrate saving and loading

# model = DQN.load(model_name)

# rate = rospy.Rate(30)
# observation = env.reset()
# time_step_counter = 0
# while True:
#     time_step_counter +=1
#     # env.render()   #can't render with ros
#     action, _states = model.predict(obs, deterministic=True)
#     observation, reward, done, info = env.step(action)
#     if done:
#         print("Episode {} finished after {} timesteps".format(i_episode, t+1))
#         observation = env.reset()
#     if time_step_counter % 500 == 0:
#         print('Model saved after 5000 training timesteps!')
#         model.save(model_name)
#     rate.sleep()
env.close()
end_time = datetime.now()
print('======>>>>>>>>>>>>>>>>>>>')
print('Finish training the model')
print('Training time: ', end_time - start_time)





