#!/usr/bin/env python3
import rospy
import gym
import yaml
import os
# from openai_ros.task_envs.cartpole_stay_up import stay_up
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment 
from stable_baselines3 import SAC
import torch
from datetime import datetime

#set the script running path to the folder "results"
os.chdir('/home/dennis/catkin_ws/src/wamv_drl_ros/results/')
# init the python node 
rospy.init_node('ob_sac', anonymous=True, log_level=rospy.INFO)
start_time = datetime.now()
# load params
with open("/home/dennis/catkin_ws/src/wamv_drl_ros/config/ob_sac_params.yaml", 'r') as stream:
    param = yaml.safe_load(stream)
rospy.set_param('/wamv', param['wamv'])


# lr = rospy.get_param('/wamv/lr')
# batch_size = rospy.get_param('/wamv/batch_size')
# buffer_size = rospy.get_param('/wamv/buffer_size')
# gamma = rospy.get_param('/wamv/gamma')
# # episodes_training = rospy.get_param('/wamv/episodes_training')
# target_update_interval = rospy.get_param('/wamv/target_update_interval')

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
print("The current device is:")
print(device)
print(torch.cuda.get_device_name(device))


# init the environment
print('init the Environment ====>>>>>>>')
environment_name = rospy.get_param('/wamv/task_and_robot_environment_name')
env = StartOpenAI_ROS_Environment(environment_name)
# 'WamvNavObstacles-v1'

# create the model
print("create the model ====>>>>>>>>>>")

model = SAC(policy="MlpPolicy", 
            env=env, 
            verbose=1,
            learning_rate=1e-3,
            batch_size=256,
            buffer_size=10000, 
            learning_starts=0,
            gamma=0.99,
            train_freq=1,
            gradient_steps=1,
            policy_kwargs=dict(net_arch=[400, 300]),
            device=device,
            tensorboard_log="./tensorboard_logs/ob_submit_sac/")





# model.learn(total_timesteps=1e5, log_interval=4)
model.learn(total_timesteps=1e5, log_interval=4)
model_name = "./trained_models/submit_sac_"+str(start_time)
model.save(model_name)
print('model saved first time!')

env.close()
end_time = datetime.now()
print('======>>>>>>>>>>>>>>>>>>>')
print('Finish training the model')
print('Training time: ', end_time - start_time)





