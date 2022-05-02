# CartPole3d
The Cart Pole problem is solved in 3D using Reinforcement Learning (RL).
The cartpole3d environment is implemented in ROS with the package [openai_ros](http://wiki.ros.org/openai_ros).
The RL algorithms used for training are DQN, A2C, ACKTR, PPO, TRPO implemented in the [stable-baselines](https://github.com/hill-a/stable-baselines) library.


## Installation
1. Install [ROS Melodic](http://wiki.ros.org/ROS/Installation)

2. Install the [openai_ros](http://wiki.ros.org/openai_ros) package.
```bash
cd ~/catkin_make/src/
git clone https://bitbucket.org/theconstructcore/openai_ros.git
cd ~/catkin_make
catkin_make
source devel/setup.bash
rosdep install openai_ros
```

3. Install Gym
```bash
pip install gym
```

4. Install [stable-baselines](https://github.com/hill-a/stable-baselines)

5. Install cartpole3d
```bash
cd ~/catkin_make/src/
git clone https://github.com/PierreExeter/cartpole3d_ros.git
cd ~/catkin_make
catkin_make
```

6. Make the scripts executable
```bash
chmod +x ~/catkin_make/src/cartpole3d/scripts/cartpole3d_random.py
chmod +x ~/catkin_make/src/cartpole3d/scripts/cartpole3d_dqn.py
chmod +x ~/catkin_make/src/cartpole3d/scripts/cartpole3d_trpo.py
chmod +x ~/catkin_make/src/cartpole3d/scripts/cartpole3d_train_all.py
chmod +x ~/catkin_make/src/cartpole3d/scripts/cartpole3d_enjoy_all.py
```

7. Create simulation workspace
```bash
mkdir -p ~/simluation_ws/src
cd ..
catkin_make
source devel/setup.bash
rospack profile
```

## Test the installation
```bash
roslaunch cartpole3d start_training_cartpole3d_random.launch 
```
You should see the cartpole executing random actions in Gazebo.
![cartpole](results/videos/cartpole.png)

## Perform training
```bash
roslaunch cartpole3d start_training_cartpole3d_train_all.launch 
```


## Visualise training metrics in Tensorboard
```bash
cd ~/catkin_make/src/cartpole3d/results/
tensorboard --logdir=A2C:tensorboard_logs/A2C/,ACKTR:tensorboard_logs/ACKTR/,PPO2:tensorboard_logs/PPO2/,TRPO:tensorboard_logs/TRPO/
```

## Observe trained agents
```bash
roslaunch cartpole3d start_training_cartpole3d_enjoy_all.launch 
```

## Plot the results
```bash
cd ~/catkin_make/src/cartpole3d/scripts/
python plot_reward.py
```

## Supported systems
Tested with:
 
- Ubuntu 18.04
- Python 2.7
- ROS Melodic
- Gazebo 9.12


