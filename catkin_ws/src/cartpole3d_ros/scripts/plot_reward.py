from pathlib import Path
import pandas as pd 
import matplotlib.pyplot as plt
from numpy import genfromtxt
import numpy as np
from scipy.signal import savgol_filter

file_list = Path('../results/discounted_reward_vs_steps').glob('**/*.csv')
file_list = sorted(file_list)
print(file_list)

label_list = ['A2C', 'ACKTR', 'PPO2', 'TRPO']

def movingaverage(interval, window_size):
    window = np.ones(int(window_size))/float(window_size)
    return np.convolve(interval, window, 'same')


plt.figure(figsize=(10, 5))

score_list = []

for filename, lab in zip(file_list, label_list):
    print(filename)

    X = genfromtxt(filename, delimiter=',', skip_header=1)
    print(X)

    # smooth signal
    # X_av = movingaverage(X[:, 2], 50)
    # X_av = savgol_filter(X[:, 2], 31, 3)
    X_av = savgol_filter(X[:, 2], 21, 3)
    # plt.plot(X[:, 1], X[:, 2], label=lab)
    plt.plot(X[:, 1], X_av, label=lab)


    score_list.append(sum(X[:, 2]) / X[-1, 1])

plt.xlabel('nb steps')
plt.ylabel('dicounted reward')
plt.legend()
# # plt.show()
plt.savefig('../results/plots/discounted_rewards_vs_steps.png', dpi=500)


df = pd.read_csv('../results/train_time.csv')
print(score_list)

df['score'] = score_list
df['train_time (s)'] = df['train_time (ms)']/1000
print(df)

df.plot.bar(x='algo', y='score', rot=0)
plt.savefig('plots/score_vs_algo.png', dpi=500)

df.plot.bar(x='algo', y='train_time (s)', rot=0)
plt.savefig('../results/plots/traintime_vs_algo.png', dpi=500)
# plt.show()


