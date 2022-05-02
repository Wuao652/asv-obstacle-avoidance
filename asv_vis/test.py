import numpy as np
import seaborn as sns
import pandas as pd
import matplotlib.pyplot as plt
with open('run-ob_sac_SAC_7-tag-rollout_ep_rew_mean.csv', 'r') as f:
    data = f.read().split()[2:]
    data = [i.split(",") for i in data]
    # data = data[3:]
    ppo = np.array(data, dtype=float).reshape((-1, 3))
with open('run-ob_submit_sac_SAC_4-tag-rollout_ep_rew_mean.csv', 'r') as f:
    data = f.read().split()[2:]
    data = [i.split(",") for i in data]
    # data = data[3:]
    sac = np.array(data, dtype=float).reshape((-1, 3))

sns.set_theme(style="darkgrid")
f, axs = plt.subplots(1, 1)
df1 = pd.DataFrame(dict(timesteps=ppo[:, 1],
                       reward=ppo[:, 2]))
sns.lineplot(x="timesteps", y="reward", data=df1, ax=axs)
df2 = pd.DataFrame(dict(timesteps=sac[:, 1],
                       reward=sac[:, 2]))

sns.lineplot(x="timesteps", y="reward", data=df2, ax=axs)
plt.legend(labels=["PPO", "SAC"], loc="lower right")

# plt.figure()
# plt.plot(ppo[:, 1]/1000, ppo[:, 2]/10000)
# plt.plot(sac[:, 1]/1000, sac[:, 2]/10000)
# plt.xlabel('timesteps(10^3)')
# plt.ylabel('reward(10^4)')
# plt.legend(["PPO", "SAC"], loc="lower right")
plt.show()