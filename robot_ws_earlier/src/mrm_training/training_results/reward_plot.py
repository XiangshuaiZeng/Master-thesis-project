import numpy as np
import matplotlib.pyplot as plt

episode_rewards_ppo = np.loadtxt('episode_reward_position_control7.txt')
episodes_ppo = np.arange(len(episode_rewards_ppo)) + 1


plt.plot(episodes_ppo, episode_rewards_ppo)
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.show()






