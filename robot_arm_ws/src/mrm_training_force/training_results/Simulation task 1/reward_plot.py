import numpy as np
import matplotlib.pyplot as plt

episode_rewards_ppo = np.loadtxt('episode_reward_turning_left_various_angles_ppo4.txt')
episodes_ppo = np.arange(len(episode_rewards_ppo)) + 1

episode_rewards_trpo = np.loadtxt('episode_reward_turning_left_various_angles_ppo5.txt')
episodes_trpo = np.arange(len(episode_rewards_trpo)) + 1


plt.plot(episodes_ppo, episode_rewards_ppo, episodes_trpo, episode_rewards_trpo)
plt.legend(['PPO_d140', 'PPO_d125'])
plt.title('Episode rewards under 140mm pipe with different angles')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.xlim([0,2100])
plt.savefig(fname='episode_rewards_trpo vs ppo.svg', format='svg')
plt.show()



# calculate the variance of the rewards
ppo_mean = np.mean(episode_rewards_ppo[1000:2000])
ppo_var = np.std(episode_rewards_ppo[1000:2000])

trpo_mean = np.mean(episode_rewards_trpo[1000:2000])
trpo_var = np.std(episode_rewards_trpo[1000:2000])

print 'trpo reward: \n mean: %d std: %d' % (trpo_mean, trpo_var)
print 'ppo reward: \n mean: %d std: %d' % (ppo_mean, ppo_var)



