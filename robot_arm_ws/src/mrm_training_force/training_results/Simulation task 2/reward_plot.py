import numpy as np
import matplotlib.pyplot as plt

episode_rewards_ppo = np.loadtxt('episode_reward_turning_left_detecting_stuck2.txt')
episodes_ppo = np.arange(len(episode_rewards_ppo)) + 1

episode_rewards_ppo_position = np.loadtxt('episode_reward_position_control5.txt')
episodes_ppo_position = np.arange(len(episode_rewards_ppo_position)) + 1



plt.plot(episodes_ppo, episode_rewards_ppo, episodes_ppo_position, episode_rewards_ppo_position)
plt.legend(['PPO_d125_torque_control', 'PPO_d125_position_control'])
plt.title('Episode rewards under 125mm pipe with different angles')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.xlim([0,2100])
plt.savefig(fname='episode_rewards_ppo_adding_stuck_detection', format='svg')
plt.show()

# calculate the variance of the rewards
ppo_mean_torq = np.mean(episode_rewards_ppo[1000:2000])
ppo_var_torq = np.std(episode_rewards_ppo[1000:2000])

ppo_mean_pos = np.mean(episode_rewards_ppo_position[1000:2000])
ppo_var_pos = np.std(episode_rewards_ppo_position[1000:2000])

print 'ppo reward torque control: \n mean: %d std: %d' % (ppo_mean_torq, ppo_var_torq)

print 'ppo reward position control: \n mean: %d std: %d' % (ppo_mean_pos, ppo_var_pos)

