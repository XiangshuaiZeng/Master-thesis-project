import numpy as np
import matplotlib.pyplot as plt

def lighten_color(color, amount=0.5):
    """
    Lightens the given color by multiplying (1-luminosity) by the given amount.
    Input can be matplotlib color string, hex string, or RGB tuple.

    Examples:
    >> lighten_color('g', 0.3)
    >> lighten_color('#F034A3', 0.6)
    >> lighten_color((.3,.55,.1), 0.5)
    """
    import matplotlib.colors as mc
    import colorsys
    try:
        c = mc.cnames[color]
    except:
        c = color
    c = colorsys.rgb_to_hls(*mc.to_rgb(c))
    return colorsys.hls_to_rgb(c[0], 1 - amount * (1 - c[1]), c[2])

def average_list(array, N):
	average_array = [np.mean(array[i:i+N]) for i in range(0, len(array)-N)]
	return average_array

# plot the original curves
epi_reward_00 = np.loadtxt('episode_reward_torque_laser_00.txt')
episodes_00 = np.arange(len(epi_reward_00)) + 1

epi_reward_03 = np.loadtxt('episode_reward_torque_laser_03.txt')
episodes_03 = np.arange(len(epi_reward_03)) + 1

epi_reward_08 = np.loadtxt('episode_reward_torque_laser_08.txt')
episodes_08 = np.arange(len(epi_reward_08)) + 1

#epi_reward_n2 = np.loadtxt('episode_reward_torque_laser__-2.txt')
#episodes_n2 = np.arange(len(epi_reward_n2)) + 1

#epi_reward_n5 = np.loadtxt('episode_reward_torque_laser__-5.txt')
#episodes_n5 = np.arange(len(epi_reward_n5)) + 1

epi_reward_n12 = np.loadtxt('episode_reward_torque_laser__-12.txt')
episodes_n12 = np.arange(len(epi_reward_n12)) + 1

plt.plot(episodes_00, epi_reward_00, color=lighten_color('b', 0.3),lw = 2.5)
plt.plot(episodes_08, epi_reward_08, color=lighten_color('r', 0.3),lw = 2.5)
plt.plot(episodes_03, epi_reward_03, color=lighten_color('g', 0.3),lw = 2.5)
#plt.plot(episodes_n2, epi_reward_n2, color=lighten_color('c', 0.3),lw = 2.5)
#plt.plot(episodes_n5, epi_reward_n5, color=lighten_color('m', 0.3),lw = 2.5)
plt.plot(episodes_n12, epi_reward_n12, color=lighten_color('k', 0.3),lw = 2.5)

# plot the averaged curve
N = 100
epi_reward_00 = average_list(epi_reward_00, N)
epi_reward_03 = average_list(epi_reward_03, N)
epi_reward_08 = average_list(epi_reward_08, N)
#epi_reward_n2 = average_list(epi_reward_n2, N)
#epi_reward_n5 = average_list(epi_reward_n5, N)
epi_reward_n12 = average_list(epi_reward_n12, N)


episodes_00 = episodes_00[0: -N]
episodes_03 = episodes_03[0: -N]
episodes_08 = episodes_08[0: -N]
#episodes_n2 = episodes_n2[0: -N]
#episodes_n5 = episodes_n5[0: -N]
episodes_n12 = episodes_n12[0: -N]


plt.plot(episodes_00, epi_reward_00, color='b',lw = 1.5, label='No reward')
plt.plot(episodes_08, epi_reward_08, color='r',lw = 1.5, label='coff as 0.3')
plt.plot(episodes_03, epi_reward_03, color='g',lw = 1.5, label='coff as 0.8')
#plt.plot(episodes_n2, epi_reward_n2, color='c',lw = 1.5, label='sparse reward as -2')
#plt.plot(episodes_n5, epi_reward_n5, color='m',lw = 1.5, label='sparse reward as -5')
plt.plot(episodes_n12, epi_reward_n12, color='k',lw = 1.5, label='coff as 1.2')

plt.rcParams.update({'font.size': 12})
plt.legend()


ax = plt.subplot(111)
for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)

plt.title('Average reward for each episode during training')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.ylim([-30, 0])
plt.show()
