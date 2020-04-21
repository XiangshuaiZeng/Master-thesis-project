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
epi_reward_00 = np.loadtxt('episode_reward_pos_control_state_history1.txt')
episodes_00 = np.arange(len(epi_reward_00)) + 1

plt.plot(episodes_00, epi_reward_00, color=lighten_color('b', 0.4),lw = 2.5)


plt.rcParams.update({'font.size': 12})
plt.legend(['Position control'])

# plot the averaged curve
N = 100
epi_reward_00 = average_list(epi_reward_00, N)

episodes_00 = episodes_00[0: -N]

plt.plot(episodes_00, epi_reward_00, color='b',lw = 1.5)


ax = plt.subplot(111)
for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)

plt.title('Average reward for each episode during training')
plt.xlabel('Episode')
plt.ylabel('Reward')
plt.ylim([-80, 0])
plt.show()
