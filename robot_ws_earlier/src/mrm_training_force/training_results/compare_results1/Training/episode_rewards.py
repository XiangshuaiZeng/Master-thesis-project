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

def draw_plot(file, label, color):
    data = np.loadtxt(file)
    iterations = np.arange(len(data)) + 1
    # avarage the data
    N = 15
    data = average_list(data, N)
    iterations = iterations[0: -N]
    plt.plot(iterations,data, color=color, label = label)

file_names = ['iteration_reward_torque_laser_00_1.txt','iteration_reward_torque_laser_02_1.txt', 'iteration_reward_torque_laser_06.txt', 'iteration_reward_torque_laser_04_1.txt', 'iteration_reward_torque_laser_08.txt']
labels = ['coeff as 0.0', 'coeff as 0.2','coeff as 0.4', 'coeff as 0.6', 'coeff as 0.8']
colors  = ['b','r','g','c','k']

# draw the plots
for file, label, color in zip(file_names, labels, colors):
    draw_plot(file, label, color)

plt.rcParams.update({'font.size': 12})
plt.legend()
ax = plt.subplot(111)
for item in ([ ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)

plt.xlabel('Iteration')
plt.ylabel('Reward')
plt.title('Average of cumulative rewards in every 20 iterations')
plt.show()
