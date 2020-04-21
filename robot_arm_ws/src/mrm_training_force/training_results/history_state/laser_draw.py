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
dis_end_effector_00 = np.loadtxt('avg_laser_history_1.txt')
episodes_00 = np.arange(len(dis_end_effector_00)) + 1

dis_end_effector_03 = np.loadtxt('avg_laser_history_2.txt')
episodes_03 = np.arange(len(dis_end_effector_03)) + 1


plt.plot(episodes_00, dis_end_effector_00, color=lighten_color('b', 0.4),lw = 2.5)
plt.plot(episodes_03, dis_end_effector_03, color=lighten_color('r', 0.4),lw = 2.5)


plt.rcParams.update({'font.size': 12})
plt.legend(['Mini laser', 'front laser'])

# plot the averaged curve
N = 100
dis_end_effector_00 = average_list(dis_end_effector_00, N)
dis_end_effector_03 = average_list(dis_end_effector_03, N)


episodes_00 = episodes_00[0: -N]
episodes_03 = episodes_03[0: -N]


plt.plot(episodes_00, dis_end_effector_00, color='b',lw = 1.5)
plt.plot(episodes_03, dis_end_effector_03, color='r',lw = 1.5)


ax = plt.subplot(111)

for item in ([ ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)

plt.xlabel('Episode')
plt.ylim([0.0,0.06])
plt.ylabel('Distance')
plt.title('Average of minimal distance from the end-effector to the pipe')
plt.show()
