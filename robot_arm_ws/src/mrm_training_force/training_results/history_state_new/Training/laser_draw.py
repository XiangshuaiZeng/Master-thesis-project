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

files_name = ['avg_laser_torque_history_00.txt', 'avg_laser_torque_history_10.txt', 'avg_laser_torque_history_15.txt']
labels = ['No reward on joint change', 'coeff as 1.0','coeff as 1.5']
colors  = ['b','r','g']

def draw_plot(file, label, color):
    data = np.loadtxt(file)
    iterations = np.arange(len(data)) + 1
    # avarage the data
    N = 20
    data = average_list(data, N)
    iterations = iterations[0: -N]

    plt.plot(iterations,data, color=color, label = label)

# draw the plots
for file, label, color in zip(files_name, labels, colors):
    draw_plot(file, label, color)

plt.rcParams.update({'font.size': 12})
plt.legend()
ax = plt.subplot(111)
for item in ([ ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)

plt.xlabel('Iteration')
plt.ylabel('Distance')
plt.title('Average of minimal distance from the end-effector to the pipe')
plt.show()
