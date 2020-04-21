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
	data *= 100
	iterations = np.arange(len(data)) + 1
	# avarage the data
	N = 20
	data = average_list(data, N)
	iterations = iterations[0: -N]
	plt.plot(iterations,data, color=color, label = label)

file_names = ['success_times_high_fidelity_15.txt', 'success_times_high_fidelity_15_1.txt']
labels = ['C_2 = 0.1', 'C_2 = 0.2']
colors  = ['b','r','g','c']

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
plt.ylabel('Success rate %')
plt.title('Success rate inside 120mm pipes during training')
plt.show()
