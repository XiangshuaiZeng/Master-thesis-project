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

def draw_plot(file_name, color):
    success_times = np.loadtxt(file_name)
    episodes = np.arange(len(success_times)) + 1
    N = 300
    success_times = average_list(success_times, N)
    episodes = episodes[0: -N]
    plt.plot(episodes, success_times,lw = 1.5, color=color)

file_names = ['success_times_pos_control_state_history1.txt']
colors = ['b','r']

for file_name, color in zip(file_names, colors):
    draw_plot(file_name, color)

plt.rcParams.update({'font.size': 12})
ax = plt.subplot(111)
for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)
plt.legend(['Position control'])
plt.title('Success rate during training within 120mm pipes')
plt.xlabel('Episode')
plt.ylabel('Success rate')
plt.show()
