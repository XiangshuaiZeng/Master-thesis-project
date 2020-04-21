import numpy as np
import matplotlib.pyplot as plt

#file_names = ['success_rate_state_history1_110.txt', 'success_rate_state_history2_110.txt']
file_names = ['success_rate_state_history1_110.txt', 'success_rate_state_history1_100.txt']
markers = ['o','v']
colors = ['b','r']

def draw_plot(file_name, marker, color):
    data = np.loadtxt(file_name)
    data = np.asarray(data) * 100
    angles = range(60, 126, 5)
    plt.plot(angles, data, marker = marker, color = color)


for file_name, marker, color in zip(file_names, markers, colors):
    draw_plot(file_name, marker, color)


plt.rcParams.update({'font.size': 12})
ax = plt.subplot(111)
for item in ([ax.title, ax.xaxis.label, ax.yaxis.label] +
             ax.get_xticklabels() + ax.get_yticklabels()):
    item.set_fontsize(12)

plt.ylabel('Success rate %')
plt.xlabel('Angles')
plt.legend(['110mm pipe', '100mm pipe'])
plt.title('Success rate of turning within 100 & 110mm pipes')
plt.show()
