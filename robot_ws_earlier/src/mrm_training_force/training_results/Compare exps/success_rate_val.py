import numpy as np
import matplotlib.pyplot as plt

#file_names = ['success_rate00_110.txt', 'success_rate08_110.txt', 'success_rate03_110.txt']
file_names = ['success_rate_00_100.txt', 'success_rate_08_100.txt', 'success_rate_03_100.txt']
markers = ['o','v','8']
colors = ['b','r','m']

def draw_plot(file_name, marker, color):
    data = np.loadtxt(file_name)
    data = np.asarray(data) * 100
    angles = range(55, 126, 5)
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
plt.legend(['No reward', 'coff as 0.3', 'coff as 0.8', 'coff as 1.2'])
plt.title('Success rate of turning within 100mm pipes')
plt.show()
